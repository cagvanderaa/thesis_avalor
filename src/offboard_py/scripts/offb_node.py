#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


current_state = State()
current_pose_uav1 = PoseStamped()
current_pose_uav2 = PoseStamped()
projection_results = []

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb_uav1(msg):
    global current_pose_uav1
    current_pose_uav1 = msg

def pose_cb_uav2(msg):
    global current_pose_uav2
    current_pose_uav2 = msg

def project_point(cam_coords, angles, P):
    fx, fy = 800, 800
    cx, cy = 400, 400
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0, 0, 1]])
    
    
    
    print(angles)
    
    P_translated = P - cam_coords
    
    
    yaw, pitch, roll = angles
    
    # Rotation matrix around the x-axis (roll)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Rotation matrix around the y-axis (yaw)
    R_y = np.array([
        [np.cos(yaw), 0, np.sin(yaw)],
        [0, 1, 0],
        [-np.sin(yaw), 0, np.cos(yaw)]
    ])
    
    # Rotation matrix around the z-axis (pitch)
    R_z = np.array([
        [np.cos(pitch), -np.sin(pitch), 0],
        [np.sin(pitch), np.cos(pitch), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix using the 'yxz' order
    R_mat = R_y @ R_z @ R_x  # Correct order: 'yxz'

    # Rotation matrix for 90 degrees around the x-axis
    angle = np.pi / 2
    R_x_90 = np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])
    
    P_translated = P - cam_coords
    P_rotated_x = R_x_90 @ P_translated
    
    
    
    P_camera = R_mat @ P_rotated_x
    #if P_camera[2] <= 0:
        #return None, None
    P_image = K @ P_camera
    P_image /= P_image[2]
    #if 0 <= P_image[0] <= 800 and 0 <= P_image[1] <= 800:
    return P_image[:2], np.linalg.norm(P_translated)
    #else:
        #return None, None

if __name__ == "__main__":
    rospy.init_node("pursuer_node")

    namespace_uav1 = "/uav0/"
    namespace_uav2 = "/uav1/"
    rospy.loginfo(f"Namespace UAV1: {namespace_uav1}")
    rospy.loginfo(f"Namespace UAV2: {namespace_uav2}")

    state_sub = rospy.Subscriber(f"{namespace_uav1}mavros/state", State, callback=state_cb)
    local_pos_sub_uav1 = rospy.Subscriber(f"{namespace_uav1}mavros/local_position/pose", PoseStamped, callback=pose_cb_uav1)
    local_pos_sub_uav2 = rospy.Subscriber(f"{namespace_uav2}mavros/local_position/pose", PoseStamped, callback=pose_cb_uav2)

    local_pos_pub = rospy.Publisher(f"{namespace_uav1}mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service(f"{namespace_uav1}mavros/cmd/arming")
    arming_client = rospy.ServiceProxy(f"{namespace_uav1}mavros/cmd/arming", CommandBool)

    rospy.wait_for_service(f"{namespace_uav1}mavros/set_mode")
    set_mode_client = rospy.ServiceProxy(f"{namespace_uav1}mavros/set_mode", SetMode)
    
    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0  # Different starting position for target
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    yaw_angle = 0  # Initialize yaw angle

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        # Update yaw angle for a slow 360-degree turn
        yaw_angle += np.radians(1)  # Increment yaw by 1 degree in radians
        if yaw_angle > 2 * np.pi:
            yaw_angle -= 2 * np.pi  # Reset after a full turn

        quaternion = quaternion_from_euler(0, 0, yaw_angle)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        local_pos_pub.publish(pose)
    
        cam_coords = np.array([current_pose_uav1.pose.position.x, current_pose_uav1.pose.position.y, current_pose_uav1.pose.position.z])
        
        # Convert quaternion to Euler angles
        orientation_q = current_pose_uav1.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        angles = euler_from_quaternion(orientation_list)
        
        P = np.array([current_pose_uav2.pose.position.x, current_pose_uav2.pose.position.y, current_pose_uav2.pose.position.z])

        #rospy.loginfo("UAV1 attitude: yaw: {}, pitch: {}, roll: {}".format(angles[2], angles[1], angles[0]))

        uv, distance = project_point(cam_coords, angles, P)
        
        rospy.loginfo("Image: u: {}, v: {}".format(uv[0], uv[1]))

        if uv is not None and distance is not None:
            projection_results.append((tuple(uv), distance))
            rospy.loginfo(f"***** In Frame *****")

        rate.sleep()

