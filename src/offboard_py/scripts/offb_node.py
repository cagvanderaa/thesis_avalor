#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pid_controller import PIDController
from projector import Projector

print("Python Path:", sys.path)

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
    projector = Projector()
    image_center = (400, 400)
    last_time = rospy.Time.now()
    
    #horizontal_pid = PIDController(Kp=10, Ki=0.1, Kd=0.05)
    #vertical_pid = PIDController(Kp=2.0, Ki=0.1, Kd=0.05)
    #distance_pid = PIDController(Kp=1.50, Ki=0.1, Kd=0.05)

    #pid_control = PIDController()


    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    
    orientation_list = tf.transformations.quaternion_from_euler(0, 0, 0.5* np.pi)

    
    pose = PoseStamped()
    pose.pose.position.x = 0  # Different starting position for target
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    pose.pose.orientation.w = orientation_list[0]
    pose.pose.orientation.x = orientation_list[1]
    pose.pose.orientation.y = orientation_list[2]
    pose.pose.orientation.z = orientation_list[3]

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

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_angle)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        local_pos_pub.publish(pose)
    
        cam_coords = np.array([current_pose_uav1.pose.position.x, current_pose_uav1.pose.position.y, current_pose_uav1.pose.position.z])

        orientation_q = current_pose_uav1.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_list)
        angles = (yaw, pitch, roll)

        P = np.array([current_pose_uav2.pose.position.x, current_pose_uav2.pose.position.y, current_pose_uav2.pose.position.z])

        uv, distance = projector.project_point(cam_coords, angles, P)

        if uv is not None and distance is not None:
            rospy.loginfo(f"***** In Frame *****")
            horizontal_error, vertical_error = projector.calculate_pixel_error(uv, image_center)
            rospy.loginfo("Horizontal error: {}, Vertical error: {}".format(horizontal_error, vertical_error))
            
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time
            
            #delta_yaw = - horizontal_pid.update(horizontal_error / 400, dt)
            #delta_h = vertical_pid.update(vertical_error / 400, dt)
            #delta_r_forward = distance_pid.update(distance, dt)
            #rospy.loginfo("Control Commands: delta_r_forward: {}, delta_h: {}, delta_yaw: {}".format(delta_r_forward, delta_h,delta_yaw))
            
            
	    
        rate.sleep()

