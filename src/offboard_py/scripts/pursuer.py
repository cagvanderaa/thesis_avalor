#!/usr/bin/env python3

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

current_pose_iris0 = None
current_pose_iris1 = None
projection_results = []

def model_states_cb(msg):
    global current_pose_iris0, current_pose_iris1

    try:
        iris0_index = msg.name.index('iris0')
        iris1_index = msg.name.index('iris1')
        current_pose_iris0 = msg.pose[iris0_index]
        current_pose_iris1 = msg.pose[iris1_index]
    except ValueError:
        rospy.logerr("UAV models iris0 and iris1 not found in Gazebo ModelStates")

def project_point(cam_coords, angles, P):
    fx, fy = 800, 800
    cx, cy = 400, 400
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0, 0, 1]])

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

    P_rotated_x = R_x_90 @ P_translated
    P_camera = R_mat @ P_rotated_x
    if P_camera[2] <= 0:
        return None, None
    P_image = K @ P_camera
    P_image /= P_image[2]
    return P_image[:2], np.linalg.norm(P_translated)

if __name__ == "__main__":
    rospy.init_node("projection_node")

    # Subscribe to the Gazebo ModelStates topic
    model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, callback=model_states_cb)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if current_pose_iris0 is None or current_pose_iris1 is None:
            rate.sleep()
            continue

        cam_coords = np.array([current_pose_iris0.position.x, current_pose_iris0.position.y, current_pose_iris0.position.z])

        # Convert quaternion to Euler angles
        orientation_q = current_pose_iris0.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        angles = (yaw, pitch, roll)

        P = np.array([current_pose_iris1.position.x, current_pose_iris1.position.y, current_pose_iris1.position.z])

        uv, distance = project_point(cam_coords, angles, P)

        if uv is not None and distance is not None:
            projection_results.append((tuple(uv), distance))
            rospy.loginfo(f"***** In Frame *****")
            rospy.loginfo("Image: u: {}, v: {}".format(uv[0], uv[1]))

        rate.sleep()

