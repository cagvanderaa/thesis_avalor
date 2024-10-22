#!/usr/bin/env python3

import numpy as np
import rospy
import smach
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from offboard_py.cfg import VelocityConfigConfig

from pid_controller import PIDController
from projector import Projector


class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["TARGET_ACQUIRED", "CONTINUE_SEARCH"])
        
        # Subscribers
        self.local_pos_sub_uav1 = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.pose_cb_uav1)
        
        # Publishers
        self.vel_pub = rospy.Publisher("/uav0/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        
        # PID Controllers
        self.pid_control_hover = PIDController(Kp=1.0, Ki=0.0, Kd=0.5)
        
        # Retrieve parameters from the parameter server
        self.virtual_plane = rospy.get_param("pursuer/vision/virtual_plane", True)
        self.IOU = rospy.get_param("pursuer/vision/IOU", 1)
        
        # Initialize Dynamic Reconfigure Server
        self.srv = Server(VelocityConfigConfig, self.dynamic_reconfigure_callback, namespace="search_state")
        rospy.loginfo("Dynamic Reconfigure Server for Search Initialized.")
        
        self.set_initial_reconfigure_values()

        self.current_pose_uav1 = PoseStamped()
        self.rate = rospy.Rate(30)  # 10 Hz
        self.projector = Projector()

    def set_initial_reconfigure_values(self):
        # Pull the parameters from the parameter server (populated by the YAML file)
        initial_config = {
            'virtual_plane': rospy.get_param("pursuer/vision/virtual_plane", True),
            'IOU': rospy.get_param("pursuer/vision/IOU", 1.0)
        }

        # Update the dynamic reconfigure server with these values from the parameter server
        self.srv.update_configuration(initial_config)

    def dynamic_reconfigure_callback(self, config, level):
        rospy.loginfo(f"Reconfigure Request: {config}")
        self.IOU = config.IOU
        self.virtual_plane = config.virtual_plane
        return config

    def pose_cb_uav1(self, msg):
        self.current_pose_uav1 = msg

    def execute(self, userdata):
        while not rospy.is_shutdown():
            # Compute Ground Truth Projection (without noise)
            uv, distance = self.projector.project_point(
                self.IOU, self.virtual_plane, add_noise=False
            )

            if uv is not None and distance is not None:
                rospy.loginfo("Target acquired. Proceeding to FOLLOW state.")
                return "TARGET_ACQUIRED"

            # Velocity-based control to hover and rotate
            velocity_msg = PositionTarget()
            velocity_msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
            velocity_msg.type_mask = (
                PositionTarget.IGNORE_PX | 
                PositionTarget.IGNORE_PY | 
                PositionTarget.IGNORE_PZ | 
                PositionTarget.IGNORE_AFX | 
                PositionTarget.IGNORE_AFY | 
                PositionTarget.IGNORE_AFZ | 
                PositionTarget.IGNORE_YAW
            )
            
            hover_error = 8.0 - self.current_pose_uav1.pose.position.z  # Target height = 8 meters (adjust if needed)
            velocity_msg.velocity.z = self.pid_control_hover.update(hover_error, 1.0 / 30.0)  # Assuming 10 Hz update rate
            velocity_msg.velocity.x = 0  # Move forward to search for the target
            velocity_msg.velocity.y = 0
            velocity_msg.yaw_rate = 0.6  # Rotate until target comes into FOV

            self.vel_pub.publish(velocity_msg)
            #rospy.loginfo("Target not in sight. Hovering and rotating.")

            self.rate.sleep()

        return "CONTINUE_SEARCH"
