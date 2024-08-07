#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from projector import Projector
from std_msgs.msg import Float32, Empty as EmptyMsg

class VisualServoController:
    def __init__(self):
        self.current_state = State()
        self.current_pose_uav1 = PoseStamped()
        self.current_pose_uav2 = PoseStamped()

        self.namespace_uav1 = "/uav0/"
        self.namespace_uav2 = "/uav1/"
        self.rate = rospy.Rate(30)

        self.projector = Projector()
        self.image_center = (400, 400)
        self.last_time = rospy.Time.now()

        # Parameters from the paper or typical values
        self.k1 = 0.02
        self.k2 = 0.01
        self.k4 = 3
        self.k5 = 0.01
        self.k6 = 5
        self.m = 1.5  # Mass of the quadcopter in kg
        self.g = 9.81  # Gravitational constant

        self.state_sub = rospy.Subscriber(f"{self.namespace_uav1}mavros/state", State, callback=self.state_cb)
        self.local_pos_sub_uav1 = rospy.Subscriber(f"{self.namespace_uav1}mavros/local_position/pose", PoseStamped, callback=self.pose_cb_uav1)
        self.local_pos_sub_uav2 = rospy.Subscriber(f"{self.namespace_uav2}mavros/local_position/pose", PoseStamped, callback=self.pose_cb_uav2)

        self.local_pos_pub = rospy.Publisher(f"{self.namespace_uav1}mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher("/uav0/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

        rospy.wait_for_service(f"{self.namespace_uav1}mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy(f"{self.namespace_uav1}mavros/cmd/arming", CommandBool)

        rospy.wait_for_service(f"{self.namespace_uav1}mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy(f"{self.namespace_uav1}mavros/set_mode", SetMode)

        # Publish the horizontal image error and distance error for analysis
        self.error_pub = rospy.Publisher("/uav0/horizontal_error", Float32, queue_size=10)
        self.distance_error_pub = rospy.Publisher("/uav0/distance_error", Float32, queue_size=10)

        # Subscribe to the departure signal
        self.departure_sub = rospy.Subscriber("/target_departure", EmptyMsg, self.departure_callback)
        self.pursuit_started = False

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb_uav1(self, msg):
        self.current_pose_uav1 = msg

    def pose_cb_uav2(self, msg):
        self.current_pose_uav2 = msg

    def departure_callback(self, msg):
        rospy.loginfo("Target departure signal received")
        self.pursuit_started = True

    def run(self):
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

        pose = PoseStamped()
        pose.pose.position.x = -2  # Different starting position for target
        pose.pose.position.y = -5
        pose.pose.position.z = 2

        # Send a few setpoints before starting
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(pose)
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()
        yaw_angle = 0  # Initialize yaw angle

        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")
                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()

            if not self.current_state.armed or self.current_state.mode != "OFFBOARD":
                rospy.loginfo("Waiting for arming and OFFBOARD mode")
                self.rate.sleep()
                continue

            #if not self.pursuit_started:
            #    rospy.loginfo("Waiting for target to depart")
            #    self.rate.sleep()
            #    continue

            position_uav1 = np.array([self.current_pose_uav1.pose.position.x, self.current_pose_uav1.pose.position.y, self.current_pose_uav1.pose.position.z])
            orientation_q = self.current_pose_uav1.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_list)
            angles = (yaw, pitch, roll)

            position_uav2 = np.array([self.current_pose_uav2.pose.position.x, self.current_pose_uav2.pose.position.y, self.current_pose_uav2.pose.position.z])
            uv, distance = self.projector.project_point(position_uav1, angles, position_uav2)

            if uv is not None and distance is not None:
                rospy.loginfo(f"***** In Frame *****")
                horizontal_error, vertical_error = self.projector.calculate_pixel_error(uv, self.image_center)

                current_time = rospy.Time.now()
                dt = (current_time - self.last_time).to_sec()
                self.last_time = current_time

                # Longitudinal Channel Control
                cvy = self.k1 * vertical_error / 400
                c_omega_x = -self.k2 * vertical_error / 400
                f = (self.m / np.cos(pitch)) * (self.k4 * (cvy - self.k1 * vertical_error / 400) + self.g)

                # Lateral Channel Control
                c_omega_y = self.k5 * horizontal_error / 400
                c_omega_z = self.k6 * (roll - 0)  # Desired roll angle is zero

                twist = TwistStamped()
                twist.twist.linear.x = 0
                twist.twist.linear.y = cvy
                twist.twist.linear.z = f
                twist.twist.angular.x = c_omega_x
                twist.twist.angular.y = c_omega_y
                twist.twist.angular.z = c_omega_z

                self.vel_pub.publish(twist)

                # Publish the horizontal error
                self.error_pub.publish(horizontal_error)
                self.distance_error_pub.publish(distance)

            else:
                rospy.loginfo("No target detected. Hovering.")

                # Update yaw angle for a slow 360-degree turn
                yaw_angle += np.radians(1)  # Increment yaw by 1 degree in radians
                if yaw_angle > 2 * np.pi:
                    yaw_angle -= 2 * np.pi  # Reset after a full turn

                quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_angle)
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                self.local_pos_pub.publish(pose)

            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("pursuer_node")
    controller = VisualServoController()
    controller.run()
