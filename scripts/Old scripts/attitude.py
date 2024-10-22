#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from projector import Projector
from std_msgs.msg import Float32, Empty as EmptyMsg
import matplotlib.pyplot as plt

class VisualServoController:
    def __init__(self):
        self.current_state = State()
        self.current_pose_uav1 = PoseStamped()
        self.current_pose_uav2 = PoseStamped()
        self.cvy_actual = 0.0  # Initialize the vertical velocity in camera frame

        self.namespace_uav1 = "/uav0/"
        self.namespace_uav2 = "/uav1/"
        self.rate = rospy.Rate(30)

        self.projector = Projector()
        self.image_center = (400, 400)
        self.last_time = rospy.Time.now()

        # Parameters from the paper or typical values
        self.k1 = 1  # Vertical error gain
        self.k2 = 0  # Pitch rate control gain
        self.k3 = 0     # Pitch angle control gain
        self.k4 = 0.51    # Thrust control gain
        self.k5 = 0  # Horizontal error gain
        self.k6 = 0     # Roll rate control gain
        self.m = 1.5    # Mass of the quadcopter in kg
        self.g = 9.81   # Gravitational constant
        self.max_thrust = 20.76  #25.47657  # Maximum thrust (empirically determined)

        self.state_sub = rospy.Subscriber(f"{self.namespace_uav1}mavros/state", State, callback=self.state_cb)
        self.local_pos_sub_uav1 = rospy.Subscriber(f"{self.namespace_uav1}mavros/local_position/pose", PoseStamped, callback=self.pose_cb_uav1)
        self.local_pos_sub_uav2 = rospy.Subscriber(f"{self.namespace_uav2}mavros/local_position/pose", PoseStamped, callback=self.pose_cb_uav2)
        self.vel_sub = rospy.Subscriber(f"{self.namespace_uav1}mavros/local_position/velocity_body", TwistStamped, callback=self.velocity_cb)  # Subscribe to velocity

        self.local_pos_pub = rospy.Publisher(f"{self.namespace_uav1}mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.attitude_pub = rospy.Publisher(f"{self.namespace_uav1}mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

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

        self.pitch_log = []
        self.added_thrust_log = []
        self.vertical_error_log = []
        self.force_log = []
        self.time_log = []

        rospy.on_shutdown(self.plot_results)

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb_uav1(self, msg):
        self.current_pose_uav1 = msg

    def pose_cb_uav2(self, msg):
        self.current_pose_uav2 = msg

    def velocity_cb(self, msg):
        self.cvy_actual = msg.twist.linear.z  # Use z-component as the vertical velocity in the camera frame

    def departure_callback(self, msg):
        rospy.loginfo("Target departure signal received")
        self.pursuit_started = True

    def run(self):
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

            position_uav1 = np.array([self.current_pose_uav1.pose.position.x, self.current_pose_uav1.pose.position.y, self.current_pose_uav1.pose.position.z])
            orientation_q = self.current_pose_uav1.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_list)
            angles = (yaw, pitch, roll)

            position_uav2 = np.array([self.current_pose_uav2.pose.position.x, self.current_pose_uav2.pose.position.y, self.current_pose_uav2.pose.position.z])
            uv, distance = self.projector.project_point(position_uav1, angles, position_uav2)

            if uv is not None and distance is not None:
                # Normalize the image errors
                horizontal_error, vertical_error = self.projector.calculate_pixel_error(uv, self.image_center)
                horizontal_error /= self.image_center[0]  # Normalizing
                vertical_error /= - self.image_center[1]  # Normalizing

                current_time = rospy.Time.now()
                dt = (current_time - self.last_time).to_sec()
                self.last_time = current_time

                # Calculate the desired pitch angle (theta_d)
                theta_d = max(-np.radians(30), np.arctan(vertical_error))

                # Calculate the pitch angle error
                e_theta = pitch - theta_d

                # Longitudinal Channel Control using normalized coordinates
                desired_cvy = self.k1 * vertical_error

                # Calculate added thrust term
                added_thrust = (self.m / np.cos(pitch)) * (self.k4 * (desired_cvy - self.cvy_actual))

                # Use the actual vertical velocity from the velocity callback
                c_omega_x = -self.k2 * vertical_error - self.k3 * e_theta
                f = (self.m / np.cos(pitch)) * self.g +  added_thrust

                # Normalize thrust (ensure it stays within 0-1 range)
                normalized_thrust = min(max(f / self.max_thrust, 0.0), 1.0)

                # Log values using rospy.loginfo
                rospy.loginfo(f"Pitch: {pitch}, Added Thrust: {added_thrust}, Vertical Error: {vertical_error}, "
                              f"Force: {f}, Normalized Thrust: {normalized_thrust}, e_theta: {e_theta}, theta_d: {theta_d}")

                # Store the values in lists for plotting later
                self.pitch_log.append(pitch)
                self.added_thrust_log.append(added_thrust)
                self.vertical_error_log.append(vertical_error)
                self.force_log.append(f)
                self.time_log.append(current_time.to_sec())

                # Lateral Channel Control using normalized coordinates
                c_omega_y = self.k5 * horizontal_error
                c_omega_z = self.k6 * roll  # Desired roll angle is zero

                # Publish attitude and thrust command
                attitude_target = AttitudeTarget()
                attitude_target.thrust = normalized_thrust
                attitude_target.body_rate.x = c_omega_x
                attitude_target.body_rate.y = c_omega_y
                attitude_target.body_rate.z = c_omega_z
                attitude_target.type_mask = AttitudeTarget.IGNORE_ATTITUDE

                self.attitude_pub.publish(attitude_target)

                # Publish the horizontal error
                self.error_pub.publish(horizontal_error)
                self.distance_error_pub.publish(distance)

            else:
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

        self.plot_results()

    def plot_results(self):
        plt.figure()

        plt.subplot(4, 1, 1)
        plt.plot(self.time_log, self.pitch_log, label='Pitch')
        plt.ylabel('Pitch (rad)')
        plt.legend()

        plt.subplot(4, 1, 2)
        plt.plot(self.time_log, self.added_thrust_log, label='Added Thrust')
        plt.ylabel('Added Thrust (N)')
        plt.legend()

        plt.subplot(4, 1, 3)
        plt.plot(self.time_log, self.vertical_error_log, label='Vertical Error')
        plt.ylabel('Vertical Error (normalized)')
        plt.legend()

        plt.subplot(4, 1, 4)
        plt.plot(self.time_log, self.force_log, label='Force f')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.legend()

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    rospy.init_node("pursuer_node")
    controller = VisualServoController()
    controller.run()
