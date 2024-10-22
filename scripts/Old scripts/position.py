#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pid_controller import PIDController
from projector import Projector
from std_msgs.msg import Float32, Empty as EmptyMsg



# simulation settings and parameters: 
virtual_plane = True # Set to true to enable virtual plane
IOU = 1 # Set to 1 for perfect vision, and to 0.8 for realistic state of the art tracking performance
follow_distance = 1


class VisualServoController:
    def __init__(self):
        # Initialize current_state
        self.current_state = State()  # Ensures current_state is defined
        self.current_pose_uav1 = PoseStamped()
        self.current_pose_uav2 = PoseStamped()

        self.namespace_uav1 = "/uav0/"
        self.namespace_uav2 = "/uav1/"
        self.rate = rospy.Rate(30)

        self.projector = Projector()
        self.image_center = (400, 400)
        self.last_time = rospy.Time.now()

        self.pid_control_horizontal = PIDController(Kp=60, Ki=0, Kd=6)
        self.pid_control_vertical = PIDController(Kp=60.0, Ki=0, Kd=1)
        self.pid_control_distance = PIDController(Kp=60, Ki=0, Kd=15)

        self.K_forward = 0
        self.lock_time = 8.0  # Time (in seconds) to reach K_forward = 1
        self.target_in_sight = False  # Flag to track target visibility
        self.time_since_lock = 0  # Time since target lock
        self.previous_target_in_sight = False  # Track previous state

        self.current_velocity = None


        # Subscribers
        self.state_sub = rospy.Subscriber(f"{self.namespace_uav1}mavros/state", State, callback=self.state_cb)
        self.local_pos_sub_uav1 = rospy.Subscriber(f"{self.namespace_uav1}mavros/local_position/pose", PoseStamped, callback=self.pose_cb_uav1)
        self.local_pos_sub_uav2 = rospy.Subscriber(f"{self.namespace_uav2}mavros/local_position/pose", PoseStamped, callback=self.pose_cb_uav2)
        self.local_vel_sub = rospy.Subscriber(f"{self.namespace_uav1}mavros/local_position/velocity_local", TwistStamped, callback=self.velocity_cb)


        # Publishers
        self.local_pos_pub = rospy.Publisher(f"{self.namespace_uav1}mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Added publishers for errors and data
        self.vertical_error_projector_pub = rospy.Publisher("/uav0/vertical_error_projector", Float32, queue_size=10)
        self.pitch_angle_pub = rospy.Publisher("/uav0/pitch_angle", Float32, queue_size=10)
        self.vertical_position_diff_pub = rospy.Publisher("/uav0/vertical_position_diff", Float32, queue_size=10)
        self.time_log_pub = rospy.Publisher("/uav0/time_log", Float32, queue_size=10)
        self.error_pub = rospy.Publisher("/uav0/horizontal_error", Float32, queue_size=10)
        self.distance_error_pub = rospy.Publisher("/uav0/distance_error", Float32, queue_size=10)
        self.yaw_rate_pub = rospy.Publisher("/uav0/yaw_rate", Float32, queue_size=10)
        self.delta_forward_pub = rospy.Publisher("/uav0/delta_forward", Float32, queue_size=10)
        self.delta_h_pub = rospy.Publisher("/uav0/delta_h", Float32, queue_size=10)

        rospy.wait_for_service(f"{self.namespace_uav1}mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy(f"{self.namespace_uav1}mavros/cmd/arming", CommandBool)

        rospy.wait_for_service(f"{self.namespace_uav1}mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy(f"{self.namespace_uav1}mavros/set_mode", SetMode)


    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb_uav1(self, msg):
        self.current_pose_uav1 = msg

    def pose_cb_uav2(self, msg):
        self.current_pose_uav2 = msg

    def velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear

    def run(self):
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

        pose = PoseStamped()
        pose.pose.position.x = 30  
        pose.pose.position.y = -20
        pose.pose.position.z = 8

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
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")
                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()

            position_uav1 = np.array([self.current_pose_uav1.pose.position.x, self.current_pose_uav1.pose.position.y, self.current_pose_uav1.pose.position.z])
            orientation_q = self.current_pose_uav1.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(orientation_list)
            angles = (yaw, pitch, roll)

            position_uav2 = np.array([self.current_pose_uav2.pose.position.x, self.current_pose_uav2.pose.position.y, self.current_pose_uav2.pose.position.z])
            uv, distance = self.projector.project_point(position_uav1, angles, position_uav2, IOU, virtual_plane)
            
            if uv is not None and distance is not None:
                horizontal_error, vertical_error = self.projector.calculate_pixel_error(uv, self.image_center)
        
                current_time = rospy.Time.now()
                dt = (current_time - self.last_time).to_sec()
                self.last_time = current_time
                

                if self.current_velocity is not None:
                    vx = self.current_velocity.x
                    vy = self.current_velocity.y
                    vz = self.current_velocity.z

                    # Compute the absolute linear velocity
                    absolute_velocity = np.sqrt(vx**2 + vy**2 + vz**2)

                    # Log the actual velocity to the terminal
                    rospy.loginfo(f"Actual Linear Velocity: {absolute_velocity:.2f} m/s")

                # locking the target based on lock time
                if self.target_in_sight:
                    # If the target is newly acquired, start timin
                    self.time_since_lock += dt  # Increment lock time
                    #if self.K_forward < 1: 
                        #rospy.loginfo(f"Locking target in {self.lock_time - self.time_since_lock}")
                    self.K_forward = min(1, (self.time_since_lock / self.lock_time))
                
                self.target_in_sight = True

                delta_yaw = - self.pid_control_horizontal.update(horizontal_error / 400, dt)
                delta_h = self.pid_control_vertical.update(-vertical_error / 400, dt)
                delta_r_forward = self.K_forward * self.pid_control_distance.update(distance - follow_distance, dt)

                yaw += delta_yaw * dt
                position_uav1[0] += delta_r_forward * - np.sin(yaw) * dt
                position_uav1[1] += delta_r_forward * np.cos(yaw) * dt
                position_uav1[2] += delta_h * dt

                orientation_list = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

                pose.pose.position.x = position_uav1[0]
                pose.pose.position.y = position_uav1[1]
                pose.pose.position.z = position_uav1[2] 
                pose.pose.orientation.x = orientation_list[0]
                pose.pose.orientation.y = orientation_list[1]
                pose.pose.orientation.z = orientation_list[2]
                pose.pose.orientation.w = orientation_list[3]

                self.local_pos_pub.publish(pose)
                
                # Publish the necessary data for analysis
                self.vertical_error_projector_pub.publish(vertical_error)
                self.pitch_angle_pub.publish(pitch)
                self.vertical_position_diff_pub.publish(position_uav1[2] - position_uav2[2])
                self.time_log_pub.publish(current_time.to_sec())
                self.error_pub.publish(horizontal_error)
                self.distance_error_pub.publish(distance - follow_distance)
                self.yaw_rate_pub.publish(delta_yaw)
                self.delta_forward_pub.publish(delta_r_forward)
                self.delta_h_pub.publish(delta_h)

            else:

                # Target was lost, reset K_forward and time_since_lock
                self.target_in_sight = False
                self.K_forward = 0
                self.time_since_lock = 0

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
    try:
        controller.run()
    except KeyboardInterrupt:
        rospy.loginfo("Shutdown requested. Plotting results...")

