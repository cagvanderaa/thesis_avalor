#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from offboard_py.cfg import VelocityConfigConfig 
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))
from pid_controller import PIDController
from projector import Projector
from kalman_filter import KalmanFilterPredictor

class VisualServoController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("velocity_node")
        
        # Retrieve parameters from the parameter server
        self.virtual_plane = rospy.get_param("pursuer/vision/virtual_plane", True)
        self.kalman_filter = rospy.get_param("pursuer/vision/kalman_filter", True)
        self.IOU = rospy.get_param("pursuer/vision/IOU", 1)
        self.accuracy = rospy.get_param("pursuer/vision/accuracy", 100)
        self.vertical_offset = rospy.get_param("pursuer/vision/vertical_offset", 0)
        
        
        self.follow_distance = rospy.get_param("pursuer/control/follow_distance", 3)
        
        # Initialize PID controllers with parameters from YAML
        pid_horizontal_params = rospy.get_param("pursuer/control/pid_control_horizontal")
        pid_vertical_params = rospy.get_param("pursuer/control/pid_control_vertical")
        pid_distance_params = rospy.get_param("pursuer/control/pid_control_distance")
        
        self.pid_control_horizontal = PIDController(
            Kp=pid_horizontal_params["Kp"],
            Ki=pid_horizontal_params["Ki"],
            Kd=pid_horizontal_params["Kd"]
        )
        
        self.pid_control_vertical = PIDController(
            Kp=pid_vertical_params["Kp"],
            Ki=pid_vertical_params["Ki"],
            Kd=pid_vertical_params["Kd"]
        )
        
        self.pid_control_distance = PIDController(
            Kp=pid_distance_params["Kp"],
            Ki=pid_distance_params["Ki"],
            Kd=pid_distance_params["Kd"]
        )

        # Initialize Kalman Filter Predictor
        self.kalman_predictor = KalmanFilterPredictor(
            dt=1.0/30.0,  # Assuming a 30 Hz update rate
            process_noise_std=10, 
            measurement_noise_std=4
        )
        
        # Initialize Hover PID controller (remains unchanged)
        self.pid_control_hover = PIDController(Kp=1.0, Ki=0.0, Kd=0.5)
        
        # Initialize other variables
        self.current_state = State()
        self.current_pose_uav1 = PoseStamped()
        self.current_pose_uav2 = PoseStamped()
        self.current_velocity = None
        
        self.namespace_uav1 = "/uav0/"
        self.namespace_uav2 = "/uav1/"
        self.rate = rospy.Rate(30)
        
        self.projector = Projector()
        self.image_center = (400, 400)
        self.last_time = rospy.Time.now()
        
        self.K_forward = 0
        self.lock_time = 8.0  # Time (in seconds) to reach K_forward = 1
        self.target_in_sight = False
        self.time_since_lock = 0
        self.previous_target_in_sight = False
        
        # Subscribers
        self.state_sub = rospy.Subscriber(f"{self.namespace_uav1}mavros/state", State, self.state_cb)
        self.local_pos_sub_uav1 = rospy.Subscriber(f"{self.namespace_uav1}mavros/local_position/pose", PoseStamped, self.pose_cb_uav1)
        self.local_pos_sub_uav2 = rospy.Subscriber(f"{self.namespace_uav2}mavros/local_position/pose", PoseStamped, self.pose_cb_uav2)
        self.local_vel_sub = rospy.Subscriber(f"{self.namespace_uav1}mavros/local_position/velocity_local", TwistStamped, self.velocity_cb)
        
        # Publishers
        self.local_pos_pub = rospy.Publisher(f"{self.namespace_uav1}mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher(f"{self.namespace_uav1}mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        
        # Publishers for errors and data (similar to the position controller)
        self.vertical_error_projector_pub = rospy.Publisher("/uav0/vertical_error_projector", Float32, queue_size=10)
        self.pitch_angle_pub = rospy.Publisher("/uav0/pitch_angle", Float32, queue_size=10)
        self.vertical_position_diff_pub = rospy.Publisher("/uav0/vertical_position_diff", Float32, queue_size=10)
        self.time_log_pub = rospy.Publisher("/uav0/time_log", Float32, queue_size=10)
        self.error_pub = rospy.Publisher("/uav0/horizontal_error", Float32, queue_size=10)
        self.distance_error_pub = rospy.Publisher("/uav0/distance_error", Float32, queue_size=10)
        self.yaw_rate_pub = rospy.Publisher("/uav0/yaw_rate", Float32, queue_size=10)
        self.delta_forward_pub = rospy.Publisher("/uav0/delta_forward", Float32, queue_size=10)
        self.delta_h_pub = rospy.Publisher("/uav0/delta_h", Float32, queue_size=10)
        
        # Service proxies for arming and setting mode
        rospy.wait_for_service(f"{self.namespace_uav1}mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy(f"{self.namespace_uav1}mavros/cmd/arming", CommandBool)
        
        rospy.wait_for_service(f"{self.namespace_uav1}mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy(f"{self.namespace_uav1}mavros/set_mode", SetMode)
        
        # Initialize Dynamic Reconfigure Server
        self.srv = Server(VelocityConfigConfig, self.dynamic_reconfigure_callback)
        rospy.loginfo("Dynamic Reconfigure Server for VelocityController Initialized.")

        self.set_initial_reconfigure_values()
    
    def state_cb(self, msg):
        self.current_state = msg
    
    def pose_cb_uav1(self, msg):
        self.current_pose_uav1 = msg
    
    def pose_cb_uav2(self, msg):
        self.current_pose_uav2 = msg
    
    def velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear
    
    def set_initial_reconfigure_values(self):
        # Pull the parameters from the parameter server (populated by the YAML file)
        initial_config = {
            'virtual_plane': rospy.get_param("pursuer/vision/virtual_plane", True),
            'kalman_filter': rospy.get_param("pusuer/vision/kalman_filter", True),
            'IOU': rospy.get_param("pursuer/vision/IOU", 1.0),
            'accuracy': rospy.get_param("pursuer/vision/accuracy", 100),
            'vertical_offset': rospy.get_param("pursuer/vision/vertical_offset", 0.0),
            'pid_horizontal_Kp': rospy.get_param("pursuer/control/pid_control_horizontal/Kp", 1.0),
            'pid_horizontal_Ki': rospy.get_param("pursuer/control/pid_control_horizontal/Ki", 0.0),
            'pid_horizontal_Kd': rospy.get_param("pursuer/control/pid_control_horizontal/Kd", 0.0),
            'pid_vertical_Kp': rospy.get_param("pursuer/control/pid_control_vertical/Kp", 1.0),
            'pid_vertical_Ki': rospy.get_param("pursuer/control/pid_control_vertical/Ki", 0.0),
            'pid_vertical_Kd': rospy.get_param("pursuer/control/pid_control_vertical/Kd", 0.0),
            'pid_distance_Kp': rospy.get_param("pursuer/control/pid_control_distance/Kp", 1.0),
            'pid_distance_Ki': rospy.get_param("pursuer/control/pid_control_distance/Ki", 0.0),
            'pid_distance_Kd': rospy.get_param("pursuer/control/pid_control_distance/Kd", 0.0),
            'follow_distance': rospy.get_param("pursuer/control/follow_distance", 3)
        }

        # Update the dynamic reconfigure server with these values from the parameter server
        self.srv.update_configuration(initial_config)


    def dynamic_reconfigure_callback(self, config, level):
        """
        Callback function for dynamic reconfigure.
        Updates PID controller parameters, follow distance, and vision parameters based on user input.
        """
        rospy.loginfo(f"Reconfigure Request: {config}")
        
        # Update PID parameters for Horizontal Control
        self.pid_control_horizontal.Kp = config.pid_horizontal_Kp
        self.pid_control_horizontal.Ki = config.pid_horizontal_Ki
        self.pid_control_horizontal.Kd = config.pid_horizontal_Kd
        
        # Update PID parameters for Vertical Control
        self.pid_control_vertical.Kp = config.pid_vertical_Kp
        self.pid_control_vertical.Ki = config.pid_vertical_Ki
        self.pid_control_vertical.Kd = config.pid_vertical_Kd
        
        # Update PID parameters for Distance Control
        self.pid_control_distance.Kp = config.pid_distance_Kp
        self.pid_control_distance.Ki = config.pid_distance_Ki
        self.pid_control_distance.Kd = config.pid_distance_Kd
        
        # Update Follow Distance
        self.follow_distance = config.follow_distance
        rospy.loginfo(f"Updated Follow Distance: {self.follow_distance} m")
        
        # Update Vision Parameters
        self.virtual_plane = config.virtual_plane
        self.kalman_filter = config.kalman_filter
        self.IOU = config.IOU
        self.accuracy = config.accuracy
        self.vertical_offset = config.vertical_offset
        rospy.loginfo(f"Vision Params: Virtual Plane: {self.virtual_plane}, IOU: {self.IOU}, Accuracy: {self.accuracy}, Vertical Offset: {self.vertical_offset}")
        
        return config

    def run(self):
        # Wait until the drone is connected
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("Waiting for connection...")
            self.rate.sleep()
        
        # Set OFFBOARD mode
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'
        
        # Arm the drone
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        
        last_req = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD mode enabled")
                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()
            
            # Get current positions
            position_uav1 = np.array([
                self.current_pose_uav1.pose.position.x,
                self.current_pose_uav1.pose.position.y,
                self.current_pose_uav1.pose.position.z
            ])
            
            orientation_q = self.current_pose_uav1.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            roll, pitch, yaw = euler_from_quaternion(orientation_list)
            angles = (yaw, pitch, roll)
            
            position_uav2 = np.array([
                self.current_pose_uav2.pose.position.x,
                self.current_pose_uav2.pose.position.y,
                self.current_pose_uav2.pose.position.z
            ])
            
            # Compute Ground Truth Projection (without noise)
            uv_truth, distance_truth = self.projector.project_point(
                self.IOU, self.virtual_plane, add_noise=False
            )
            
            # Compute Measured Projection (with noise)
            uv_measured, distance_measured = self.projector.project_point(
                self.IOU, self.virtual_plane, add_noise=True
            )
            
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time
            
            velocity_msg = PositionTarget()
            
            # Set the coordinate frame to BODY_NED
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
            
            if uv_measured is not None and distance_measured is not None:
                horizontal_error, vertical_error = self.projector.calculate_pixel_error(uv_measured, self.image_center)
                
                if self.kalman_filter: 
                    # Initialize Kalman Filter with the first measurement
                    if not self.kalman_predictor.is_initialized:
                        self.kalman_predictor.initialize(np.array(uv_measured))
                    
                    # Update Kalman Filter with the latest measurement
                    measurement = np.array(uv_measured)
                    self.kalman_predictor.predict()
                    self.kalman_predictor.update(measurement)
                    
                    # Predict the next position (1 step ahead)
                    predicted_uv = self.kalman_predictor.get_predicted_position(steps_ahead=1)
                    
                    # Calculate errors based on predicted position
                    horizontal_error, vertical_error = self.projector.calculate_pixel_error(predicted_uv, self.image_center)

                

                
                # Locking the target based on lock time
                if self.target_in_sight:
                    self.time_since_lock += dt
                    self.K_forward = min(1, (self.time_since_lock / self.lock_time))
                else:
                    self.time_since_lock = 0
                    self.K_forward = 0
                
                # Update PID controllers using predicted errors
                delta_yaw = -self.pid_control_horizontal.update(horizontal_error / 400.0, dt)
                delta_h = self.pid_control_vertical.update(-(vertical_error - self.vertical_offset) / 400.0, dt)
                delta_r_forward = self.K_forward * self.pid_control_distance.update(distance_measured - self.follow_distance, dt)
                
                # Velocity-based control in BODY_NED
                velocity_msg.velocity.x = -delta_r_forward
                velocity_msg.velocity.z = delta_h
                velocity_msg.yaw_rate = delta_yaw
                
                self.vel_pub.publish(velocity_msg)
                self.target_in_sight = True
                
                # Publish the necessary data for analysis
                self.vertical_error_projector_pub.publish(vertical_error)
                self.pitch_angle_pub.publish(pitch)
                self.vertical_position_diff_pub.publish(position_uav1[2] - position_uav2[2])
                self.time_log_pub.publish(current_time.to_sec())
                self.error_pub.publish(horizontal_error)
                self.distance_error_pub.publish(distance_measured - self.follow_distance)
                self.yaw_rate_pub.publish(delta_yaw)
                self.delta_forward_pub.publish(delta_r_forward)
                self.delta_h_pub.publish(delta_h)
                
                if self.current_velocity is not None:
                    vx = self.current_velocity.x
                    vy = self.current_velocity.y
                    vz = self.current_velocity.z
                    
                    # Compute the absolute linear velocity
                    absolute_velocity = np.sqrt(vx**2 + vy**2 + vz**2)
                    
                    # Log the actual velocity to the terminal
                    rospy.loginfo(f"Actual Linear Velocity: {absolute_velocity:.2f} m/s")
                
                # **Log Ground Truth, Measured, and Predicted Projections**
                rospy.loginfo(f"Ground Truth UV: {uv_truth}, Measured UV: {uv_measured}, Predicted UV: {predicted_uv}")
                # rospy.loginfo(f"Ground Truth Distance: {distance_truth:.2f} m, Measured Distance: {distance_measured:.2f} m")
            
            else:
                rospy.loginfo("Target not in sight. Hovering and rotating.")
                hover_error = 8 - position_uav1[2]  # Target height = 8 meters (adjust if needed)
                velocity_msg.velocity.z = self.pid_control_hover.update(hover_error, dt)
                velocity_msg.velocity.x = 0  # Move forward to search for the target
                velocity_msg.velocity.y = 0
                velocity_msg.yaw_rate = 0.6  # Rotate until target comes into FOV
                
                self.vel_pub.publish(velocity_msg)
                self.target_in_sight = False
                self.K_forward = 0
                self.time_since_lock = 0
            
            self.rate.sleep()

if __name__ == "__main__":
    try:
        controller = VisualServoController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutdown requested.")