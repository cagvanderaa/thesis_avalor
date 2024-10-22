#!/usr/bin/env python3

import rospy
import smach
import math  # Ensure math is imported
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from offboard_py.cfg import VelocityConfigConfig
from pid_controller import PIDController
from projector import Projector
from kalman_filter import KalmanFilterPredictor
import numpy as np

class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["READY_TO_INTERCEPT", "TARGET_LOST"])
        
        # Subscribers
        self.local_pos_sub_uav1 = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.pose_cb_uav1)
        self.image_error_sub = rospy.Subscriber("/image_error/", Point, self.image_error_callback)
        
        # Publishers
        self.vel_pub = rospy.Publisher("/uav0/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.vertical_error_projector_pub = rospy.Publisher("/uav0/vertical_error_projector", Float32, queue_size=10)
        self.error_pub = rospy.Publisher("/uav0/horizontal_error", Float32, queue_size=10)
        self.distance_error_pub = rospy.Publisher("/uav0/distance_error", Float32, queue_size=10)
        self.yaw_rate_pub = rospy.Publisher("/uav0/yaw_rate", Float32, queue_size=10)
        self.delta_forward_pub = rospy.Publisher("/uav0/delta_forward", Float32, queue_size=10)
        self.delta_h_pub = rospy.Publisher("/uav0/delta_h", Float32, queue_size=10)
        self.last_velocity_pub = rospy.Publisher("/uav0/last_velocity_x", Float32, queue_size=10)

        # PID Controllers Initialization
        self.initialize_pid_controllers()
        
        # Dynamic Reconfigure Server
        self.srv = Server(VelocityConfigConfig, self.dynamic_reconfigure_callback)
        rospy.loginfo("Dynamic Reconfigure Server for Follow Initialized.")
        self.set_initial_reconfigure_values()
        
        # State Variables
        self.current_pose_uav1 = PoseStamped()
        self.rate = rospy.Rate(30)  # 30 Hz
        self.projector = Projector()
        self.kalman_filter = rospy.get_param("pursuer/vision/kalman_filter", True)
        self.virtual_plane = rospy.get_param("pursuer/vision/virtual_plane", True)
        self.IOU = rospy.get_param("pursuer/vision/IOU", 1.0)
        self.follow_distance = rospy.get_param("pursuer/control/follow_distance", 3.0)
        self.vertical_offset = rospy.get_param("pursuer/vision/vertical_offset", 0)
        self.kalman_predictor = KalmanFilterPredictor(dt=1.0/30.0, process_noise_std=10, measurement_noise_std=4)
        self.image_center = (400, 400)
        self.last_time = rospy.Time.now()
        self.target_in_sight = False
        self.time_since_lock = 0
        self.lock_criteria_met_time = rospy.Time.now()
        self.lock_duration_threshold = rospy.Duration(3.0)
        self.lock_criteria_met = False
        self.lock_time = 8.0  # Time (in seconds) to reach K_forward = 1
        self.K_forward = 0

        # Vision Mode Flags        
        self.vision = rospy.get_param("pursuer/vision/vision", False)        # Load 'vision' from parameters
        self.intercept = rospy.get_param("pursuer/control/intercept", False) # Load 'intercept' from parameters


        # Camera-Based Errors
        self.horizontal_error_camera = float('nan')  # Initialize as NaN
        self.vertical_error_camera = float('nan')    # Initialize as NaN

    def initialize_pid_controllers(self):
        """Initialize PID controllers with parameters from the parameter server."""

        # PID Controllers
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

    def set_initial_reconfigure_values(self):
        """Set initial values for dynamic reconfigure based on ROS parameters."""
        initial_config = {
            'virtual_plane': rospy.get_param("pursuer/vision/virtual_plane", True),
            'kalman_filter': rospy.get_param("pursuer/vision/kalman_filter", True),
            'IOU': rospy.get_param("pursuer/vision/IOU", 1.0),
            'pid_horizontal_Kp': rospy.get_param("pursuer/control/pid_control_horizontal/Kp", 1.0),
            'pid_horizontal_Ki': rospy.get_param("pursuer/control/pid_control_horizontal/Ki", 0.0),
            'pid_horizontal_Kd': rospy.get_param("pursuer/control/pid_control_horizontal/Kd", 0.0),
            'pid_vertical_Kp': rospy.get_param("pursuer/control/pid_control_vertical/Kp", 1.0),
            'pid_vertical_Ki': rospy.get_param("pursuer/control/pid_control_vertical/Ki", 0.0),
            'pid_vertical_Kd': rospy.get_param("pursuer/control/pid_control_vertical/Kd", 0.0),
            'pid_distance_Kp': rospy.get_param("pursuer/control/pid_control_distance/Kp", 1.0),
            'pid_distance_Ki': rospy.get_param("pursuer/control/pid_control_distance/Ki", 0.0),
            'pid_distance_Kd': rospy.get_param("pursuer/control/pid_control_distance/Kd", 0.0),
            'follow_distance': rospy.get_param("pursuer/control/follow_distance", 3.0),
            'vertical_offset': rospy.get_param("pursuer/vision/vertical_offset", 0),
            'vision': rospy.get_param("pursuer/vision/vision", False),
            'intercept': rospy.get_param("pursuer/control/intercept", False)
        }
        
        self.srv.update_configuration(initial_config)


    def dynamic_reconfigure_callback(self, config, level):
        rospy.loginfo(f"Reconfigure Request: {config}")

        # Update PID parameters for Horizontal Control, Vertical Control, and Distance Control
        self.pid_control_horizontal.update_gains(config.pid_horizontal_Kp, config.pid_horizontal_Ki, config.pid_horizontal_Kd)
        self.pid_control_vertical.update_gains(config.pid_vertical_Kp, config.pid_vertical_Ki, config.pid_vertical_Kd)
        self.pid_control_distance.update_gains(config.pid_distance_Kp, config.pid_distance_Ki, config.pid_distance_Kd)
        
        # Update Follow Distance and Vision Parameters
        self.follow_distance = config.follow_distance
        rospy.loginfo(f"Updated Follow Distance: {self.follow_distance} m")
        
        self.virtual_plane = config.virtual_plane
        self.vertical_offset = config.vertical_offset
        self.kalman_filter = config.kalman_filter
        self.IOU = config.IOU
        self.vision = config.vision  # Update vision mode
        self.intercept = config.intercept
        return config

    def pose_cb_uav1(self, msg):
        self.current_pose_uav1 = msg

    def image_error_callback(self, msg):
        """
        Callback function for /image_error/ topic.
        Stores the horizontal and vertical errors from the camera.
        """
        self.horizontal_error_camera = msg.x  # Horizontal error in pixels
        self.vertical_error_camera = msg.y    # Vertical error in pixels
        rospy.logdebug(f"Received /image_error/: x={self.horizontal_error_camera}, y={self.vertical_error_camera}")

    def get_current_errors(self, uv_measured):
        """
        Determines the current horizontal and vertical errors based on vision mode and data validity.
        Returns:
            tuple: (horizontal_error, vertical_error)
        """
        if self.vision:
            horizontal_error = self.horizontal_error_camera
            vertical_error = self.vertical_error_camera

            # Use camera-based errors if they are valid
            if not math.isnan(horizontal_error) and not math.isnan(vertical_error):
                rospy.logdebug("Using camera-based errors.")
                return horizontal_error, vertical_error
            else:
                rospy.logwarn("Camera errors are NaN. Falling back to projection-based errors.")
        
        # Fallback to projection-based errors
        horizontal_error, vertical_error = self.projector.calculate_pixel_error(uv_measured, self.image_center)
        rospy.logdebug("Using projection-based errors.")
        return horizontal_error, vertical_error

    def apply_kalman_filter(self, uv_measured):
        """
        Applies Kalman Filter to the measured UV coordinates if enabled and not in vision mode.
        Returns:
            tuple: (predicted_horizontal_error, predicted_vertical_error)
        """
        if self.kalman_filter and not self.vision:
            if not self.kalman_predictor.is_initialized:
                self.kalman_predictor.initialize(np.array(uv_measured))
                rospy.logdebug("Kalman Filter initialized with initial measurement.")
            
            measurement = np.array(uv_measured)
            self.kalman_predictor.predict()
            self.kalman_predictor.update(measurement)
            predicted_uv = self.kalman_predictor.get_predicted_position(steps_ahead=1)
            horizontal_error, vertical_error = self.projector.calculate_pixel_error(predicted_uv, self.image_center)
            rospy.logdebug("Applied Kalman Filter to projected errors.")
            return horizontal_error, vertical_error
        return None, None

    def execute(self, userdata):
        self.time_since_lock = 0
        self.K_forward = 0

        while not rospy.is_shutdown():
            # Compute Measured Projection (with noise)
            uv_measured, distance_measured = self.projector.project_point(
                self.IOU, self.virtual_plane, add_noise=True
            )

            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time

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

            if uv_measured is not None and distance_measured is not None:
                # Determine current errors
                horizontal_error, vertical_error = self.get_current_errors(uv_measured)

                # Apply Kalman Filter if applicable
                kalman_horizontal_error, kalman_vertical_error = self.apply_kalman_filter(uv_measured)
                if kalman_horizontal_error is not None and kalman_vertical_error is not None:
                    horizontal_error, vertical_error = kalman_horizontal_error, kalman_vertical_error

                # Locking the target based on lock time
                if self.target_in_sight:
                    self.time_since_lock += dt
                    self.K_forward = min(1, (self.time_since_lock / self.lock_time))
                    rospy.loginfo(f'K_forward = {self.K_forward}')
                else:
                    self.time_since_lock = 0
                    self.K_forward = 0

                # Compute PID controller outputs
                delta_yaw = -self.pid_control_horizontal.update(horizontal_error / 400.0, dt)
                delta_h = self.pid_control_vertical.update(-(vertical_error - self.vertical_offset) / 400.0, dt)
                delta_r_forward = self.K_forward * self.pid_control_distance.update(distance_measured - self.follow_distance, dt)

                # Populate the velocity message
                velocity_msg.velocity.x = -delta_r_forward
                velocity_msg.velocity.z = delta_h
                velocity_msg.yaw_rate = delta_yaw
                
                # Publish the velocity command
                self.vel_pub.publish(velocity_msg)
                self.target_in_sight = True
                
                # Publish errors based on vision mode
                if self.vision and not (math.isnan(self.horizontal_error_camera) or math.isnan(self.vertical_error_camera)):
                    self.vertical_error_projector_pub.publish(Float32(data=self.vertical_error_camera))
                    self.error_pub.publish(Float32(data=self.horizontal_error_camera))
                else:
                    self.vertical_error_projector_pub.publish(Float32(data=vertical_error))
                    self.error_pub.publish(Float32(data=horizontal_error))
                
                # Publish distance error and other parameters
                self.distance_error_pub.publish(Float32(data=distance_measured - self.follow_distance))
                self.yaw_rate_pub.publish(Float32(data=delta_yaw))
                self.delta_forward_pub.publish(Float32(data=delta_r_forward))
                self.delta_h_pub.publish(Float32(data=delta_h))

                # Handle intercept state
                if self.intercept: 
                    rospy.loginfo("Proceeding to INTERCEPT state.")
                    self.last_velocity_pub.publish(Float32(data=delta_r_forward))
                    return "READY_TO_INTERCEPT"

                # Check if criteria for interception are met
                if (abs(horizontal_error) <= 1 and 
                    abs(vertical_error) <= 1 and 
                    abs(distance_measured - self.follow_distance) <= 3.0):
                    
                    if not self.lock_criteria_met:
                        self.lock_criteria_met_time = rospy.Time.now()
                        self.lock_criteria_met = True
                    elif current_time - self.lock_criteria_met_time >= self.lock_duration_threshold:
                        rospy.loginfo("Stable following achieved. Proceeding to INTERCEPT state.")
                        # Publish last velocity for the interception algorithm
                        self.last_velocity_pub.publish(Float32(data=delta_r_forward))
                        return "READY_TO_INTERCEPT"
                else:
                    self.lock_criteria_met = False
                    
            else:
                rospy.loginfo("Target lost. Switching to TARGET_LOST state.")
                self.target_in_sight = False
                self.K_forward = 0
                self.time_since_lock = 0
                return "TARGET_LOST"

            self.rate.sleep()

        return "TARGET_LOST"

if __name__ == '__main__':
    try:
        rospy.init_node('follow_state_node')  # Initialize ROS node
        follow_state = Follow()
        outcome = follow_state.execute(None)
        rospy.loginfo(f"State Outcome: {outcome}")
    except rospy.ROSInterruptException:
        pass
