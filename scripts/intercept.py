import rospy
import smach
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from offboard_py.cfg import VelocityConfigConfig
from pid_controller import PIDController
from projector import Projector
from kalman_filter import KalmanFilterPredictor
import numpy as np

class Intercept(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["TARGET_HIT", "TARGET_LOST"])
        
        # Subscribers
        self.local_pos_sub_uav1 = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.pose_cb_uav1)
        self.last_velocity_sub = rospy.Subscriber("/uav0/last_velocity_x", Float32, self.last_velocity_cb)
        
        # New Subscriber for /image_error
        self.image_error_sub = rospy.Subscriber("/image_error/", Point, self.image_error_callback)
        
        # Publishers
        self.vel_pub = rospy.Publisher("/uav0/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.vertical_error_projector_pub = rospy.Publisher("/uav0/vertical_error_projector", Float32, queue_size=10)
        self.error_pub = rospy.Publisher("/uav0/horizontal_error", Float32, queue_size=10)
        self.distance_error_pub = rospy.Publisher("/uav0/distance_error", Float32, queue_size=10)
        self.yaw_rate_pub = rospy.Publisher("/uav0/yaw_rate", Float32, queue_size=10)
        self.delta_forward_pub = rospy.Publisher("/uav0/delta_forward", Float32, queue_size=10)
        self.delta_h_pub = rospy.Publisher("/uav0/delta_h", Float32, queue_size=10)
        
        # init dynamic reconfigurable parameters
        self.init_dynamic_paramaters()
     

        # Variables for state and velocity control
        self.current_pose_uav1 = PoseStamped()
        self.rate = rospy.Rate(50)  # 50 Hz
        self.projector = Projector()

        self.kalman_predictor = KalmanFilterPredictor(dt=1.0/50.0, process_noise_std=10, measurement_noise_std=4)
        self.image_center = (400, 400)
        self.last_time = rospy.Time.now()
        self.target_in_sight = False
        self.time_since_lock = 0
        self.lock_time = 5.0  # Time (in seconds) to reach K_forward = 1
        self.K_forward = 0
        self.last_velocity_x = 0.0
        self.delta_r_forward = 0.0  # Cumulative velocity tracking variable
        self.desired_forward_velocity = 0.0  # New variable for smooth transition

        # Variables to store camera-based errors
        self.horizontal_error_camera = 0.0
        self.vertical_error_camera = 0.0

    def pose_cb_uav1(self, msg):
        self.current_pose_uav1 = msg

    def last_velocity_cb(self, msg):
        self.last_velocity_x = msg.data

    def image_error_callback(self, msg):
        """
        Callback function for /image_error/ topic.
        Stores the horizontal and vertical errors from the camera.
        """
        self.horizontal_error_camera = msg.x  # Horizontal error in pixels
        self.vertical_error_camera = msg.y    # Vertical error in pixels
        rospy.logdebug(f"Received /image_error/: x={self.horizontal_error_camera}, y={self.vertical_error_camera}")

    def init_dynamic_paramaters(self): 
        
        # PID Controllers (re-initializing might not be necessary unless parameters can change dynamically)
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

        
        self.kalman_filter = rospy.get_param("pursuer/vision/kalman_filter", True)
        self.virtual_plane = rospy.get_param("pursuer/vision/virtual_plane", True)
        self.IOU = rospy.get_param("pursuer/vision/IOU", 1.0)
        self.follow_distance = rospy.get_param("pursuer/control/follow_distance", 3.0)
        self.vision = rospy.get_param("pursuer/vision/vision", False)



    def execute(self, userdata):
        # Reset state parameters at the start of execution
        self.time_since_lock = 0
        self.K_forward = 0
        self.delta_r_forward = self.last_velocity_x  # Initialize with last received velocity
        self.desired_forward_velocity = self.last_velocity_x  # Initialize desired velocity

        self.init_dynamic_paramaters()

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
                # Determine which errors to use based on vision mode
                if self.vision:
                    horizontal_error = self.horizontal_error_camera
                    vertical_error = self.vertical_error_camera
                    rospy.logdebug("Using camera-based errors.")
                else:  # 'projected' mode
                    horizontal_error, vertical_error = self.projector.calculate_pixel_error(uv_measured, self.image_center)
                    rospy.logdebug("Using projector-based errors.")
                
                # Apply Kalman Filter if enabled and using projected mode
                if self.kalman_filter and not self.vision:
                    if not self.kalman_predictor.is_initialized:
                        self.kalman_predictor.initialize(np.array(uv_measured))
                    measurement = np.array(uv_measured)
                    self.kalman_predictor.predict()
                    self.kalman_predictor.update(measurement)
                    predicted_uv = self.kalman_predictor.get_predicted_position(steps_ahead=1)
                    horizontal_error, vertical_error = self.projector.calculate_pixel_error(predicted_uv, self.image_center)
                    rospy.logdebug("Applied Kalman Filter to projected errors.")

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
                delta_h = self.pid_control_vertical.update(-(vertical_error) / 400.0, dt)
                delta_r_forward = self.K_forward * self.pid_control_distance.update(distance_measured + 7, dt)

                # Update desired_forward_velocity based on delta_r_forward and last_velocity_x
                if delta_r_forward > self.last_velocity_x:
                    self.desired_forward_velocity = delta_r_forward
                else:
                    self.desired_forward_velocity = self.last_velocity_x

                # Populate the velocity message
                velocity_msg.velocity.x = -self.desired_forward_velocity  # Negative because of coordinate frame
                velocity_msg.velocity.z = delta_h
                velocity_msg.yaw_rate = delta_yaw
                
                # Publish the velocity command
                self.vel_pub.publish(velocity_msg)
                self.target_in_sight = True
                
                # Publish errors based on vision mode
                if self.vision:
                    self.vertical_error_projector_pub.publish(self.vertical_error_camera)
                    self.error_pub.publish(self.horizontal_error_camera)
                else:
                    self.vertical_error_projector_pub.publish(vertical_error)
                    self.error_pub.publish(horizontal_error)
                
                # Publish distance error and other parameters
                self.distance_error_pub.publish(distance_measured - self.follow_distance)
                self.yaw_rate_pub.publish(delta_yaw)
                self.delta_forward_pub.publish(delta_r_forward)
                self.delta_h_pub.publish(delta_h)

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
        intercept = Intercept()
        intercept.execute(None)
    except rospy.ROSInterruptException:
        pass
