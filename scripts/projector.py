import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from tf.transformations import euler_from_quaternion


class Projector:
    def __init__(self):
        self.fx = 800
        self.fy = 800
        self.cx = 400
        self.cy = 400
        self.K = np.array([[self.fx, 0, self.cx],
                           [0, self.fy, self.cy],
                           [0, 0, 1]])
        
        self.drone_width = 0.5  # meters
        self.drone_height = 0.15  # meters
        self.pixel_range = 800  # Total range from -400 to 400


        # Current pose of UAV1 and UAV2
        self.current_pose_uav1 = PoseStamped()
        self.current_pose_uav2 = PoseStamped()

        # Subscribers
        self.local_pos_sub_uav1 = rospy.Subscriber("/uav0/mavros/local_position/pose", PoseStamped, self.pose_cb_uav1)
        self.local_pos_sub_uav2 = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, self.pose_cb_uav2)

    def pose_cb_uav1(self, msg):
        self.current_pose_uav1 = msg
    
    def pose_cb_uav2(self, msg):
        self.current_pose_uav2 = msg

    def project_point(self,iou, vp, add_noise=True):
    

        cam_coords = np.array([
                self.current_pose_uav1.pose.position.x,
                self.current_pose_uav1.pose.position.y,
                self.current_pose_uav1.pose.position.z
            ])

        orientation_q = self.current_pose_uav1.pose.orientation
        
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        angles = (yaw, pitch, roll)

        P = np.array([
                self.current_pose_uav2.pose.position.x,
                self.current_pose_uav2.pose.position.y,
                self.current_pose_uav2.pose.position.z
            ])

        P_translated = P - cam_coords
        distance_xy = np.sqrt(P_translated[0]**2 + P_translated[1]**2)

        yaw, pitch, roll = angles

        if vp:
            # rospy.loginfo(f"vp enabled")
            roll = 0
            pitch = 0

        # Rotation matrices
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        R_y = np.array([[np.cos(yaw), 0, np.sin(yaw)],
                        [0, 1, 0],
                        [-np.sin(yaw), 0, np.cos(yaw)]])
        R_z = np.array([[np.cos(pitch), -np.sin(pitch), 0],
                        [np.sin(pitch), np.cos(pitch), 0],
                        [0, 0, 1]])

        R_mat = R_y @ R_z @ R_x  # Combined rotation matrix in 'yxz' order

        # Rotate around x-axis by 90 degrees
        angle = np.pi / 2
        R_x_90 = np.array([[1, 0, 0],
                           [0, np.cos(angle), -np.sin(angle)],
                           [0, np.sin(angle), np.cos(angle)]])
        R_y_90 = np.array([[np.cos(angle), 0, np.sin(angle)],
                           [0, 1, 0],
                           [-np.sin(angle), 0, np.cos(angle)]])

        P_rotated_x = R_x_90 @ P_translated
        P_rotated_y = R_y_90 @ P_rotated_x

        P_camera = R_mat @ P_rotated_y

        # Prevent negative Z (behind the camera)
        if P_camera[2] <= 0:
            return None, None

        # Project the point to image coordinates
        P_image = self.K @ P_camera
        P_image /= P_image[2]  # Normalize by Z coordinate

        # Calculate the area of the bounding box based on the drone's distance
        area_box = self.drone_width * self.drone_height / distance_xy  # Proportional to 1 / distance

        # Scale the noise by pixel range (based on [-400, 400])
        box_size_in_pixels = (self.drone_width * self.pixel_range) / distance_xy  # Assume width scales with pixel range

        # Adjust noise to reflect IoU directly in pixel terms
        sigma_position = np.sqrt((1 - iou) * box_size_in_pixels / 2)  # For horizontal and vertical errors
        sigma_distance = np.sqrt((1 - iou) * area_box)  # For distance errors

        if add_noise:
            # Add Gaussian noise to the projected image coordinates
            noise_position = np.random.normal(0, sigma_position, 2)
            P_image[:2] += noise_position  # Horizontal and vertical noise

            # Add Gaussian noise to the distance measurement
            distance_noise = np.random.normal(0, sigma_distance)
            distance_xy += distance_noise

        if 0 <= P_image[0] <= 800 and 0 <= P_image[1] <= 800:
            return P_image[:2], distance_xy
        else:
            return None, None

    def calculate_pixel_error(self, uv, image_center):
        if uv is not None:
            horizontal_error = uv[0] - image_center[0]
            vertical_error = uv[1] - image_center[1]
            return horizontal_error, vertical_error
        else:
            return 0, 0
