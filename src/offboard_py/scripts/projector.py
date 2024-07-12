# projector.py
import numpy as np

class Projector:
    def __init__(self):
        self.fx = 800
        self.fy = 800
        self.cx = 400
        self.cy = 400
        self.K = np.array([[self.fx, 0, self.cx],
                           [0, self.fy, self.cy],
                           [0, 0, 1]])

    def project_point(self, cam_coords, angles, P):
        P_translated = P - cam_coords

        yaw, pitch, roll = angles

        # Rotation matrix around the x-axis (roll)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        # Rotation matrix around the y-axis (pitch)
        R_y = np.array([
            [np.cos(yaw), 0, np.sin(yaw)],
            [0, 1, 0],
            [-np.sin(yaw), 0, np.cos(yaw)]
        ])

        # Rotation matrix around the z-axis (yaw)
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
            
        P_image = self.K @ P_camera
        P_image /= P_image[2]
        
        if 0 <= P_image[0] <= 800 and 0 <= P_image[1] <= 800:
            return P_image[:2], np.linalg.norm(P_translated)
        else:
            return None, None

