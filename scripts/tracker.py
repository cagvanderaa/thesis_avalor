#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import numpy as np

class DroneTracker:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('drone_tracker_node', anonymous=True)

        # Create a CvBridge object for converting ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the input image topic
        #self.image_sub = rospy.Subscriber('/uav0/camera_ir/camera/color/image_raw', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/camera/image_resized', Image, self.image_callback)

        # Publisher for the tracked image
        self.image_pub = rospy.Publisher('/uav0/camera/image_raw/tracked', Image, queue_size=10)

        # Publisher for the centroid errors
        self.error_pub = rospy.Publisher('/image_error/', Point, queue_size=10)

        # Tracker selection variable: only 'CSRT' is used now
        self.tracker_type = 'CSRT'

        # Initialize the CSRT tracker
        self.tracker = self.create_tracker(self.tracker_type)

        # Bounding box coordinates (x, y, w, h)
        self.bbox = None

        # Previous bounding box for smoothing
        self.prev_bbox = None
        self.alpha = 0.5  # Smoothing factor

        # Tracking state
        self.initialized = False  # Ensure this attribute is initialized

        # OpenCV window for displaying the tracking
        cv2.namedWindow("Drone Tracker", cv2.WINDOW_NORMAL)

        # Image dimensions (as per camera characteristics)
        self.image_width = 640
        self.image_height = 480
        self.image_center_x = self.image_width / 2
        self.image_center_y = self.image_height / 2

    def create_tracker(self, tracker_type):
        """
        Create and return a CSRT tracker.
        """
        if tracker_type == 'CSRT':
            tracker = cv2.TrackerCSRT_create()
            rospy.loginfo("CSRT tracker created.")
        else:
            rospy.logerr(f"Unsupported tracker type: {tracker_type}. Falling back to CSRT.")
            tracker = cv2.TrackerCSRT_create()
        
        return tracker

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to a BGR8 OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        if not self.initialized:
            # Allow the user to select the ROI on the first frame
            rospy.loginfo("Select the target drone bounding box and press ENTER or SPACE to confirm.")
            self.bbox = cv2.selectROI("Drone Tracker", cv_image, fromCenter=False, showCrosshair=True)
            if self.bbox != (0, 0, 0, 0):
                # Initialize the tracker with the selected ROI
                success = self.tracker.init(cv_image, self.bbox)
                if success:
                    self.initialized = True
                    rospy.loginfo(f"Tracker '{self.tracker_type}' initialized with bounding box: {self.bbox}")
                    cv2.destroyWindow("Drone Tracker")
                else:
                    rospy.logerr("Failed to initialize tracker with the selected bounding box.")
        else:
            # Update the tracker for subsequent frames
            success, bbox = self.tracker.update(cv_image)
            error_msg = Point()  # Initialize error message

            if success:
                if self.prev_bbox is not None:
                    # Apply exponential smoothing
                    smoothed_bbox = (
                        self.alpha * np.array(bbox) + (1 - self.alpha) * np.array(self.prev_bbox)
                    )
                    self.bbox = tuple(smoothed_bbox.tolist())
                else:
                    self.bbox = bbox

                self.prev_bbox = self.bbox

                # Tracking success: draw the bounding box
                p1 = (int(self.bbox[0]), int(self.bbox[1]))
                p2 = (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3]))
                cv2.rectangle(cv_image, p1, p2, (255, 0, 0), 2, 1)

                # Calculate and draw the centroid
                centroid_x = int(self.bbox[0] + self.bbox[2] / 2)
                centroid_y = int(self.bbox[1] + self.bbox[3] / 2)
                cv2.circle(cv_image, (centroid_x, centroid_y), 5, (0, 255, 0), -1)

                # Calculate error relative to image center
                error_x = centroid_x - self.image_center_x
                error_y = centroid_y - self.image_center_y

                # Assign error values to the Point message
                error_msg.x = error_x  # Horizontal error
                error_msg.y = error_y  # Vertical error
                error_msg.z = 0  # Unused

                rospy.logdebug(f"Published error - X: {error_x}, Y: {error_y}")
            else:
                # Tracking failure: set errors to None
                rospy.logwarn("Tracking failure detected.")
                error_msg.x = float('nan')
                error_msg.y = float('nan')
                error_msg.z = 0  # Unused

            # Publish the error message
            self.error_pub.publish(error_msg)

            if success:
                # Publish the annotated image
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
                except CvBridgeError as e:
                    rospy.logerr(f"CvBridge Error: {e}")

            # Display the image with tracking
            if self.initialized:
                cv2.imshow("Drone Tracker", cv_image)
                cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tracker = DroneTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass
