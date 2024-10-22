#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class ImageResizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/camera/image_resized", Image, queue_size=10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Resize the image to 320x240
            resized_image = cv2.resize(cv_image, (320, 240))

            # Convert back to ROS Image message and publish
            resized_msg = self.bridge.cv2_to_imgmsg(resized_image, "bgr8")
            self.image_pub.publish(resized_msg)

        except Exception as e:
            rospy.logerr("Failed to resize image: %s" % e)

if __name__ == '__main__':
    rospy.init_node('image_resizer', anonymous=True)
    ImageResizer()
    rospy.spin()
