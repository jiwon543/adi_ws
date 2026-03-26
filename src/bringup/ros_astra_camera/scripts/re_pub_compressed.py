#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge


class ImageRepubisher:
    def __init__(self):
        rospy.init_node('image_republisher', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/camera/color/image_raw/compressed',CompressedImage, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image_bgr)
            self.image_pub.publish(compressed_image_msg)
        except Exception as e:
            rospy.logerr(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ImageRepubisher()
        node.run()
    except rospy.ROSInterruptException:
        pass


