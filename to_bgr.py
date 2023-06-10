#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

"""
Node to transform an input Image topic in RGB into a BRG image topic

With help from:
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class BRSwitch(object):
    def __init__(self, name_topic_in, name_topic_out):
        self.cv_bridge = CvBridge()
        rospy.loginfo("Converting Images from topic " + name_topic_in +
                      " to bgr, output topic: " + name_topic_out)
        self.pub = rospy.Publisher(name_topic_out, Image, queue_size=5)
        self.sub = rospy.Subscriber(
            name_topic_in, Image, self.image_cb, queue_size=5)

    def image_cb(self, img_msg):
        # Transform to cv2/numpy image
        img_in_cv2 = self.cv_bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='passthrough')
        # Transform to bgr,
        # available encodings: http://docs.ros.org/jade/api/sensor_msgs/html/image__encodings_8h_source.html
        if "rgb" in img_msg.encoding:
            bgr_img = cv2.cvtColor(img_in_cv2, cv2.COLOR_RGB2BGR)
        elif "bgr" in img_msg.encoding:
            bgr_img = cv2.cvtColor(img_in_cv2, cv2.COLOR_BGR2RGB)
        # Transform back to Image message
        bgr_img_msg = self.cv_bridge.cv2_to_imgmsg(
            bgr_img, encoding="bgr8")
        self.pub.publish(bgr_img_msg)


if __name__ == '__main__':
    argv = rospy.myargv(sys.argv)
    if len(argv) == 1:
        argv.append("/camera/rgb/image_rect_color")
        argv.append("/camera/bgr/image_rect_color")
    if len(argv) != 3:
        print("Usage:")
        print(argv[0] + " name_topic_in name_topic_out")
        print("Converts a RGB name_topic_in Image topic into a BGR name_topic_out Image topic")
        exit(0)
    rospy.init_node("image_to_bgr", anonymous=True)

    gs = BRSwitch(argv[1], argv[2])
    rospy.spin()

