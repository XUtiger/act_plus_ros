#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    try:
        # 将 ROS 图像消息转换为 OpenCV 图像
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 在 OpenCV 中处理图像
        cv2.imshow("USB Camera", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        print(e)

if __name__ == "__main__":
    rospy.init_node("image_processor")
    rospy.Subscriber("/usb_cam_right_wrist/image_raw", Image, image_callback)
    rospy.spin()