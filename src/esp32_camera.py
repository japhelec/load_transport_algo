#!/usr/bin/env python3

import cv2
import numpy as np
import requests
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import threading


class ESP32_CAM:
    def __init__(self):
        self.url = "http://192.168.4.1"
        self.cap = cv2.VideoCapture(self.url + ":81/stream")
        
        self.set_resolution(self.url, index=8)
        self.set_awb(self.url, True)

        self.pub_image_raw = rospy.Publisher(
                'esp32/image_raw', Image, queue_size=10)
        self.bridge = CvBridge()
        
        rospy.on_shutdown(self.shutdown_hook)
        self.streaming()

    def shutdown_hook(self):
        cv2.destroyAllWindows()
        self.cap.release()

    def streaming(self):
        while not rospy.is_shutdown():
            if self.cap.isOpened():
                ret, frame = self.cap.read()

                if ret:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    self.pub_image_raw.publish(self.bridge.cv2_to_imgmsg(gray))
                    
                    # gray = cv2.equalizeHist(gray)
                    # cv2.imshow("frame", gray)
                    # key = cv2.waitKey(1)

    def set_resolution(self, url: str, index: int=1, verbose: bool=False):
        try:
            if verbose:
                resolutions = "10: UXGA(1600x1200)\n9: SXGA(1280x1024)\n8: XGA(1024x768)\n7: SVGA(800x600)\n6: VGA(640x480)\n5: CIF(400x296)\n4: QVGA(320x240)\n3: HQVGA(240x176)\n0: QQVGA(160x120)"
                print("available resolutions\n{}".format(resolutions))

            if index in [10, 9, 8, 7, 6, 5, 4, 3, 0]:
                requests.get(url + "/control?var=framesize&val={}".format(index))
            else:
                print("Wrong index")
        except:
            print("SET_RESOLUTION: something went wrong")

    def set_quality(self, url: str, value: int=1, verbose: bool=False):
        try:
            if value >= 10 and value <=63:
                requests.get(url + "/control?var=quality&val={}".format(value))
        except:
            print("SET_QUALITY: something went wrong")

    def set_awb(self, url: str, awb: int=1):
        try:
            awb = not awb
            requests.get(url + "/control?var=awb&val={}".format(1 if awb else 0))
        except:
            print("SET_QUALITY: something went wrong")
        return awb


if __name__ == '__main__':
    rospy.init_node('esp32_camera', anonymous=True)
    ESP32_CAM()