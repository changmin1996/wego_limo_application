#! /usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import numpy as np

from std_msgs.msg import Int32

class DetectFloorColor:
    def __init__(self):
        self.sub = rospy.Subscriber('camera/rgb/image_raw/compressed', CompressedImage, self.image_callback)
        self.br = CvBridge()
        
        self.pub = rospy.Publisher('floor_color', Int32, queue_size = 10)

        self.RED_FLOOR_LOW_TH = np.array([150, 90, 100])
        self.RED_FLOOR_HIGH_TH = np.array([255, 255, 255])

        self.BLUE_FLOOR_LOW_TH = np.array([0, 90, 100])
        self.BLUE_FLOOR_HIGH_TH = np.array([155, 155, 255])

    def image_callback(self, msg):
        try:
            self.cv_image = self.br.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
        
        #cv2.imshow('original_image', self.cv_image)
        self.cropped_image = self.imageCrop(self.cv_image)
        
        self.blue_thresholded_image = self.blueColorDetect(self.cropped_image)
        self.red_thresholded_image = self.redColorDetect(self.cropped_image)
        
        cv2.imshow('blue_thresholded_image', self.blue_thresholded_image)
        cv2.imshow('red_thresholded_image', self.red_thresholded_image)
        
        M_blue = cv2.moments(self.blue_thresholded_image)
        M_red = cv2.moments(self.red_thresholded_image)

        floor_color = Int32()
        floor_color.data = 0 # 0: default 1: blue, 2: red

        if M_red['m00'] > 1800000:
            #print(M_red['m00'])
            floor_color.data = 2
        
        if M_blue['m00'] > 1800000:
            #print(M_blue['m00'])
            floor_color.data = 1

        self.pub.publish(floor_color)
        
        cv2.waitKey(3)

    def imageCrop(self, _img):
        return _img[400:480, 270:370]
    
    def blueColorDetect(self, _img):
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_blue = cv2.inRange(hls, self.BLUE_FLOOR_LOW_TH, self.BLUE_FLOOR_HIGH_TH)
        return mask_blue
    
    def redColorDetect(self, _img):
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_red = cv2.inRange(hls, self.RED_FLOOR_LOW_TH, self.RED_FLOOR_HIGH_TH)
        return mask_red

def main():
    rospy.init_node('detect_floor')
    df = DetectFloorColor()
    rospy.spin()

if __name__ == '__main__':
    main()




