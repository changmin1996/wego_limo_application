#! /usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
import numpy as np

class DetectLane:
    def __init__(self):
        self.sub = rospy.Subscriber('camera/rgb/image_raw/compressed',
                                    CompressedImage,
                                    self.image_callback)
        self.br = CvBridge()

        self.YELLO_LANE_LOW_TH = np.array([0,90,100])
        self.YELLO_LANE_HIGH_TH = np.array([60, 220, 255])

        self.pub = rospy.Publisher('distance_y', Int32, queue_size=10)
    
    def image_callback(self, msg):
        try:
            self.cv_image = self.br.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
        
        #cv2.imshow('original_image', self.cv_image)

        self.cropped_image = self.imageCrop(self.cv_image)
        #cv2.imshow('cropped_image', self.cropped_image)

        self.thresholded_image = self.colorDetect(self.cropped_image)
        cv2.imshow('thresholded_image', self.thresholded_image)

        (x, y) = self.calcMoment(self.thresholded_image)
        ref = 100

        # calculate error input
        error = Int32()
        if y == -1:
            error.data = 0
        else:
            error.data = ref - x
        self.pub.publish(error)

        self.debug_image = self.makeDebugImage(x, y, ref, self.cv_image)
        cv2.imshow('debug_image', self.debug_image)

        cv2.waitKey(3)
    
    def imageCrop(self, _img):
        return _img[350:480, 0:320]
    
    def colorDetect(self, _img):
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_yellow = cv2.inRange(hls, self.YELLO_LANE_LOW_TH, self.YELLO_LANE_HIGH_TH)
        return mask_yellow
    
    def calcMoment(self, _img):
        try:
            M = cv2.moments(_img)
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00']) 
        except:
            x = -1
            y = -1
        return x, y

    def makeDebugImage(self, x, y, y_ref, img):
        cv2.line(img, (y_ref, 0), (y_ref, 480), (0, 255, 0), 5)
        if x == -1:
            return img        
        cv2.circle(img, (x, y+350), 10, (255, 0, 0), -1)        
        return img
    
def main():
    rospy.init_node('detect_lane')
    dl =DetectLane()
    rospy.spin()

if __name__ == '__main__':
    main()
