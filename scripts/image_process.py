#! /usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

class ImageProcess:
    def __init__(self):
        self.sub = rospy.Subscriber('camera/rgb/image_raw/compressed',
                                    CompressedImage,
                                    self.image_callback)
        self.br = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.br.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
        
        (rows, cols, channels) = cv_image.shape
        cv2.circle(cv_image,(cols/2, rows/2), 10, (255,0,0), -1)

        cv2.imshow('image', cv_image)
        cv2.waitKey(3)

def main():
    rospy.init_node('image_process')

    ip = ImageProcess()
    rospy.spin()

if __name__ == '__main__':
    main()
