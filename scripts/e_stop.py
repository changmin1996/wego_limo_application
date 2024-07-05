#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from math import cos, sin

class EStop:
    def __init__(self):
        self.sub = rospy.Subscriber('scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('stop', Bool, queue_size=10)

    def laser_callback(self, msg):
        estop = Bool()
        estop.data = False

        for i, data in enumerate(msg.ranges):
            current_angle = i * msg.angle_increment + msg.angle_min
            cx=  data * cos(current_angle)
            cy = data * sin(current_angle)
            if 0.01 < cx < 0.2 and -0.1 < cy < 0.1:
                estop.data=True
                break

        self.pub.publish(estop)

def main():
    rospy.init_node('e_stop')
    es =EStop()
    rospy.spin()

if __name__ == '__main__':
    main()
