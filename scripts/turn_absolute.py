#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math

class TurnAbsolute:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('imu', Imu, self.imu_callback)
        self.yaw = 0
        self.goal = 0

    def imu_callback(self, msg):
        self.set_yaw(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    
    def set_yaw(self, x, y, z, w):
        t3 = 2.0 * (w*z + x*y)
        t4 = 1.0 - 2.0 *(y*y + z*z)
        self.yaw = math.atan2(t3, t4)

    def set_goal(self, angle):
        self.goal = angle
    
    def turn(self):
        gap = self.goal - self.yaw
        gap = self.normalize_angle(gap)
        data =Twist()

        if abs(gap) < 0.05:
            data.angular.z = 0.0
            self.pub.publish(data)
            return

        if gap > 0:
            data.angular.z = 0.3
        else:
            data.angular.z = -0.3
    
        self.pub.publish(data)

    def normalize_angle(self, angle):
        while angle >= math.pi:
            angle -= 2*math.pi
        while angle <= -math.pi:
            angle += 2*math.pi 
        return angle
        
def main():
    rospy.init_node('turn_absolute')

    ta = TurnAbsolute()
    ta.set_goal(2.0)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ta.turn()
        rate.sleep()

if __name__ == '__main__':
    main()
