#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32

class ControlLimo:
    def __init__(self):
        self.stop_sub = rospy.Subscriber('stop', Bool, self.stop_callback)
        self.error_sub = rospy.Subscriber('distance_y', Int32, self.dis_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.floor_sub = rospy.Subscriber('floor_color', Int32, self.floor_callback)

        self.stop_flag = False
        self.lateral_gain = 0.008
        self.floor_color = 0

    def floor_callback(self, msg):
        self.floor_color = msg.data


    def stop_callback(self, msg):
        self.stop_flag = msg.data

    def dis_callback(self, msg):
        cmd = Twist()
        control = self.lateral_gain * msg.data

        cmd.angular.z = min(max(-1.5, control), 1,5)        

        if self.floor_color == 1:
            cmd.linear.x = 0.7
        elif self.floor_color == 2:
            cmd.linear.x = 0.2
        else:
            cmd.linear.x = 0.3

        if self.stop_flag:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.pub.publish(cmd)       

def main():
    rospy.init_node('limo_control')
    cl = ControlLimo()
    rospy.spin()

if __name__ == '__main__':
    main()
