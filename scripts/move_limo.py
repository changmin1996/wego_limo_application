#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class MoveLimo:
    def __init__(self):
        self.pub  = rospy.Publisher('cmd_vel',Twist, queue_size=10)
        self.rate = rospy.Rate(10)
    
    def move_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.5
        for i in range(1, 20):
            self.pub.publish(cmd)
            self.rate.sleep()

    def move_backward(self):
        cmd = Twist()
        cmd.linear.x = -0.5
        for i in range(1, 20):
            self.pub.publish(cmd)
            self.rate.sleep()

    def turn_left(self):
        cmd = Twist()
        cmd.angular.z = 0.5
        for i in range(1, 20):
            self.pub.publish(cmd)
            self.rate.sleep()
    
    def turn_right(self):
        cmd = Twist()
        cmd.angular.z = -0.5
        for i in range(1, 20):
            self.pub.publish(cmd)
            self.rate.sleep()

def main():
    rospy.init_node('move_limo')
    
    ml=MoveLimo()

    ml.move_forward()
    ml.move_backward()
    ml.turn_left()
    ml.turn_right()

if __name__=='__main__':
    main()

