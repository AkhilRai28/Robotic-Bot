#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import acos, pi, sqrt,degrees

class Obstacle:
    def __init__(self):
        rospy.init_node('obstacle', anonymous=True)

        self.left_distance = None
        self.middle_distance = None
        self.right_distance = None

        rospy.Subscriber('ultrasound', Twist, self.ultrasound_callback)
        
        self.vel_pub = rospy.Publisher('mybot', Twist, queue_size=10)
        
    def ultrasound_callback(self, data):
        self.left_distance = data.linear.x
        self.middle_distance = data.linear.y
        self.right_distance = data.linear.z

    def avoid_obstacle(self):
        twist_cmd = Twist()
        if self.middle_distance>10:
            distance = sqrt((self.right_distance - self.left_distance)**2 + self.middle_distance**2)
            angle1 = acos((self.right_distance - self.left_distance) /distance)
            angle=degrees(angle1)
            twist_cmd.linear.x = (180 - angle) * distance/100*3*0.63*1.35
            twist_cmd.linear.y = angle * distance/100*3*0.63*1.35
            twist_cmd.linear.z=1
            print(angle)
            print(distance)
        if self.middle_distance<10:
            twist_cmd.linear.z=0
            twist_cmd.linear.y=255
            twist_cmd.linear.x=115            
        self.vel_pub.publish(twist_cmd)

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.left_distance is not None and self.middle_distance is not None and self.right_distance is not None:
                self.avoid_obstacle()

            rate.sleep()

if __name__ == '__main__':
    try:
        node = Obstacle()
        node.run()
    except rospy.ROSInterruptException:
        pass