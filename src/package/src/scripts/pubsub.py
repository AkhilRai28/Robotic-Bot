#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def modify(data):
    rospy.loginfo('I heard %f', data.data)
    pub = rospy.Publisher('mod_msg', Float32, queue_size=10)
    cal_data = float(data.data)
    cal_data = cal_data * 10**3
    rospy.loginfo('I send %f', cal_data)
    pub.publish(float(cal_data)) 

def sub_pub():
    
    rospy.init_node('ultra_input', anonymous=True)
    rospy.Subscriber('ultra_msg', Float32, modify)   
    rospy.spin()

if __name__ == '__main__':
    sub_pub()
