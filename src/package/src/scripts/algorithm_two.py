#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def pid(dis):
    kp = 1
    ki = 0
    kd = 0
    x.pop(0)
    x.append(dis)
    p = dis
    i = 0
    for num in x:
        i = i + num
    d = x[9] - x[8]
    
    speed =  (kp*p) + (ki*i) + (kd*d)
    print(speed)
    return (speed)


def modify(data):
    pub = rospy.Publisher('mod_msg', Twist, queue_size=10)
    rev_data = Twist()
    cal_data = Twist()
    send_data = Twist()
    rev_data = data

    if(rev_data.linear.x==0):
        rev_data.linear.x = 65
    elif(rev_data.linear.y==0):
        rev_data.linear.y = 65
    elif(rev_data.linear.z==0):
        rev_data.linear.z = 65

    # rev_data.linear.x = 65
    # rev_data.linear.y = 65
    # rev_data.linear.z = 5

    error = (-1*rev_data.linear.x)+(1*rev_data.linear.z)
    error_pid = pid(error)

    cal_data.linear.x = error_pid * rev_data.linear.x
    cal_data.linear.z = error_pid * rev_data.linear.z
      
    if((rev_data.linear.y<20 and rev_data.linear.y>10)  or (rev_data.linear.z<20 and rev_data.linear.z>10) or (rev_data.linear.x<20 and rev_data.linear.x>10)):
        send_data.linear.x = cal_data.linear.z
        send_data.linear.z = -(cal_data.linear.x)
    elif(rev_data.linear.y<10 or rev_data.linear.z<10 or rev_data.linear.x<10):
        send_data.linear.x = -(cal_data.linear.z)*2
        send_data.linear.z = -(cal_data.linear.x)*2
    else:
        send_data.linear.x = cal_data.linear.z
        send_data.linear.z = cal_data.linear.x
    print(send_data.linear.x, send_data.linear.z)
    # if(condition>45 and condition<55):
    #     send_data.linear.x = cal_data.linear.z
    #     send_data.linear.z = -(cal_data.linear.x)
    # elif(condition>55):
    #     send_data.linear.x = -(cal_data.linear.z)*2
    #     send_data.linear.z = -(cal_data.linear.x)*2
    # elif(condition<45):
    #     send_data.linear.x = cal_data.linear.z
    #     send_data.linear.z = cal_data.linear.x

    pub.publish(send_data)

def algorithm_two():
    
    rospy.init_node('ultra_input', anonymous=True)
    rospy.Subscriber('ultra_msg', Twist, modify)   
    rospy.spin()

if __name__ == '__main__':
    algorithm_two()