#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
z = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def pid(dis, pos):
    kp = 1
    ki = 0
    kd = 0
    if pos==-1:
        x.pop(0)
        x.append(dis)
        p = dis
        i = 0
        for num in x:
            i = i + num
        d = x[9] - x[8]
    elif pos==1:
        z.pop(0)
        z.append(dis)
        p = dis
        i = 0
        for num in z:
            i = i + num
        d = z[9] - z[8]
    speed =  (kp*p) + (ki*i) + (kd*d)
    return (speed)


def modify(data):
    pub = rospy.Publisher('mod_msg', Twist, queue_size=10)
    rev_data = Twist()
    cal_data = Twist()
    send_data = Twist()
    rev_data = data

    # rev_data.linear.x = 65
    # rev_data.linear.y = 65
    # rev_data.linear.z = 5

    cal_data.linear.x = pid(rev_data.linear.x, -1)
    cal_data.linear.z = pid(rev_data.linear.z, 1)

    # if(rev_data.linear.y<20):
    #     send_data.linear.x = cal_data.linear.z
    #     send_data.linear.z = -(cal_data.linear.x)
    # else:
    #     send_data.linear.x = cal_data.linear.z
    #     send_data.linear.z = cal_data.linear.x

    if((rev_data.linear.y<20 and rev_data.linear.y>10)  or (rev_data.linear.z<20 and rev_data.linear.z>10) or (rev_data.linear.x<20 and rev_data.linear.x>10)):
        send_data.linear.x = cal_data.linear.z
        send_data.linear.z = -(cal_data.linear.x)
    elif(rev_data.linear.y<10 or rev_data.linear.z<10 or rev_data.linear.x<10):
        send_data.linear.x = -(cal_data.linear.z)*2
        send_data.linear.z = -(cal_data.linear.x)*2
    else:
        send_data.linear.x = cal_data.linear.z
        send_data.linear.z = cal_data.linear.x

    pub.publish(send_data)

def algorithm_three():
    
    rospy.init_node('ultra_input', anonymous=True)
    rospy.Subscriber('ultra_msg', Twist, modify)   
    rospy.spin()

if __name__ == '__main__':
    algorithm_three()