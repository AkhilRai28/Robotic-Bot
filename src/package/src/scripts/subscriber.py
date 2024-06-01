#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
x1, x2, x3 = 0,0,0
pub = rospy.Publisher('mod_msg', Twist, queue_size=10)


def callback(data):
    rospy.loginfo("%f %f %f %f %f", data.linear.x, data.linear.y,
                  data.linear.z, data.angular.x, data.angular.y)

    pub = rospy.Publisher('mod_msg', Twist, queue_size=10)
    x1 = data.linear.x #front
    x2 = data.linear.y #left
    x3 = data.linear.z #right
    try:
        x5 = (x2-30)/abs(x2-30)
        x6 = (x3-30)/abs(x3-30)
    except:
        x5=0
        x6=0
    #min_dist=10cm
    s=104
    xa=s
    xb=s
    if(x1<=15 or x2 <=15 or  x3 <=15):
        xa=-s*1.25
        xb=-s*1.25
    else:
        if(x1<30 or x2<60 or x3<60):
            if(x2>x3):
                xb=(s(abs(x3-30))*x5)/2*x6
            else:
                xa=(s+(abs(x2-30))*x6)/2*x5
    z1 = Twist()
    z1.linear.x = xa
    z1.linear.z = xb
    pub.publish(z1)
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('mod_msg', Twist, queue_size=10)
    rospy.Subscriber("ultra_msg", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()