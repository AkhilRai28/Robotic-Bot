#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float32
import logging

x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)

        self.kp = rospy.get_param('~kp', 1)
        self.ki = rospy.get_param('~ki', 0)
        self.kd = rospy.get_param('~kd', 0)

        self.min_distance_threshold = rospy.get_param('~min_distance_threshold', 10)
        self.max_distance_threshold = rospy.get_param('~max_distance_threshold', 20)

        self.setup_logging()
        self.setup_dynamic_reconfigure()

        self.pub = rospy.Publisher('mod_msg', Twist, queue_size=10)
        rospy.Subscriber('ultra_msg', Twist, self.modify)

    def setup_logging(self):
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger('ObstacleAvoidance')
        self.logger.info("ObstacleAvoidance node initialized")

    def setup_dynamic_reconfigure(self):
        self.server = Server(ObstacleConfig, self.dynamic_reconfigure_callback)

    def dynamic_reconfigure_callback(self, config, level):
        self.kp = config.kp
        self.ki = config.ki
        self.kd = config.kd
        self.min_distance_threshold = config.min_distance_threshold
        self.max_distance_threshold = config.max_distance_threshold
        self.logger.info(f"Dynamic reconfigure parameters updated: {config}")
        return config

    def pid(self, dis):
        x.pop(0)
        x.append(dis)
        p = dis
        i = sum(x)
        d = x[-1] - x[-2]
        speed = (self.kp * p) + (self.ki * i) + (self.kd * d)
        self.logger.debug(f"PID calculated speed: {speed}")
        return speed

    def modify(self, data):
        rev_data = data

        if rev_data.linear.x == 0:
            rev_data.linear.x = 65
        elif rev_data.linear.y == 0:
            rev_data.linear.y = 65
        elif rev_data.linear.z == 0:
            rev_data.linear.z = 65

        error = (-1 * rev_data.linear.x) + (1 * rev_data.linear.z)
        error_pid = self.pid(error)

        cal_data = Twist()
        cal_data.linear.x = error_pid * rev_data.linear.x
        cal_data.linear.z = error_pid * rev_data.linear.z

        send_data = Twist()

        if ((rev_data.linear.y < self.max_distance_threshold and rev_data.linear.y > self.min_distance_threshold) or 
            (rev_data.linear.z < self.max_distance_threshold and rev_data.linear.z > self.min_distance_threshold) or 
            (rev_data.linear.x < self.max_distance_threshold and rev_data.linear.x > self.min_distance_threshold)):
            send_data.linear.x = cal_data.linear.z
            send_data.linear.z = -cal_data.linear.x
        elif (rev_data.linear.y < self.min_distance_threshold or rev_data.linear.z < self.min_distance_threshold or rev_data.linear.x < self.min_distance_threshold):
            send_data.linear.x = -cal_data.linear.z * 2
            send_data.linear.z = -cal_data.linear.x * 2
        else:
            send_data.linear.x = cal_data.linear.z
            send_data.linear.z = cal_data.linear.x

        self.logger.debug(f"Sending modified data: {send_data.linear.x}, {send_data.linear.z}")
        self.pub.publish(send_data)

    def diagnostics(self):
        diag_msg = Twist()
        diag_msg.linear.x = sum(x) / len(x)
        diag_msg.linear.z = sum(z) / len(z)
        self.logger.info(f"Diagnostics: {diag_msg}")
        self.pub.publish(diag_msg)

    def run(self):
        rate = rospy.Rate(10)
        self.logger.info("ObstacleAvoidance node running")

        while not rospy.is_shutdown():
            self.diagnostics()
            rate.sleep()

def algorithm_two():
    node = ObstacleAvoidance()
    node.run()

if __name__ == '__main__':
    try:
        algorithm_two()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
        logging.exception("Unhandled exception")
