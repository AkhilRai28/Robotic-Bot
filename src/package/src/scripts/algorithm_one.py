#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import acos, pi, sqrt, degrees
import logging
from dynamic_reconfigure.server import Server
from my_robot.cfg import ObstacleConfig

class Obstacle:
    def __init__(self):
        rospy.init_node('obstacle', anonymous=True)

        self.left_distance = None
        self.middle_distance = None
        self.right_distance = None

        self.min_distance_threshold = rospy.get_param('~min_distance_threshold', 10)
        self.max_distance_threshold = rospy.get_param('~max_distance_threshold', 100)

        self.safe_distance = rospy.get_param('~safe_distance', 20)
        self.turning_speed = rospy.get_param('~turning_speed', 0.5)
        self.forward_speed = rospy.get_param('~forward_speed', 0.2)

        self.setup_logging()
        self.setup_dynamic_reconfigure()

        rospy.Subscriber('ultrasound', Twist, self.ultrasound_callback)

        self.vel_pub = rospy.Publisher('mybot', Twist, queue_size=10)
        
    def setup_logging(self):
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger('ObstacleNode')
        self.logger.info("Obstacle node initialized")

    def setup_dynamic_reconfigure(self):
        self.server = Server(ObstacleConfig, self.dynamic_reconfigure_callback)

    def dynamic_reconfigure_callback(self, config, level):
        self.min_distance_threshold = config.min_distance_threshold
        self.max_distance_threshold = config.max_distance_threshold
        self.safe_distance = config.safe_distance
        self.turning_speed = config.turning_speed
        self.forward_speed = config.forward_speed
        self.logger.info(f"Dynamic reconfigure parameters updated: {config}")
        return config

    def ultrasound_callback(self, data):
        self.left_distance = data.linear.x
        self.middle_distance = data.linear.y
        self.right_distance = data.linear.z
        self.logger.debug(f"Ultrasound data received: left={self.left_distance}, middle={self.middle_distance}, right={self.right_distance}")

    def avoid_obstacle(self):
        twist_cmd = Twist()
        if self.middle_distance > self.min_distance_threshold:
            distance = sqrt((self.right_distance - self.left_distance) ** 2 + self.middle_distance ** 2)
            angle1 = acos((self.right_distance - self.left_distance) / distance)
            angle = degrees(angle1)
            twist_cmd.linear.x = (180 - angle) * distance / 100 * 3 * 0.63 * 1.35
            twist_cmd.linear.y = angle * distance / 100 * 3 * 0.63 * 1.35
            twist_cmd.linear.z = 1
            self.logger.info(f"Avoiding obstacle: angle={angle}, distance={distance}")
        else:
            twist_cmd.linear.z = 0
            twist_cmd.linear.y = 255
            twist_cmd.linear.x = 115
            self.logger.warning("Obstacle too close! Stopping and turning.")
        self.vel_pub.publish(twist_cmd)

    def diagnostics(self):

        diag_msg = Twist()
        diag_msg.linear.x = self.left_distance or 0
        diag_msg.linear.y = self.middle_distance or 0
        diag_msg.linear.z = self.right_distance or 0
        self.logger.info(f"Diagnostics: {diag_msg}")
        self.vel_pub.publish(diag_msg)

    def run(self):
        rate = rospy.Rate(10)
        self.logger.info("Obstacle node running")

        while not rospy.is_shutdown():
            if self.left_distance is not None and self.middle_distance is not None and self.right_distance is not None:
                self.avoid_obstacle()
            else:
                self.logger.debug("Waiting for ultrasound data...")

            self.diagnostics()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = Obstacle()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
        logging.exception("Unhandled exception")
