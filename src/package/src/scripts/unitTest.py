import unittest
from obstacle_avoidance import ObstacleAvoidance
from geometry_msgs.msg import Twist

class TestObstacleAvoidance(unittest.TestCase):

    def setUp(self):
        self.node = ObstacleAvoidance()
        self.node.kp = 1
        self.node.ki = 0
        self.node.kd = 0

    def test_pid(self):
        self.node.pid(15, -1)
        self.assertEqual(self.node.pid(15, -1), 15)

    def test_modify(self):
        data = Twist()
        data.linear.x = 15
        data.linear.y = 15
        data.linear.z = 15
        self.node.modify(data)
        self.assertTrue(self.node.pub.get_num_connections() > 0)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('obstacle_avoidance', 'test_obstacle_avoidance', TestObstacleAvoidance)
