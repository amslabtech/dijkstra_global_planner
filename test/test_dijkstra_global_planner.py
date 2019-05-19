#!/usr/bin/env python
import rospy, unittest, rostest
import rosnode
import time
from std_msgs.msg import Int32MultiArray

class DijkstraGlobalPlannerTest(unittest.TestCase):
    def setUp(self):
        self.count=0
        rospy.Subscriber('/global_path/path', Int32MultiArray, self.callback)
        self.global_path = Int32MultiArray()

    def callback(self, data):
        self.count += 1
        self.global_path = data

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/global_planner/dijkstra', nodes, "node does not exist")

    def test_dijkstra(self):
        time.sleep(3)
        self.assertFalse(self.count == 0, "cannot subscribe the topic")
        a = [0,1,2,3,4,7,9,10,11,1,0]
        for i in range(len(a)):
            self.assertEqual(self.global_path.data[i], a[i], "not collect answer")

if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('test_dijkstra_global_planner')
    rostest.rosrun('dijkstra_global_planner', 'test_dijkstra_global_planner', DijkstraGlobalPlannerTest)
