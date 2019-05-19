import rospy, unittest, rostest
import rosnode
fro std_msgs.msg import Int32MultiArray

class DijkstraGlobalPlannerTest(unittest.TestCase):
    def setUp(self):
        rospy.Subscriber('/global_path/path', IntMultiArray, self.callback)
        self.global_path = Int32MultiArray()

    def callback(self, data):
        self.global_path = data

    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/dijkstra', nodes, "node does not exist")

    def test_dijkstra(self):
        self.assertFalse(self.count == 0, "cannot subscribe the topic")
        a = [0,1,2,3,4,7,9,10,11,0]
        self.assertEqual(self.global_path.data, a, "not collect answer")

if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('dijkstra_global_planner_test')
    rostest.rosrun('dijkstra_global_planner', 'test_dijkstra_global_planner', DijkstraGlobalPlannerTest)
