#!/usr/bin/env python
#! coding:utf-8
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray 
from visualization_msgs.msg import Marker, MarkerArray

from amsl_navigation_msgs.msg import NodeEdgeMap

class GlobalPathViz:
    def __init__(self):
        rospy.init_node('global_path_viz')

        #Publisher
        self.global_path_marker_pub = rospy.Publisher('/node_edge_map/viz/global_path', MarkerArray, queue_size=1, latch=True)

        #Subscriber
        self.node_edge_map_sub = rospy.Subscriber('/node_edge_map/map', NodeEdgeMap, self.node_edge_map_callback)
        self.global_path_sub = rospy.Subscriber('/global_path', Int32MultiArray, self.global_path_callback)

        self.global_path_marker = MarkerArray()
        self.global_path = Int32MultiArray()
        self.node_edge_map = NodeEdgeMap()
        self.sub_map = False

    def node_edge_map_callback(self, msg):
        self.node_edge_map = msg
        self.sub_map = True

    def global_path_callback(self, msg):
        self.global_path = msg

    def process(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.sub_map:
                self.make_global_path_marker()
                self.global_path_marker_pub.publish(self.global_path_marker)

            rate.sleep()

    def make_global_path_marker(self):
        global_path_marker = MarkerArray()
        time = rospy.get_rostime()
        for i in range(len(self.global_path.data)-1):
            current_node_id = self.global_path.data[i]
            next_node_id = self.global_path.data[i+1]
            x = (self.node_edge_map.nodes[current_node_id].point.x + self.node_edge_map.nodes[next_node_id].point.x ) / 2.0
            y = (self.node_edge_map.nodes[current_node_id].point.y + self.node_edge_map.nodes[next_node_id].point.y ) / 2.0
            yaw = np.arctan2(self.node_edge_map.nodes[next_node_id].point.y - self.node_edge_map.nodes[current_node_id].point.y, self.node_edge_map.nodes[next_node_id].point.x - self.node_edge_map.nodes[current_node_id].point.x)
            x += np.cos(np.pi*0.5-yaw)
            y -= np.sin(np.pi*0.5-yaw)
            length  = 0.0
            for j in range(len(self.node_edge_map.edges)):
                if self.node_edge_map.edges[j].node0_id == current_node_id: 
                    if self.node_edge_map.edges[j].node1_id == next_node_id: 
                        length = self.node_edge_map.edges[j].distance*0.3

            n = Marker()
            n.ns = "global_path"
            n.header.frame_id = self.node_edge_map.header.frame_id
            n.header.stamp = time
            n.id = i 
            n.action = Marker().ADD
            n.type = Marker().ARROW
            n.lifetime = rospy.Duration()
            self.set_marker_scale(n, length, 0.5, 0.1)
            self.set_marker_rgb(n, 1., 1., 1.)
            self.set_marker_position(n, x, y, 1.0)
            self.set_marker_orientation(n, 0, 0, yaw)
            global_path_marker.markers.append(n)
        self.global_path_marker = global_path_marker

    def set_marker_scale(self, marker, x, y, z):
	marker.scale.x = x
	marker.scale.y = y
	marker.scale.z = z

    def set_marker_position(self, marker, x, y, z):
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = z

    def set_marker_rgb(self, marker, r, g, b, a=0.7):
	marker.color.r = r
	marker.color.g = g
	marker.color.b = b
	marker.color.a = a

    def set_marker_orientation(self, marker, r, p, y):
	q = tf.transformations.quaternion_from_euler(r, p, y)
	marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

if __name__=='__main__':
    global_path_viz = GlobalPathViz()
    global_path_viz.process()
