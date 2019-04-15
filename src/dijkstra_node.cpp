#include "ros/ros.h"
#include <std_msgs/Int32MultiArray.h>
#include <amsl_navigation_msgs/Node.h>
#include <amsl_navigation_msgs/Edge.h>
#include <amsl_navigation_msgs/NodeEdgeMap.h>

class Node
{
	public:
		Node(int _id, std::string _type, std::vector<int> _child_id, std::vector<double> _child_cost){
			id = _id;
			type = _type;
			cost = 0;
			open = false;
			close = false;
			parent = -1;
			child_id = _child_id;
			child_cost = _child_cost;
		}
		int id;
		std::string type;
		double cost;
		bool open;
		bool close;
		int parent;
		std::vector<int> child_id;
		std::vector<double> child_cost;

};

std::vector<Node> nodes;
std::vector<int> checkpoints;
std::vector<amsl_navigation_msgs::Edge> edges;
amsl_navigation_msgs::Edge current_edge;
int num_checkpoints=-1;
int num_nodes=-1;
bool sub_current_edge = false;

void NodeEdgeMapCallback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
	amsl_navigation_msgs::NodeEdgeMap map = *msg;
	num_nodes = map.nodes.size();
	int num_edges = map.edges.size();
	for(int i=0; i<num_edges; i++){
		edges.push_back(map.edges[i]);
	}
	for(int i=0; i<num_nodes; i++){
		int id = map.nodes[i].id;
		std::vector<int> child_id;
		std::vector<double> child_cost;
		for(int j=0; j<num_edges; j++){
			if(edges[j].node0_id == id){
				child_id.push_back(edges[j].node1_id);
				child_cost.push_back(edges[j].distance);
			}
			if(edges[j].node1_id == id){
				child_id.push_back(edges[j].node0_id);
				child_cost.push_back(edges[j].distance);
			}
		}
		int num_child = child_id.size();
		Node node(id, map.nodes[i].type,child_id,child_cost);
		nodes.push_back(node);
		std::cout << "-------------" << std::endl;
		std::cout << "node:"  << id << std::endl;
		for(int k=0; k<num_child; k++){
			std::cout << "child["<< k <<"]:"  << child_id[k] << std::endl;
			std::cout << "child_cost["<< k <<"]:"  << child_cost[k] << std::endl;
		}
	}
}

void CheckPointCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
	std_msgs::Int32MultiArray check_points = *msg;
	num_checkpoints = check_points.data.size();
	for(int i=0;i<num_checkpoints; i++){
		checkpoints.push_back(check_points.data[i]);
	}
}

void CurrentEdgeCallback(const amsl_navigation_msgs::EdgeConstPtr& msg)
{
	current_edge = *msg;
	std::string type = "add_node";
	std::vector<int> child_id;
	child_id.push_back(current_edge.node0_id);
	child_id.push_back(current_edge.node1_id);
	std::vector<double> child_cost;
	child_cost.push_back(current_edge.distance*current_edge.progress);
	child_cost.push_back(current_edge.distance*(1.0-current_edge.progress));
	Node node(num_nodes+1,type,child_id,child_cost);
	sub_current_edge = true;
}

std::vector<int> Dijkstra(std::vector<Node> nodes, int start_id, int goal_id)
{
	nodes[start_id].open = true;
	bool found = false;
	bool resign = false;
	while(!found and !resign){
		//update open list
		int min_id = -1;
		float min_cost = 100000;
		for(int i=0; i< num_nodes; i++){
			if(nodes[i].open==true){
				if(nodes[i].cost < min_cost){
					min_cost = nodes[i].cost;
					min_id = nodes[i].id;
				}
			}
		}
		int next_id = min_id;
		if(min_id == -1){
			resign = true;
		}else if(next_id == goal_id){
			found = true;
			//std::cout << "goal" << std::endl;
		}else{
			nodes[next_id].open = false;
			for(int i=0; i<nodes[next_id].child_id.size(); i++){
				if(nodes[nodes[next_id].child_id[i]].close == false){
					nodes[nodes[next_id].child_id[i]].open = true;
					nodes[nodes[next_id].child_id[i]].close = true;
					nodes[nodes[next_id].child_id[i]].parent = next_id;
					nodes[nodes[next_id].child_id[i]].cost = 
						nodes[next_id].cost + nodes[next_id].child_cost[i];
				}
			}
		}
	}
	std::vector<int> path;
	int id = goal_id;
	while(id != start_id){
		path.push_back(id);
		id = nodes[id].parent;
	}
	std::reverse(path.begin(), path.end());
	return path;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dijkstra");
    ros::NodeHandle nh;
	//subscriber
    ros::Subscriber node_edge_map_sub = nh.subscribe("/node_edge_map",1,NodeEdgeMapCallback);
    ros::Subscriber check_point_sub = nh.subscribe("/node_edge_map/checkpoint",1,CheckPointCallback);
    ros::Subscriber current_edge_sub = nh.subscribe("/estimated_pose/edge",1,CurrentEdgeCallback);

	//publisher
    ros::Publisher global_path_pub = nh.advertise<std_msgs::Int32MultiArray>("/global_path",100,true);
    
	ros::Rate loop_rate(10);
	while(ros::ok()){
		std::cout << "-----------------------" << std::endl;
		std_msgs::Int32MultiArray global_path;
		if(num_checkpoints != -1 and num_nodes != -1){
			std::cout << checkpoints[0] << std::endl;
			// global_path.data.push_back(checkpoints[0]);
			for(int i=0; i<num_checkpoints-1; i++){
				std::vector<int> path;
				// std::cout << checkpoints[i] << " to "<< checkpoints[i+1] << std::endl;
				path = Dijkstra(nodes,checkpoints[i],checkpoints[i+1]);
				for(int j=0;j<path.size();j++){
					global_path.data.push_back(path[j]);
				}
			}
			for(int i=0;i<global_path.data.size();i++){
				std::cout << global_path.data[i] << std::endl;
			}
			global_path_pub.publish(global_path);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}
