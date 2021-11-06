#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include <std_msgs/Int32MultiArray.h>
#include <amsl_navigation_msgs/Node.h>
#include <amsl_navigation_msgs/Edge.h>
#include <amsl_navigation_msgs/NodeEdgeMap.h>
#include <amsl_navigation_msgs/Replan.h>
class GlobalPathCreator
{
public:
    GlobalPathCreator();
    void process();

private:
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);
    void path_callback(const std_msgs::Int32MultiArray::ConstPtr&);
    void node_edge_map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr&);
    void make_global_path();

    bool replan_flag;
    bool node_edge_flag;
    bool global_path_flag;
    ros::NodeHandle n;
    ros::NodeHandle private_n;
  
    ros::Publisher pub_path;
    ros::Publisher pub_open_grid;
    ros::Subscriber sub_map;
    ros::Subscriber sub_path;
    ros::Subscriber node_edge_map_sub;
    ros::Subscriber check_point_sub;

    nav_msgs::Path global_path;
    nav_msgs::Path waypoint_path;
    amsl_navigation_msgs::NodeEdgeMap map;
    std_msgs::Int32MultiArray check_points;
    std_msgs::Int32MultiArray global_path_num;
};

GlobalPathCreator::GlobalPathCreator() :private_n("~")
{
    // subscriber
    node_edge_map_sub = n.subscribe("/node_edge_map/map",1, &GlobalPathCreator::node_edge_map_callback, this);
    sub_path = n.subscribe("/global_path/path",10,&GlobalPathCreator::path_callback,this);
    
    //publisher
    pub_path = n.advertise<nav_msgs::Path>("/global_path/path_path",1);
    replan_flag = false;
    node_edge_flag = false;
    global_path_flag = false;
}


void GlobalPathCreator::node_edge_map_callback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg_map)
{
    map = *msg_map;
    std::cout << "subscribe map information" << std::endl;
}

void GlobalPathCreator::path_callback(const std_msgs::Int32MultiArray::ConstPtr& msg_path)
{
    std::cout << "subscribe global_path" << std::endl;
    global_path_num = *msg_path;
    global_path_flag = true;
    make_global_path();  
}

void GlobalPathCreator::make_global_path()
{
    int count_i = 0;
    int count_j = 0;
    for(auto path : global_path_num.data){
        for(auto n : map.nodes){
            if(path == n.id){
                // std::cout << "path; " << path << std::endl;
                geometry_msgs::PoseStamped temp_path_point;
                temp_path_point.pose.position.x = n.point.x;
                temp_path_point.pose.position.y = n.point.y;
                temp_path_point.header.frame_id = "map"; 
                global_path.poses.push_back(temp_path_point);
            }
            count_j ++;
        }
        count_i ++;
    }
    std::cout << "i: " << count_i << " j: " << count_j << std::endl;
    std::cout << "global_path poses size:  " << global_path.poses.size() << std::endl;
    global_path.header.frame_id = "map";
    // while(!replan_flag) // 応急処置
    for(int i = 0; i < 1000000; i++) // for visualize
        pub_path.publish(global_path);
}
void GlobalPathCreator::process()
{
    while(ros::ok()){
        ros::spinOnce();
    }
}
int main (int argc, char **argv)
{
  ros::init(argc,argv,"global_path_creator");
  GlobalPathCreator global_path_creator;
  global_path_creator.process();
  return 0;
}