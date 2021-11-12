#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Int32MultiArray.h>
#include "amsl_navigation_msgs/Node.h"
#include "amsl_navigation_msgs/Edge.h"
#include "amsl_navigation_msgs/NodeEdgeMap.h"
class LocalGoalCreator
{
public:
    LocalGoalCreator();
    void process();

private:
    //method
    void global_path_callback(const nav_msgs::Path::ConstPtr&);
    void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void path_callback(const std_msgs::Int32MultiArray::ConstPtr&);
    void select_next_goal();
    //parameter
    int hz;
    double border_distance;

    //member
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher pub_local_goal;
    ros::Publisher pub_estimated_edge;
    ros::Subscriber sub_global_path;
    ros::Subscriber sub_current_pose;
    ros::Subscriber sub_path;
    nav_msgs::Path global_path;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped local_goal;
    std_msgs::Int32MultiArray global_path_num;
    amsl_navigation_msgs::Edge estimated_edge;
    unsigned int goal_number;
    bool have_recieved_path = false;
    bool have_recieved_multi_array = false;
    bool have_recieved_pose = false;
};

LocalGoalCreator::LocalGoalCreator():private_nh("~")
{
    //parameter
    private_nh.param("hz",hz,{10});
    private_nh.param("border_distance",border_distance,{2.0});
    //subscriber
    sub_global_path = nh.subscribe("/global_path/path_path",10,&LocalGoalCreator::global_path_callback,this);
    sub_current_pose = nh.subscribe("/ekf_pose",10,&LocalGoalCreator::current_pose_callback,this);
    sub_path = nh.subscribe("/global_path/path",10,&LocalGoalCreator::path_callback,this);
    //publisher
    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",1);
    pub_estimated_edge = nh.advertise<amsl_navigation_msgs::Edge>("/estimated_pose/edge", 1);
}

void LocalGoalCreator::path_callback(const std_msgs::Int32MultiArray::ConstPtr& msg_path)
{
    std::cout<<"path callback "<<std::endl;
    global_path_num = *msg_path;
    have_recieved_multi_array = true;
}
void LocalGoalCreator::global_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    std::cout<<"global_path callback "<<std::endl;
    global_path=*msg;
    std::cout<<"global_path size: " << global_path.poses.size() << std::endl;
    goal_number = 0;
    local_goal = global_path.poses[goal_number]; // goal_number番目の位置
    have_recieved_path = true;
}
void LocalGoalCreator::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    std::cout<<"current_pose callback "<<std::endl;
    current_pose = *msg;
    // if(!have_recieved_pose) 
    have_recieved_pose = true;
}
void LocalGoalCreator::select_next_goal()
{
    double measure_distance = sqrt(pow(local_goal.pose.position.x-current_pose.pose.position.x,2)+pow(local_goal.pose.position.y-current_pose.pose.position.y,2)); // 自分の位置と次の通過ポイントの距離
    std::cout<<"distance: "<< measure_distance<<std::endl;
    if(measure_distance < border_distance) goal_number += 1;
    if(global_path.poses.size() > goal_number) local_goal = global_path.poses[goal_number];
    else local_goal = global_path.poses[global_path.poses.size()-1];
    if(goal_number == 0) goal_number ++;
    std::cout<<"goal_number: "<< goal_number <<std::endl;
    estimated_edge.node0_id = global_path_num.data[goal_number-1]; // 最後に通過したnode
    estimated_edge.node1_id = global_path_num.data[goal_number];
    std::cout<<"estimated_edge.node0: "<< estimated_edge.node0_id << " node1: " << estimated_edge.node1_id <<std::endl;
    pub_estimated_edge.publish(estimated_edge);
 }

void LocalGoalCreator::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(have_recieved_path && have_recieved_multi_array )
        {
            select_next_goal();
            local_goal.header.frame_id = "map";
            std::cout<<"local_goal :"<<local_goal.pose.position.x<<","<<local_goal.pose.position.y<<std::endl;
            pub_local_goal.publish(local_goal);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main (int argc,char **argv)
{
    ros::init(argc, argv, "local_goal_creator");
    LocalGoalCreator local_goal_creator;
    local_goal_creator.process();
    return 0;
}