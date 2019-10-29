#include "ros/ros.h"
#include <std_msgs/Int32MultiArray.h>
#include <amsl_navigation_msgs/Node.h>
#include <amsl_navigation_msgs/Edge.h>
#include <amsl_navigation_msgs/NodeEdgeMap.h>
#include <amsl_navigation_msgs/Replan.h>

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

class Dijkstra{
    public:
        Dijkstra();
        void NodeEdgeMapCallback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg);
        void CheckPointCallback(const std_msgs::Int32MultiArrayConstPtr& msg);
        void CurrentEdgeCallback(const amsl_navigation_msgs::EdgeConstPtr& msg);
        bool ReplanHandler(amsl_navigation_msgs::Replan::Request&, amsl_navigation_msgs::Replan::Response&);
    private:
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        //subscriber
        ros::Subscriber node_edge_map_sub;
        ros::Subscriber check_point_sub;
        ros::Subscriber current_edge_sub;

        //publisher
        ros::Publisher global_path_pub;

        // service server
        ros::ServiceServer replan_server;

        std::vector<int> CalcDijkstra(std::vector<Node>, int, int);
        void SetCurrentEdge(amsl_navigation_msgs::Edge&);
        void MakeAndPublishGlobalPath();
        int childid2index(Node, int);
        int id2index(std::vector<Node>, int);

        std::vector<Node> nodes;
        std::vector<int> checkpoints;
        std::vector<amsl_navigation_msgs::Edge> edges;
        amsl_navigation_msgs::Edge current_edge;
        int num_checkpoints=-1;
        int num_nodes=-1;
        bool sub_current_edge = false;
        bool first_sub_edge_flag = true;
        bool first_pub_path = false;
        amsl_navigation_msgs::Edge first_edge;
        int INIT_NODE0_ID;
        int INIT_NODE1_ID;
        double INIT_PROGRESS;
};

Dijkstra::Dijkstra()
    : private_nh("~")
{
    //subscriber
    node_edge_map_sub = nh.subscribe("/node_edge_map/map",1, &Dijkstra::NodeEdgeMapCallback, this);
    check_point_sub = nh.subscribe("/node_edge_map/checkpoint",1, &Dijkstra::CheckPointCallback, this);
    current_edge_sub = nh.subscribe("/estimated_pose/edge",1, &Dijkstra::CurrentEdgeCallback, this);

    //publisher
    global_path_pub = nh.advertise<std_msgs::Int32MultiArray>("/global_path/path",1,true);

    // service server
    replan_server = nh.advertiseService("/global_path/replan", &Dijkstra::ReplanHandler, this);

    private_nh.param("INIT_NODE0_ID", INIT_NODE0_ID, {0});
    private_nh.param("INIT_NODE1_ID", INIT_NODE1_ID, {1});
    private_nh.param("INIT_PROGRESS", INIT_PROGRESS, {0.0});
    first_edge.node0_id = INIT_NODE0_ID;
    first_edge.node1_id = INIT_NODE1_ID;
    first_edge.progress = INIT_PROGRESS;
}

void Dijkstra::CheckPointCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
    std_msgs::Int32MultiArray check_points = *msg;
    checkpoints.clear();
    for(auto check_point : check_points.data){
        checkpoints.push_back(check_point);
    }
    num_nodes = nodes.size();
    if(!first_pub_path && num_nodes>0){
        SetCurrentEdge(first_edge);
        MakeAndPublishGlobalPath();    
        first_pub_path = true;
    }
}

void Dijkstra::NodeEdgeMapCallback(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
    amsl_navigation_msgs::NodeEdgeMap map = *msg;
    nodes.clear();
    edges.clear();
    for(auto edge : map.edges){
        edges.push_back(edge);
    }
    for(auto n : map.nodes){
        int id = n.id;
        std::vector<int> child_id;
        std::vector<double> child_cost;
        for(auto edge : edges){
            if(edge.node0_id == id){
                child_id.push_back(edge.node1_id);
                child_cost.push_back(edge.distance);
            }
        }
        int num_child = child_id.size();
        int i = 0;
        Node node(id, n.type, child_id,child_cost);
        nodes.push_back(node);
        std::cout << "-------------" << std::endl;
        std::cout << "node:"  << id << std::endl;
        for(auto id: child_id){
            std::cout << "child["<< i <<"]:"  << id 
                << "(" << child_cost[i] << ")" << std::endl;
            i++;
        }
    }
    int num_checkpoints = checkpoints.size();
    if(!first_pub_path && num_checkpoints>0){
        SetCurrentEdge(first_edge);
        MakeAndPublishGlobalPath();    
        first_pub_path = true;
    }
}

void Dijkstra::CurrentEdgeCallback(const amsl_navigation_msgs::EdgeConstPtr& msg)
{
    amsl_navigation_msgs::Edge _edge = *msg;
    SetCurrentEdge(_edge);
    if(first_sub_edge_flag){
        MakeAndPublishGlobalPath();
        first_sub_edge_flag = false;
    }
    sub_current_edge = true;
}

bool Dijkstra::ReplanHandler(amsl_navigation_msgs::Replan::Request& request, amsl_navigation_msgs::Replan::Response& response)
{
    SetCurrentEdge(request.edge);
    MakeAndPublishGlobalPath();
    response.succeeded = true;
    return true;
}

std::vector<int> Dijkstra::CalcDijkstra(std::vector<Node> nodes, int start_id, int goal_id)
{
    // std::cout << "start_id:" << start_id << std::endl;
    // std::cout << "goal_id:" << goal_id << std::endl;
    nodes[id2index(nodes,start_id)].open = true;
    bool found = false;
    bool resign = false;
    while(!found and !resign){
        //update open list
        int min_id = -1;
        float min_cost = 100000;
        for(auto n : nodes){
            if(n.open==true){
                if(n.cost < min_cost){
                    min_cost = n.cost;
                    min_id = n.id;
                }
            }
        }
        // std::cout << "min_id: " << min_id << std::endl;
        int next_id = min_id;
        if(min_id == -1){
            resign = true;
            std::cout << "resign" << std::endl;
        }else if(next_id == goal_id){
            found = true;
            // std::cout << "goal" << std::endl;
        }else{
            nodes[id2index(nodes, next_id)].open = false;
            int i = 0;
            for(auto child_id : nodes[id2index(nodes,next_id)].child_id){
                ////////
                //TODO//
                ////////
                int next_child_id = id2index(nodes,nodes[id2index(nodes,next_id)].child_id[i]);

                if(nodes[next_child_id].close == false){
                    nodes[next_child_id].open = true;
                    nodes[next_child_id].close = true;
                    nodes[next_child_id].parent = next_id;
                    nodes[next_child_id].cost = nodes[id2index(nodes,next_id)].cost + nodes[id2index(nodes,next_id)].child_cost[i];
                }
                i++;
            }
        }
    }
    std::vector<int> path;
    if(found){
        int id = goal_id;
        while(id != start_id){
            path.push_back(id);
            id = nodes[id2index(nodes,id)].parent;
        }
        std::reverse(path.begin(), path.end());
    }
    return path;
}

void Dijkstra::SetCurrentEdge(amsl_navigation_msgs::Edge& edge)
{
    current_edge = edge;
    int i = 0;
    for(auto n : nodes){
        if(n.type=="add_node"){
            nodes.erase(nodes.begin()+i);
            checkpoints.erase(checkpoints.begin());
        }
        i++;
    }
    if(current_edge.node0_id!=checkpoints[0]){
        if(current_edge.progress != 0.0){
            std::string type = "add_node";
            std::vector<int> child_id;
            std::vector<double> child_cost;
            if(current_edge.impassable){
                child_id.push_back(current_edge.node0_id);
                child_cost.push_back(current_edge.distance*current_edge.progress);
            }else{
                child_id.push_back(current_edge.node0_id);
                child_id.push_back(current_edge.node1_id);
                child_cost.push_back(current_edge.distance*current_edge.progress);
                child_cost.push_back(current_edge.distance*(1.0-current_edge.progress));
            }
            int add_node_id = nodes.size();
            Node node(add_node_id, type, child_id, child_cost);
            nodes.push_back(node);
            checkpoints.insert(checkpoints.begin(), add_node_id);
        }else{
            checkpoints.insert(checkpoints.begin(), current_edge.node0_id);
        }
    }
    // int i = 0;
    // std::cout << "-----------------------" << std::endl;
    // for(auto p : checkpoints){
    //     std::cout << "checkpoints[" << i << "]:" << p << std::endl;
    //     i++;
    // }
}

void Dijkstra::MakeAndPublishGlobalPath()
{
    std::cout << "-----------------------" << std::endl;
    std_msgs::Int32MultiArray global_path;
    num_checkpoints = checkpoints.size();
    num_nodes = nodes.size();
    if(num_checkpoints != -1 and num_nodes != -1){
        if(nodes[id2index(nodes,checkpoints[0])].type!="add_node"){
             global_path.data.push_back(checkpoints[0]);
        }
        for(int i=0; i<num_checkpoints-1; i++){
            std::vector<int> path;
            // std::cout << checkpoints[i] << " to "<< checkpoints[i+1] << std::endl;
            path = CalcDijkstra(nodes,checkpoints[i],checkpoints[i+1]);
            for(auto p : path){
                global_path.data.push_back(p);
            }
        }
        // global_path.data.erase(global_path.data.begin());
        std::cout << "------------------------------------------" << std::endl;
        std::cout << "global_path" << std::endl;
        for(auto path : global_path.data){
            std::cout << path << std::endl;
        }
        std::cout << "------------------------------------------" << std::endl;
        global_path_pub.publish(global_path);
    }
}

int Dijkstra::childid2index(Node node, int id){
    int i = 0;
    for(auto child_id :node.child_id){
        if(child_id == id){
            return i;
        }
        i++;
    }
    return -1;
}

int Dijkstra::id2index(std::vector<Node> nodes, int id){
    int i = 0;
    for(auto node : nodes){
        if(node.id == id){
            return i;
        }
        i++;
    }
    return -1;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dijkstra");

    Dijkstra dijkstra;

    ros::spin();
    return 0;
}
