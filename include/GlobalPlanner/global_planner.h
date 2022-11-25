#pragma once
#include <ros/ros.h>
#include <GlobalPlanner/coordinate.h>
#include <GlobalPlanner/global_planner_algorithm.h>
#include <GlobalPlanner/dijstra.h>
#include <GlobalPlanner/astar.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class GlobalPlanner{
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber goal_sub;
    ros::Publisher path_pub;

    nav_msgs::OccupancyGrid map;
    nav_msgs::Path path;
    nav_msgs::Odometry odom_;
    geometry_msgs::PoseStamped goal_;

    GlobalPlannerAlgorithm* global_planner_algorithm;

    bool map_flag, odom_flag, goal_flag, initialize_flag;

    clock_t start, finish;
    double duration;
    
    std::string algorithm_name;
    bool print_time;
public:
    GlobalPlanner();
    ~GlobalPlanner();

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    
    void run();
    void initializePlanner();
};

GlobalPlanner::GlobalPlanner(){
    map_sub = nh_.subscribe("/map", 1, &GlobalPlanner::mapCallback, this);
    odom_sub = nh_.subscribe("odom", 1, &GlobalPlanner::odomCallback, this);
    goal_sub = nh_.subscribe("/move_base_simple/goal", 1, &GlobalPlanner::goalCallback, this);
    path_pub = nh_.advertise<nav_msgs::Path>("/global_path", 10);

    map_flag = false;
    odom_flag = false;
    goal_flag = false;
    initialize_flag = true;

    nh_.param<std::string>("/global_planner_algorithm", algorithm_name, "astar");
    nh_.param<bool>("/print_time", print_time, true);
}

GlobalPlanner::~GlobalPlanner(){
    delete global_planner_algorithm;
}

void GlobalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    odom_ = *msg;
    if(!odom_flag)
        std::cout<<"Received Odometry!"<<std::endl;
    odom_flag = true;
}

void GlobalPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goal_ = *msg;
    goal_flag = true;
    std::cout<<"Received Goal!"<<std::endl;
}

void GlobalPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    map = *msg;
    if(!map_flag)
        std::cout<<"Received Map!"<<std::endl;
    map_flag = true;
}

void GlobalPlanner::initializePlanner(){
    if(algorithm_name == "dijstra"){
        global_planner_algorithm = new Dijstra(map);
        std::cout<<"Using Dijstra Algorithm for Global Planner"<<std::endl;
    }else if(algorithm_name == "astar"){
        global_planner_algorithm = new AStar(map);
        std::cout<<"Using AStar Algorithm for Global Planner"<<std::endl;
    }else
        ROS_ERROR("ERROR: no such algorithm in global_planner_algorithm");
}

void GlobalPlanner::run(){
    if(map_flag && odom_flag && goal_flag){
        start = clock();
        if(initialize_flag){
            initializePlanner();
            initialize_flag = false;
        }
        path = global_planner_algorithm->generatePath(odom_, goal_);
        path_pub.publish(path);
        finish = clock();

        if(print_time){
            duration = (double)(finish - start) / CLOCKS_PER_SEC;
            std::cout<< "runtime: " << duration << " seconds" << std::endl;
        }
    }
}