#pragma once
#include <ros/ros.h>
#include <GlobalPlanner/global_planner_algorithm.h>
#include <GlobalPlanner/dijstra.h>
#include <cmath>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <queue>
#include <unordered_map>
#include <vector>
#include <geometry_msgs/PointStamped.h>

class NodeAStar : public NodeDijstra{
public:
    double h_cost_;
    NodeAStar* parent_;

    NodeAStar(){
        h_cost_ = INT_MAX;
        parent_ = nullptr;
    }
};

struct cmpAStar{
    bool operator()(NodeDijstra* n1, NodeDijstra* n2){
        if(n1->g_cost_ >= n2->g_cost_)
            return true;
        else
            return false;
    }

    bool operator()(NodeAStar* n1, NodeAStar* n2){
        if((n1->g_cost_ + n1->h_cost_) >= (n2->g_cost_ + n2->h_cost_))
            return true;
        else
            return false;
    }
};

class AStar : public GlobalPlannerAlgorithm{
protected:
    NodeAStar** node_map;
    NodeAStar* current_node;
    Coordinate start_, goal_;
    std::priority_queue<NodeAStar*, std::vector<NodeAStar*>, cmpAStar> open_list;
    // NodeAStar가 key가 되고, parent NodeAStar가 value가 되는 closed_list
    std::unordered_map<NodeAStar*, NodeAStar*> closed_list;

public:
    AStar(nav_msgs::OccupancyGrid &map);
    ~AStar();
    void initializeMap();
    void calculatePath();
    virtual nav_msgs::Path generatePath(nav_msgs::Odometry &start, geometry_msgs::PoseStamped &goal);
    void updateCost();
    double gFunction(int x1, int y1, int x2, int y2);
    double hFunction(int x1, int y1);
};

AStar::AStar(nav_msgs::OccupancyGrid &map): GlobalPlannerAlgorithm(map){
    // map generation
    node_map = new NodeAStar*[width_];
    for(int i = 0; i< width_; i++)
        node_map[i] = new NodeAStar[height_];

    // populating map
    for(int i = 0; i<width_; i++){
        for(int j = 0; j<height_; j++){
            node_map[i][j].set_position(i,j);
            if(map.data[coordinateToData(Coordinate(i,j))] == 0)
                node_map[i][j].occupied_ = false;
            else
                node_map[i][j].occupied_ = true;
        }
    }

    // algorithm start setting
    current_node = &node_map[start_.x_][start_.y_];
    current_node->g_cost_ = 0;
    current_node->h_cost_ = hFunction(current_node->coordinate_.x_, current_node->coordinate_.y_);

    std::cout<<"AStar constructor finished!"<<std::endl;
}

AStar::~AStar(){
    for(int i = 0; i<width_; i++)
        delete node_map[i];
    delete node_map;
}

double AStar::gFunction(int x1, int y1, int x2, int y2){
    double g = sqrt(pow(x1-x2,2) + pow(y1-y2, 2));

    return g;
}

double AStar::hFunction(int x1, int y1){
    double h = sqrt(pow(x1-goal_.x_ ,2) + pow(y1-goal_.x_, 2));

    return h;
}


void AStar::initializeMap(){
    // empty open&closed list and path
    while(!open_list.empty())
        open_list.pop();
    closed_list.clear();
    path_.poses.clear();
    // reset gcost
    for(int i = 0; i<width_; i++){
        for(int j = 0; j<height_; j++){
            node_map[i][j].g_cost_ = INT_MAX;
            node_map[i][j].h_cost_ = INT_MAX;
            node_map[i][j].parent_ = nullptr;
        }
    }
    // algorithm start setting
    current_node = &node_map[start_.x_][start_.y_];
    current_node->g_cost_ = 0;
    current_node->h_cost_ = hFunction(current_node->coordinate_.x_, current_node->coordinate_.y_);
}

void AStar::calculatePath(){
    while(current_node != &node_map[goal_.x_][goal_.y_]){ 
        // visit all the neighbors
        for(int i = -1; i <= 1; i++){
            for(int j = -1; j <= 1; j++){
                int neighbor_x = current_node->coordinate_.x_ + i;
                int neighbor_y = current_node->coordinate_.y_ + j;
                // if the neighbor is out of boundary, or occupied, then skip
                if(neighbor_x < 0 || neighbor_x >= width_ || neighbor_y < 0 || neighbor_y >= height_ || (neighbor_x==0&&neighbor_y==0) || node_map[neighbor_x][neighbor_y].occupied_ == true)
                    continue;
                double temp_g_cost = gFunction(current_node->coordinate_.x_, current_node->coordinate_.y_, neighbor_x, neighbor_y) + current_node->g_cost_;
                // if the node has a g_cost that is higher than the one proposing now, than we update it and add it in the open list
                if(node_map[neighbor_x][neighbor_y].g_cost_ > temp_g_cost){
                    node_map[neighbor_x][neighbor_y].g_cost_ = temp_g_cost;
                    node_map[neighbor_x][neighbor_y].h_cost_ = hFunction(node_map[neighbor_x][neighbor_y].coordinate_.x_, node_map[neighbor_x][neighbor_y].coordinate_.y_);
                    node_map[neighbor_x][neighbor_y].parent_ = current_node;
                    // even if the node is in open_list, because we updated the value, we insert again
                    open_list.push(&node_map[neighbor_x][neighbor_y]);
                }
            }
        }
        // choose next current_node, and insert it into closed list
        current_node = open_list.top();
        open_list.pop();

        // checking if it is already in closed_list, if so, find the best one that is not
        while(true){
            auto finder = closed_list.find(current_node);
            if (finder != closed_list.end()){
                // std::cout << "skipping.. already in closed list"<< std::endl;
                current_node = open_list.top();
                open_list.pop();
                continue;
            }else
                break;
        }
        closed_list.insert(std::make_pair(current_node, current_node->parent_));
    }
}


nav_msgs::Path AStar::generatePath(nav_msgs::Odometry &start, geometry_msgs::PoseStamped &goal){

    start_ = poseToCoordinate(start.pose.pose);
    goal_ = poseToCoordinate(goal.pose);
    
    initializeMap();
    calculatePath();

    // backtrack closed_list starting from goal node to start node
    std::queue<NodeAStar*> temp_path;

    // backtrack closed_list starting from goal to start
    NodeAStar* printer = &node_map[goal_.x_][goal_.y_];
    while(true){
        if(printer == 0){
            std::cout<<"fail to produce path!"<< std::endl;
            break;
        }
        if(printer == &node_map[start_.x_][start_.y_]){
            temp_path.push(printer);
            break;
        }        
        temp_path.push(printer);
        printer = closed_list[printer];
    }
    // flip and generate path
    while(!temp_path.empty()){
        geometry_msgs::PoseStamped path_pose;
        path_pose.header = header_;
        path_pose.pose = coordinateToPose(temp_path.front()->coordinate_);
        temp_path.pop();
        path_.poses.push_back(path_pose);
    }

    header_.seq++;
    header_.stamp = ros::Time::now();
    path_.header = header_;
    return path_;
}