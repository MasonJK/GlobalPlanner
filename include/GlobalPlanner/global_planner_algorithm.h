#pragma once
#include <ros/ros.h>
#include <GlobalPlanner/coordinate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>

class GlobalPlannerAlgorithm{
protected:
    nav_msgs::OccupancyGrid map_;
    nav_msgs::Odometry start_;
    geometry_msgs::PoseStamped goal_;
    nav_msgs::Path path_;
    std_msgs::Header header_;
    geometry_msgs::Pose origin_;
    int width_, height_;
    float resolution_;
public:
    GlobalPlannerAlgorithm(nav_msgs::OccupancyGrid &map);
    ~GlobalPlannerAlgorithm();

    virtual nav_msgs::Path generatePath(nav_msgs::Odometry &start, geometry_msgs::PoseStamped &goal) = 0;
    int coordinateToData(Coordinate coordinate);
    Coordinate poseToCoordinate(geometry_msgs::Pose pose);
    geometry_msgs::Pose coordinateToPose(Coordinate coordinate);
};

GlobalPlannerAlgorithm::GlobalPlannerAlgorithm(nav_msgs::OccupancyGrid &map){
    map_ = map;
    width_ = map.info.width;
    height_ = map.info.height;
    resolution_ = map.info.resolution;
    origin_ = map.info.origin;
    header_.frame_id = map.header.frame_id;
    header_.seq = 0;
}

GlobalPlannerAlgorithm::~GlobalPlannerAlgorithm(){}

int GlobalPlannerAlgorithm::coordinateToData(Coordinate coordinate){
    return (coordinate.y_-1)*width_ + coordinate.x_;
}

Coordinate GlobalPlannerAlgorithm::poseToCoordinate(geometry_msgs::Pose pose){
    Coordinate coordinate;
    coordinate.x_ = int(round((pose.position.x - origin_.position.x)/resolution_));
    coordinate.y_ = int(round((pose.position.y - origin_.position.y)/resolution_));
    return coordinate;
}

geometry_msgs::Pose GlobalPlannerAlgorithm::coordinateToPose(Coordinate coordinate){
    geometry_msgs::Pose pose;
    pose.position.x = (coordinate.x_ * resolution_) + origin_.position.x;
    pose.position.y = (coordinate.y_ * resolution_) + origin_.position.y;
    return pose;
}