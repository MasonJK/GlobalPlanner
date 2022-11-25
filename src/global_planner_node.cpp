#include <GlobalPlanner/dijstra.h>
#include <GlobalPlanner/astar.h>
#include <GlobalPlanner/global_planner.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;

    GlobalPlanner global_planner;

    // ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        global_planner.run();
        // loop_rate.sleep();
    }
    
    return 0;
}