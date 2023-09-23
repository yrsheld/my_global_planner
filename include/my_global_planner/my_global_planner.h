#ifndef MY_GLOBAL_PLANNER_H_
#define MY_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "astar.h"
#include "dijkstra.h"

namespace my_global_planner{
    
    class MyGlobalPlanner : public nav_core::BaseGlobalPlanner{
    public:
        MyGlobalPlanner();
        MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal, 
                    std::vector<geometry_msgs::PoseStamped>& plan);
    private:
        bool isValid(int x, int y);
        bool CoordToGrid(float& x, float& y);
        bool GridToCoord(float& x, float& y);
        size_t GridToIndex(float x, float y);
        void IndexToGrid(int idx, float& x, float& y);

        costmap_2d::Costmap2DROS* costmap2D_ros_ptr_;
        costmap_2d::Costmap2D* costmap2D_ptr_;  //2D costmap
        //std::vector<int> costmap1D_; //1D costmap

        float origin_x_, origin_y_; // origin of costmap
        float resolution_;  // meter/cell
        int size_x_, size_y_; //number of cells in x & y
        int size_; //the total size of map
        float meters_x_, meters_y_;
        int obstacle_thresh_; //threshold value of obstacle

        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
        ros::Publisher planPub_;
        
    }; 



}
#endif