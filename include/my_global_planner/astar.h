#ifndef MY_GLOBAL_PLANNER_ASTAR_H_
#define MY_GLOBAL_PLANNER_ASTAR_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>
#include "utils.h"

namespace my_global_planner{
    float heuristic(int idx, int goal_idx, int map_width);
    bool astar(int start_idx, int goal_idx, int obstacle_thresh, costmap_2d::Costmap2D* costmap2D_ptr, std::vector<int>& plan);
}

#endif