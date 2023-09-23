#ifndef MY_GLOBAL_PLANNER_DIJKSTRA_H_
#define MY_GLOBAL_PLANNER_DIJKSTRA_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>
#include "utils.h"

namespace my_global_planner{
    bool dijkstra(int start_idx, int goal_idx, int obstacle_thresh, costmap_2d::Costmap2D* costmap2D_ptr, std::vector<int>& plan);
}

#endif