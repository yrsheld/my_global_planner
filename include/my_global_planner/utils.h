#ifndef MY_GLOBAL_PLANNER_UTILS_H_
#define MY_GLOBAL_PLANNER_UTILS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>
#include <math.h>

namespace my_global_planner{

     //calculate the euclidean distance between two 1D indices
    float distance(int idx_a, int idx_b, int map_width);

    //find valid neighbors of current 1D index in the map
    void find_neighbors(int cur_idx, int obstacle_thresh, costmap_2d::Costmap2D* costmap, std::vector<std::pair<int,float>>& neighbors);

}

#endif