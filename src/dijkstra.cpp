#include <my_global_planner/dijkstra.h>

namespace my_global_planner{
    
    bool dijkstra(int start_idx, int goal_idx, int obstacle_thresh, costmap_2d::Costmap2D* costmap2D_ptr, std::vector<int>& plan){
        //f = g
        
        //read map info
        int map_size_x = costmap2D_ptr->getSizeInCellsX();

        std::map<int, int> parent;  // (child idx, parent idx) 
        std::map<int, float> gcost; //(idx, gcost)
        std::set<std::pair<float, int>> openList;  // (fcost, idx), nodes yet to be examine
        std::set<int> closedList;  // done examination

        //initialize the start_idx
        gcost[start_idx] = 0;
        openList.insert({gcost[start_idx], start_idx});
        
        while(!openList.empty()){
            //get the node with least fcost, and remove it from openList
            std::pair<float, int> cur = *openList.begin();
            openList.erase(openList.begin());

            int idx = cur.second;
            
            //if reach goal, end
            if(idx == goal_idx) break;
            
            //if already in closed list, skip 
            if(closedList.find(idx)!=closedList.end()) continue;

            //keep on spanning, get all valid neighbors
            std::vector<std::pair<int, float>> neighbors;
            find_neighbors(idx, obstacle_thresh, costmap2D_ptr, neighbors);
            
            for(std::pair<int,float>& p : neighbors){
                int next_idx = p.first;
                float step_cost = p.second;

                //1. if already closed, skip
                if(closedList.find(next_idx) != closedList.end()){
                    continue;
                }
                //2. if not closed yet
                float g = gcost[idx] + step_cost;
                
                //2.1 if already visited (i.e., in openList) and the new cost is not lower, skip
                if(gcost.find(next_idx) !=  gcost.end() &&  gcost[next_idx]<=g) continue;
                
                //2.2 else, update cost and add to open List
                gcost[next_idx] = g;
                parent[next_idx] = idx;
                openList.insert({gcost[next_idx], next_idx});
            }

            //add current node to the cloesd list
            closedList.insert(idx);
        }

        //if the goal is not reached, planning fails
        if(parent.find(goal_idx)==parent.end()) return false;
        
        //planning successful, backtrack
        plan.push_back(goal_idx);
        int child_idx = goal_idx;
        while(parent[child_idx] != start_idx){
            plan.insert(plan.begin(), parent[child_idx]);
            child_idx = parent[child_idx];
        }
        plan.insert(plan.begin(), start_idx);

        ROS_INFO("DIJKSTRA planning done!");

        return true;

    }
}