#include <my_global_planner/astar.h>

namespace my_global_planner{
    
    float heuristic(int idx, int goal_idx, int map_width){
        //heurstic function - eucliean distance to the goal index
        return distance(idx, goal_idx, map_width);
    }

    bool astar(int start_idx, int goal_idx, int obstacle_thresh, costmap_2d::Costmap2D* costmap2D_ptr, std::vector<int>& plan){
        
        //read map info
        int map_size_x = costmap2D_ptr->getSizeInCellsX();

        std::map<int, int> parent;  // (child idx, parent idx) 
        std::map<int, float> gcost; //(idx, gcost)
        std::map<int, float> fcost; //(idx, fcost)
        std::set<std::pair<float, int>> openList;  // (fcost, idx), nodes yet to be examined
        std::set<int> closedList;  // done examination

        //initialize the start_idx
        gcost[start_idx] = 0;
        float h = heuristic(start_idx, goal_idx, map_size_x);
        fcost[start_idx] = h;  //g+h = 0+h
        openList.insert({fcost[start_idx], start_idx});
        
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
                if(gcost.find(next_idx) != gcost.end()){  
                    //2.1 if already visited (i.e., in openList)
                    if(g<gcost[next_idx]){
                        //update if lower cost
                        fcost[next_idx] += (g-gcost[next_idx]);
                        gcost[next_idx] = g;
                        parent[next_idx] = idx;
                        openList.insert({fcost[next_idx], next_idx});
                    }
                }
                else{
                    //2.2 new node! (i.e., never seen before)
                    float h = heuristic(next_idx, goal_idx, map_size_x);
                    float f = g+h;
                    fcost[next_idx] = f;
                    gcost[next_idx] = g;
                    parent[next_idx] = idx;
                    openList.insert({fcost[next_idx], next_idx});
                }
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

        ROS_INFO("ASTAR planning done!");

        return true;

    }
}