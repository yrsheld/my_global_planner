#include <my_global_planner/utils.h>

namespace my_global_planner{
    
    float distance(int idx_a, int idx_b, int map_width){
        //calculate the euclidean distance between idx_0 & idx_1
        
        //convert 1d idx to 2d grid cell
        std::pair<int, int> a = {int(floor(idx_a/map_width)), idx_a/map_width};
        std::pair<int, int> b = {int(floor(idx_b/map_width)), idx_b/map_width};

        return pow((a.first-b.first),2)+pow((a.second-b.second),2);
    }

    void find_neighbors(int cur_idx, int obstacle_thresh, costmap_2d::Costmap2D* costmap, std::vector<std::pair<int,float>>& neighbors){
        //find all neighbors that are in range and no obstacles
        int map_size_x = costmap->getSizeInCellsX();
        int map_size_y = costmap->getSizeInCellsY();

        //convert 1d index to 2d
        int x = (int)std::floor(cur_idx/map_size_x);
        int y = (int)cur_idx % map_size_x;

        //8 neighbors
        std::vector<std::pair<int, int>> adj = {{-1, 0}, {0, -1}, {0, 1}, {1, 0}};
        std::vector<std::pair<int, int>> diag = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

        float step = costmap->getResolution();  //meter/cell

        for(auto& d : adj){
            int nei_x = x+d.first, nei_y = y+d.second;

            if(nei_x<0 || nei_x>=map_size_x || nei_y<0 || nei_y>=map_size_y) continue;

            //check if occupied     
            if(costmap->getCost(nei_x, nei_y) > obstacle_thresh) continue;

            //add to valid neighbor list
            neighbors.push_back({nei_x*map_size_x+nei_y, step});
        }

        for(auto& d : diag){
            int nei_x = x+d.first, nei_y = y+d.second;

            if(nei_x<0 || nei_x>=map_size_x || nei_y<0 || nei_y>=map_size_y) continue;

            //check if occupied     
            if(costmap->getCost(nei_x, nei_y) > obstacle_thresh) continue;

            //add to valid neighbor list
            neighbors.push_back({nei_x*map_size_x+nei_y, step*1.414});
        }
        
        
    }
}