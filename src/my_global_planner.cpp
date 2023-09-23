#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <my_global_planner/my_global_planner.h>

PLUGINLIB_EXPORT_CLASS(my_global_planner::MyGlobalPlanner, nav_core::BaseGlobalPlanner);

namespace my_global_planner{
    
    MyGlobalPlanner::MyGlobalPlanner() {;}
    MyGlobalPlanner::MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void MyGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle nh("~/" + name);
        
        //do some initialization
        costmap2D_ros_ptr_ = costmap_ros;
        costmap2D_ptr_ = costmap2D_ros_ptr_->getCostmap();

        //get information of the costmap
        resolution_ = costmap2D_ptr_->getResolution();
        origin_x_ = costmap2D_ptr_->getOriginX();    
        origin_y_ = costmap2D_ptr_->getOriginY();
        size_x_ = costmap2D_ptr_->getSizeInCellsX();  //size of map [cells]
        size_y_ = costmap2D_ptr_->getSizeInCellsY();
        meters_x_ = size_x_*resolution_;  //size of map [meters]
        meters_y_ = size_y_*resolution_;
        size_ = size_x_ * size_y_;
        obstacle_thresh_ = 200;

        //create 1D costmap
        /*costmap1D_ = std::vector<int>(size_);
        for(int i=0;i<size_;i++){
            int y = i % size_x_;
            int x = (int)std::floor(i/size_x_);
            costmap1D_.at(i) = (int)costmap2D_ptr_->getCost(x, y);
        }*/

        planPub_ = nh.advertise<visualization_msgs::MarkerArray>("/plan_MarkerArray", 1);
        
    }
    
    bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        float start_x = start.pose.position.x;
        float start_y = start.pose.position.y;
        float goal_x = goal.pose.position.x;
        float goal_y = goal.pose.position.y;

        //convert world coordinates to 2D grid coordinate
        if(!CoordToGrid(start_x, start_y) || !CoordToGrid(goal_x, goal_y)){
            ROS_INFO("Fail: Start/Goal pose out of range....");
            return false;
        }

        //check if start & goal are both valid (without obstacles)
        if(!isValid(start_x, start_y) || !isValid(goal_x, goal_y)){
            ROS_INFO("Fail: Start/Goal position already occupied....");
            return false;
        }

        //conver 2d grid coordinate to 1D index
        int start_idx = static_cast<int>(GridToIndex(start_x, start_y));
        int goal_idx = static_cast<int>(GridToIndex(goal_x, goal_y));
        
        std::vector<int> index_plan;
        ROS_INFO("[My global planner] Start Planning.....");
        
        //astar, dijkstra
        if(dijkstra(start_idx, goal_idx, obstacle_thresh_, costmap2D_ptr_, index_plan)){
            //successful planning
            //process the index plan, convert each index to PoseStamped
            for(auto& idx : index_plan){
                float x, y;
                IndexToGrid(idx, x, y);  //1d index --> 2d grid cell
                //ROS_INFO("PLAN INDEX: (%f, %f)", x, y);  //pring (x, y) coordinate
                if(!GridToCoord(x, y)){
                    ROS_INFO("Fail: planned path out of range....");
                    return false;
                }

                geometry_msgs::PoseStamped ps;
                ps.header.frame_id = start.header.frame_id;
                ps.pose.position.x = x;
                ps.pose.position.y = y;
                ps.pose.orientation.x = 0;
                ps.pose.orientation.y = 0;
                ps.pose.orientation.z = 0;
                ps.pose.orientation.w = 1;

                //add current pose to plan
                plan.push_back(ps);
            }
            
            plan.push_back(goal);

            //plan visualization
            publishPlan(plan);
        }
        else{
            //planning failed
            ROS_INFO("Fail: Could not find a plan....");
            return false;
        }

        return true;
    }

    bool MyGlobalPlanner::isValid(int x, int y){
        //check whether current grid is occupied
        return costmap2D_ptr_->getCost(x, y) < obstacle_thresh_;
    }
    
    bool MyGlobalPlanner::CoordToGrid(float& x, float& y){
        x = static_cast<int>((x-origin_x_)/resolution_);
        y = static_cast<int>((y-origin_y_)/resolution_);

        //check whether in range [cells]
        return x>0 && x<size_x_ && y>0 && y<size_y_;
    }

    bool MyGlobalPlanner::GridToCoord(float& x, float& y){
        x = x*resolution_ + origin_x_;
        y = y*resolution_ + origin_y_;

        //check whether in range [meters]
        return x>origin_x_ && x<meters_x_ && y>origin_y_ && y<meters_y_;
    }

    size_t MyGlobalPlanner::GridToIndex(float x, float y){
        return x * size_x_ + y;
    }

    void MyGlobalPlanner::IndexToGrid(int idx, float& x, float& y){
        y = static_cast<float>(idx % size_x_);
        x = floor(idx / size_x_);
    }

    void MyGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
        
        visualization_msgs::MarkerArray arr;
        
        std::string frame_id = plan[0].header.frame_id;
        int id = 0;
        for(auto& p : plan){
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.id = id++;
 
            marker.type = visualization_msgs::Marker::SPHERE;
        
            marker.action = visualization_msgs::Marker::ADD;
   
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = p.pose.position.x;
            marker.pose.position.y = p.pose.position.y;
            marker.pose.position.z = p.pose.position.z;
            marker.pose.orientation.x = p.pose.orientation.x;
            marker.pose.orientation.y = p.pose.orientation.y;
            marker.pose.orientation.z = p.pose.orientation.z;
            marker.pose.orientation.w = p.pose.orientation.w;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
          
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;
   
            marker.lifetime = ros::Duration(10);
            
            arr.markers.push_back(marker);    
        }

        planPub_.publish(arr);
    }


}