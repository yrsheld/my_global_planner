# my_global_planner
Global path planner implemented as ROS plugin


## How to use
### Specify the algorithm
By specifying which function to use in method - `MyGlobalPlanner::makePlan`, the planning would be based on different algorthms.

Current avaialble options:
* A*
* Dijkstra

## Demo
* To use the plugin as global planner
   * set the ros param `base_global_planner` of move_base node to `my_global_planner/MyGlobalPlanner`
* To launch the whole simulation scenario
   * ```roslaunch my_global_planner plan.launch```

## Result

The global plan is visualized as the green line.

<img src=dijkstra_demo.gif height="300">