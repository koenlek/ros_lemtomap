ROS Semantic Turtle
===================

Create and navigate a topological map
-------------------------------------
`roslaunch st_tests topological_mapping_and_topological_navigation_sim.launch`

### Send a topological goal

You can send it to any topological node using this command. The RVIZ select tool can be used to choose a proper target_node_id. 

```
rostopic pub /move_base_topo/goal st_navigation/GotoNodeActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_node_id: 1" -1
```


Load and Store topological maps
-------------------------------


save a map (execute for example from ~/catkin_ws/src/st_map_server/maps):

`rosrun st_map_server save_map -f my_toponav_map`

load a map:

`roslaunch st_tests topological_mapping_and_topological_navigation_sim.launch load_map_directory:=$(rospack find st_map_server)/maps/my_toponav_map`