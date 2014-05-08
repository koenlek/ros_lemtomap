ros_semantic_turtle
===================

create a map:
roslaunch st_tests basic_topological_mapping_nav_sim.launch

save a map (for example from ~/catkin_ws/src/st_map_server/maps):
rosrun st_map_server save_map -f my_toponav_map

load a map:
roslaunch st_tests basic_topological_mapping_nav_sim.launch load_map_directory:=$(rospack find st_map_server)/maps/my_toponav_map