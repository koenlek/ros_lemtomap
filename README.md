ROS LEMTOMap
===================

Installation
------------
This was tested on Hydro.

- Create a catkin ws (or use your existing catkin ws):
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make 
source devel/setup.bash  //add to .bashrc for convenience
```
- Add and compile the LEMTOMap Metapackage
```
roscd
cd ../src
wstool init
wstool set lemtomap_meta --git git://github.com/koenlek/ros_lemtomap.git
wstool update lemtomap_meta
rosdep install --from-paths . -i -y
cd ..
catkin_make
```

Use `catkin_make -DCMAKE_BUILD_TYPE=Release` for (faster) release build, `catkin_make -DCMAKE_BUILD_TYPE=Debug` for (slower) debuggable build. `catkin_make` will remember last build type you selected, so subsequent builds dont need the flag unless you want to change.

### Update ###
```
roscd
cd ../src
wstool update lemtomap_meta
rosdep install --from-paths . -i -y
cd ..
catkin_make
```

Usage
-----

### Create and navigate a topological map ###

`roslaunch st_tests topological_mapping_and_topological_navigation_sim.launch`

Tip: You can experiment by setting:
rolling:=true (default is false)
windowsize:=20 (default is 20, is neglected if rolling is false)
perfect_odometry:=true (default is false)

For example:
`roslaunch st_tests topological_mapping_and_topological_navigation_sim.launch rolling:=true`

#### Drive around manually ####

Use the arrow keys to increment speed, hit space to reset. If you get an error about motors not being powered up, hit 'e'.

#### Send a metric goal ####

Just use the RVIV '2D Nav Goal' tool on any unknown space (should be within the global costmap however)

#### Send a topological goal ####

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


### Load and Store topological maps (under development) ###

save a map (execute for example from ~/catkin_ws/src/st_map_server/maps):

`rosrun st_map_server save_map -f my_toponav_map`

load a map:

`roslaunch st_tests topological_mapping_and_topological_navigation_sim.launch load_map_directory:=$(rospack find st_map_server)/maps/my_toponav_map`

OR e.g.:
`roslaunch st_tests topological_mapping_and_topological_navigation_sim.launch load_map_directory:="$(pwd)/my_toponav_map" //$(pwd) expands to current directory, i.e. "./my_toponav_map"`

