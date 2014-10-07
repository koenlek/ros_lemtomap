ROS LEMTOMap
===================

Installation
------------
This was tested on Indigo.
(as of this writing, Turtlebot packages aren't officially released as .debs for Indigo, if these packages are still not available, follow the instructions in INSTALL-pre-turtlebot-indigo-release.md to build those packages yourself)

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

`roslaunch lemto_launchers topological_mapping_and_topological_navigation_sim.launch`

Tip: You can experiment by setting:
rolling:=false (default is true)
windowsize:=20 (default is 20)

For example:
`roslaunch lemto_launchers topological_mapping_and_topological_navigation_sim.launch rolling:=false`

#### Drive around manually ####

Use the arrow keys to increment speed, hit space to reset. If you get an error about motors not being powered up, hit 'e'.

#### Send a metric goal ####

Just use the RVIV '2D Nav Goal' tool on any unknown space (should be within the global costmap however)

#### Send a topological goal ####

You can send it to any topological node using this command. The RVIZ select tool can be used to choose a proper target_node_id. 

```
rostopic pub /move_base_topo/goal lemto_navigation/GotoNodeActionGoal "header:
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


### Load and Store topological maps ###

save a map (execute for example from ~/catkin_ws/src/lemto_map_server/maps):

`rosrun lemto_map_server save_map -f my_toponav_map`

load a map:

`roslaunch lemto_launchers topological_mapping_and_topological_navigation_sim.launch load_map_directory:=$(rospack find lemto_map_server)/maps/my_toponav_map`

OR e.g. (`$(pwd)` expands to current directory, e.g. `./my_toponav_map`):

`roslaunch lemto_launchers topological_mapping_and_topological_navigation_sim.launch load_map_directory:="$(pwd)/my_toponav_map"`

Troubleshoot
------------
- A GPU related issue can cause simulated runs to fail. If you experience such issues, please change `gpu_ray` to `ray` and `libgazebo_ros_gpu_laser.so` to `libgazebo_ros_laser.so` in `lemto_description/urdf/lemto_turtle_gazebo.urdf.xacro`

