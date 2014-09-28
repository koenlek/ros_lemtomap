ROS LEMTOMap without official turtlebot deb packages
===================

Follow these instructions only if turtlebot packages have not yet been released for Indigo. Check if there is a tab 'indigo' on this page http://wiki.ros.org/turtlebot/
If not, then follow these instructions. Otherwise, follow the instructions in the README.md

Procedure to get turtlebot packages from source (tested 27-09-2014)
------------

```
mkdir ~/ws_turtlebot
cd ~/ws_turtlebot
wstool init src -j5 https://raw.github.com/yujinrobot/yujin_tools/master/rosinstalls/indigo/turtlebot.rosinstall

cd src
wstool remove turtlebot_create #currently, it will not compile because turtlebot_create_desktop is not working, so we'll get rid of turtlebot_create in total...
sudo rm -r turtlebot_create*
cd ..
source /opt/ros/indigo/setup.bash
```

Change these files to remove all 'create' related dependencies:
```
~/ws_turtlebot/src/turtlebot_viz/turtlebot_dashboard/package.xml
remove line:  <run_depend>create_dashboard</run_depend>
~/ws_turtlebot/src/turtlebot/turtlebot_bringup/package.xml
remove line:  <run_depend>create_node</run_depend>
~/ws_turtlebot/src/turtlebot_simulator/turtlebot_gazebo/package.xml
remove line:  <run_depend>create_gazebo_plugins</run_depend>
~/ws_turtlebot/src/turtlebot/turtlebot_description/package.xml
remove line:  <run_depend>create_description</run_depend>
~/ws_turtlebot/src/turtlebot_simulator/turtlebot_stdr/package.xml
remove line:  <run_depend>stdr</run_depend>
```
```
rosdep install --from-paths src -i -y
# now we need to fix some bug, as it will likely crash at some point (https://github.com/turtlebot/turtlebot/issues/139 )
sudo dpkg --remove --force-remove-reinstreq libopenni-sensor-pointclouds0
sudo apt-get install -f

rosdep install --from-paths src -i -y
catkin_make
source ~/ws_turtlebot/devel/setup.bash
```

Continue normal installation
---------------

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

Tip on overlaying workspaces
---------------
This should go properly automatically with the instructions above. However you can also manually check/change it in `~/catkin_ws/devel/_setup_util.py`. Just check for the line (put your own username at USER):

    CMAKE_PREFIX_PATH = '/home/USER/catkin_ws/devel;/home/USER/ws_turtlebot/devel;/opt/ros/indigo'.split(';')
Make sure it look something like the above, which defines how your workspaces overlay each other...

