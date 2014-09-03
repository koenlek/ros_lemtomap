Gazebo perfect map generator
===================

This package can generate perfect maps of a world, which in term can be loaded to /map using the map_server package

Credits
-------------------------------------
This package was based on the code presented here:

https://bitbucket.org/brawner/collision_map_creator_plugin

Which is described here:

http://gazebosim.org/wiki/Tutorials/1.9/custom_messages

Usage
-------------------------------
Make sure that you at this line at the end of your .world file (before `</world>`):

`<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>`

### Start world

Then start the gazebo world:

`roslaunch st_gazebo_perfect_map_generator start_world.launch world_name:=<full path to map>`

For example:

`roslaunch st_gazebo_perfect_map_generator start_world.launch world_name:=$(rospack find st_gazebo)/worlds/willowgarage.world`

### Generate theperfect map files

Example:

`roslaunch st_gazebo_perfect_map_generator generate_map.launch map_name:=willowgarage_perfect save_folder:=$(rospack find st_gazebo_perfect_map_generator)/maps xmin:=-25 xmax:=45 ymin:=-30 ymax:=45 scan_height:=10 resolution:=0.05`

Tip: xmin is left and ymin is bottom in the resulting image, so use that to finetune the border values.

### Load generated map

Example:

`rosrun map_server map_server $(rospack find st_gazebo_perfect_map_generator)/maps/willowgarage_perfect.yaml`

#### Use the map with an non-default robot pose

Example: you spawn your robot at 6,-16 for willowgarage.world, you should update willowgarage_perfect.yaml manually:

Change this original line:

`origin: [-25.000000, -30.000000, 0.000000]`

To:

`origin: [-31, -14, 0.000000]`