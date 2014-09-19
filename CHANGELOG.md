Changelog for metapackage lemtomap
=========================================

-----------------------------------------

0.4.0 (2014-07-16)
------------------
* Topological mapping now by default creates as much edges as allowed based on if the edge is navigable in the global costmap (adherring the max topological distance limit to limit the risky of faulty topological loop closures).
* Fixed rolling window gmapping: it now truly forgets old measurements and resizes each particle's internal map.
* Difference between odom edges and near neighbour edges.
* Switched to `global_planner` as global planner, instead of `navfn`.
* A lot of other tweaks, including the option to forbid the global planner to plan through unknown space.

### Known issues ###
* It will likely not run well in real life experiments, as local transform relies on perfect odometry...
* Apart from that none, a lot of experiments are on the way, which will likely reveal some issues/bugs.
* When using a gazebo world with the willowgarage model, loads of errors will be thrown on exiting the related roslaunch using ctrl+c (first noticed since 2014-09-06)

0.3.0 (2014-06-22)
------------------
* `st_gmapping_rolling` now supports setting the `minimumScore` to more than 0, limiting chattering/jumping of gmapping pose estimates in large open spaces.
* `st_topological_mapping` now supplies and updates node details such as adjacent nodes, distance maps, and predessor maps when requested
* `st_navigation` updated to use `st_topological_mapping` for graph/node details

### Main features ###
* Topological mapping and navigation
* Ability to use gmapping as a rolling window, relying on topology outside the window
* Topological global metric consistency is unimportant, by applying a tf transform it should always be  locally consistent.

### Known issues ###
* The transform between the occupancy grid map and topological map is currently provided by a function that relies on a perfect odometry assumption. Gazebo provides perfect odometry for Kobuki base, thus this works fine in simulation. In future, a local localization method needs to be implemented to provide this transform.
* Chattering/jumping between transforms can occur, causing the topo map to jump around and many nodes to be created in this process. A fundamental change is likely needed to solve this, still need to think about that...

-----------------------------------------