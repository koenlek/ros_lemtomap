Changelog for metapackage semantic_turtle
=========================================

-----------------------------------------

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