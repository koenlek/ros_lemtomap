#ifndef TOPONAV_NODE_H
#define TOPONAV_NODE_H

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <map>
#include <algorithm> //std::find

/*
 * The TopoNavNode class can be used to create TopoNavNode objects.
 * These objects form the nodes and all their properties that
 * together with the edges make the TopoNavMap.
 */


class TopoNavNode {

public:
	typedef int NodeID; //This can be used to help function signatures see the difference between a node_id form any int
	typedef std::map<NodeID, TopoNavNode*> NodeMap; // ptrs are needed, as std::map makes a COPY when adding elements.

	TopoNavNode(tf::Pose pose, bool is_door, int area_id, NodeMap &nodes);
	TopoNavNode(NodeID node_id, ros::Time last_updated, ros::Time last_pose_updated, tf::Pose pose,
			bool is_door, int area_id, NodeMap &nodes); // only to be used when loading map from message!

	~TopoNavNode();

	/**
	 * Public Methods
	 */

	// get Methods
	const NodeID getNodeID() const {
		return node_id_;
	}
	const ros::Time getLastUpdatedTime() const {
		return last_updated_;
	}
        const ros::Time getLastUpdatedPoseTime() const {
                return last_pose_updated_;
        }
	const int getAreaID() const {
		return area_id_;
	}
	const tf::Pose getPose() const {
		return pose_;
	}
	const bool getIsDoor() const {
		return is_door_;
	}
	// set Methods
	void setAreaID(int area_id) {
		area_id_ = area_id;
		last_updated_ = ros::Time::now();
	}
	void setPose(tf::Pose pose) { //should only be used for small updates to the pose
		pose_ = pose;
		last_updated_ = ros::Time::now();
		last_pose_updated_ = ros::Time::now();
	}
	void setIsDoor(bool is_door) {
		is_door_ = is_door;
		last_updated_ = ros::Time::now();
	}

private:
	/**
	 * Variables
	 */
	NodeID node_id_; //node_ids should never be changed!
	ros::Time last_updated_;
	ros::Time last_pose_updated_; //used to check if edge costs need to be recalculated
	tf::Pose pose_; //TODO - p3 - It would be better to use a Stamped Pose, relative frames would become possible (not all will be defined in /map, but e.g. in previous frame). setPose could still be kept, not much would have to change...

	bool is_door_; /*TODO - p2 - Doors should generally be treated much differently from normal nodes, maybe it is easier and makes more sense to not define them as nodes at all, or as a derived class or something like that, otherwise, you need to put in a lot of bool checks to see everytime it if is a normal node or a door node...
	                * Alternatively normal nodes can be collected in the NodeMap nodes_ while doors are collected in a DoorMap (NodeMap doors_) or so?
	                */

	int area_id_; //an area is a collection of nodes, in general areas would be rooms. But in future, large spaces or outdoor spaces could be divided in smaller areas, like the coffee corner, lunch corner and sitting area in the TU Delft Aula building.
	NodeMap &nodes_;

	/** TODO - p3 - add some "Properties of space" msg type that I still need to implement. This should get objects, local room size (by local I mean: as measured at this node), local room shape. All defined using semantic/symbolic labels with a prob. distribution of the relevant collection of such labels (e.g. room size 0.9 large, 0.07 medium, 0.04 small).
	 *  add room_type, also as a semantic/symbolic probability distribution of all possible room types.
	 */

	/**
	 * Private Methods
	 */

protected:
	static int UIDGenerator_; //generates a unique ID for every new node.

};

#endif // TOPONAV_NODE_H

