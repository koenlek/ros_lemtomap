/**
 * @file save_map.cpp
 * @brief This file defines the load_map node of the lemto_map_server package.
 * @author Koen Lekkerkerker
 */

#include <lemto_map_server/save_map.h>

/*!
 * StMapSaver constructor
 */
StMapSaver::StMapSaver(const std::string& mapname) :
    saved_map_(false), mapname_(mapname), callback_started_(false) {
  ros::NodeHandle n;
  toponav_map_topic_ = "topological_navigation_mapper/topological_navigation_map";
  ROS_INFO("Waiting for the toponavmap at topic: '%s'", toponav_map_topic_.c_str());
  toponavmap_sub_ = n.subscribe(toponav_map_topic_, 1, &StMapSaver::toponavmapCB, this);
  asso_node_servcli_ = n.serviceClient<lemto_topological_mapping::GetAssociatedNode>("topological_navigation_mapper/get_associated_node");

  ros::NodeHandle private_nh_("~");

  // Parameters used by our GMapping wrapper
  if (!private_nh_.getParam("no_associated_node_and_no_pose", no_associated_node_and_no_pose_))
    no_associated_node_and_no_pose_ = false;
}

/*!
 * \brief toponavmapCB
 */
void StMapSaver::toponavmapCB(lemto_topological_mapping::TopologicalNavigationMap toponav_map)
                              {
  if (callback_started_ == true) //execute once only!
    return;
  callback_started_ = true;
  ROS_INFO("Received map with %lu nodes and %lu edges",
           toponav_map.nodes.size(), toponav_map.edges.size());

  boost::filesystem::create_directories(mapname_);

  /* write metadata to .yaml file */
  YAML::Emitter out;
  //out << YAML::BeginSeq; // adds a -, everything after this gets a dash. It begins a kind of bulleted list
  //out << YAML::Flow << vector; // for e.g. a vector with squares to be saved as '- [1, 4, 9, 16]'
  //out << YAML::EndSeq;

  int associated_node;
  geometry_msgs::Pose pose;

  if (no_associated_node_and_no_pose_) {
    ROS_INFO("Defaulting associated node to node 1, and pose to (0,0,0)");
    associated_node = 1;
    pose.orientation.w=1;
  }
  else {
    associated_node = getAssociatedNode();
    pose = getRobotPose();
  }

  out << YAML::BeginSeq;
  out << YAML::BeginMap;
  out << YAML::Key << "nodes" << YAML::Value << toponav_map.nodes.size() << YAML::Comment("For the users information only");
  out << YAML::Key << "edges" << YAML::Value << toponav_map.edges.size() << YAML::Comment("For the users information only");
  out << YAML::Key << "associated_node" << YAML::Value << associated_node;
  out << YAML::EndMap;
  out << pose;
  out << YAML::EndSeq;

  // todo - p3 - also store map save time (either sim time or real time, plus a way to make clear if it was a sim or real time experiment).

  //std::cout << "Here's the output YAML:\n" << out.c_str() << "\n"; // prints the YAML

  std::string topnavmapmetadata_file = "toponav_map_metadata.yaml";
  FILE* topnavmap_metadata_yaml = fopen(std::string(mapname_ + "/" + topnavmapmetadata_file).c_str(), "w");
  if (!topnavmap_metadata_yaml) {
    ROS_ERROR("Couldn't save map file to %s",
              topnavmapmetadata_file.c_str());
    return;
  }
  fprintf(topnavmap_metadata_yaml, "%s", out.c_str());
  fclose(topnavmap_metadata_yaml);

  /* write TopologicalNavigationMap message to .bag file */
  /* TODO - p2 - Now that yaml-cpp is known, maybe we should not use .bag,
   *              but yaml instead for saving the TopologicalNavigationMap msg.
   *              Best would be to just add two functions: saveMsgAsBag and saveMsgAsYAML.
   *              Bag is easier and always works. YAML needs changes if the msg changes,
   *              but is more elegant.
   */
  rosbag::Bag bag;
  bag.open(mapname_ + "/toponav_map.bag", rosbag::bagmode::Write);
  bag.write(toponav_map_topic_, toponav_map.header.stamp, toponav_map);
  bag.close();

  ROS_INFO("Done saving\n");
  saved_map_ = true;
}

geometry_msgs::Pose StMapSaver::getRobotPose() {
  geometry_msgs::Pose robot_pose;
  tf::StampedTransform robot_transform_tf;

  try
  {
    tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2));
    tf_listener_.lookupTransform("map", "base_link", ros::Time(0), robot_transform_tf);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Error looking up transformation\n%s", ex.what());
  }

  robot_pose.position.x = robot_transform_tf.getOrigin().getX();
  robot_pose.position.y = robot_transform_tf.getOrigin().getY();
  robot_pose.position.z = robot_transform_tf.getOrigin().getZ();

  robot_pose.orientation.x = robot_transform_tf.getRotation().getX();
  robot_pose.orientation.y = robot_transform_tf.getRotation().getY();
  robot_pose.orientation.z = robot_transform_tf.getRotation().getZ();
  robot_pose.orientation.w = robot_transform_tf.getRotation().getW();

  return robot_pose;
}

int StMapSaver::getAssociatedNode() {
  lemto_topological_mapping::GetAssociatedNode srv;
  if (asso_node_servcli_.call(srv)) {
    ROS_DEBUG("Received AssoNode: %d", (int )srv.response.asso_node_id);
    return (int)srv.response.asso_node_id;
  }
  else {
    ROS_ERROR("Failed to call service get_associated_node");
    return -1;
  }
}

#define USAGE "Usage: \n" \
" save_map -h\n"\
" save_map -f <map_name> (stores all files in folder called 'map_name', full path needs to be specified, ~ is not accepted for home folder...)"

/************************ Below here, custom YAML emitter procedures are specified *******************************/
/*
 * This is the trick to easily export/import ROS messages from cpp to a yaml file and the other way around.
 */

YAML::Emitter& operator <<(YAML::Emitter& out, const geometry_msgs::Pose& pose) {

  out << YAML::BeginMap;
  out << YAML::Key << "pose" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "position" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "x" << YAML::Value << pose.position.x;
  out << YAML::Key << "y" << YAML::Value << pose.position.y;
  out << YAML::Key << "z" << YAML::Value << pose.position.z;
  out << YAML::EndMap;
  out << YAML::Key << "orientation" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "x" << YAML::Value << pose.orientation.x;
  out << YAML::Key << "y" << YAML::Value << pose.orientation.y;
  out << YAML::Key << "z" << YAML::Value << pose.orientation.z;
  out << YAML::Key << "w" << YAML::Value << pose.orientation.w;
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;

  return out;
}

/*!
 * Main
 */

int main(int argc, char** argv) {
  ros::init(argc, argv, "lemto_save_map");
  std::string mapname = "toponav_map";

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-h")) {
      puts(USAGE);
      return 0;
    }
    else if (!strcmp(argv[i], "-f")) {
      if (++i < argc) {
        mapname = argv[i];
        ROS_INFO("-f received, with path: '%s'", argv[i]);
      }
      else {
        puts(USAGE);
        return 1;
      }
    }
    else {
      puts(USAGE);
      return 1;
    }
  }

  StMapSaver map_saver(mapname);

  while (!map_saver.callback_started_ && ros::ok()) {
    ros::spinOnce();
  }

  while (!map_saver.saved_map_ && ros::ok()) {
  };

  return 0;
}
