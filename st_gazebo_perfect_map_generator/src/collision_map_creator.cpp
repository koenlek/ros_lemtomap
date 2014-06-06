#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "collision_map_request.pb.h"

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

namespace gazebo {
typedef const boost::shared_ptr<
		const collision_map_creator_msgs::msgs::CollisionMapRequest> CollisionMapRequestPtr;
class CollisionMapCreator: public WorldPlugin {
	transport::NodePtr node;
	transport::PublisherPtr imagePub;
	transport::SubscriberPtr commandSubscriber;
	physics::WorldPtr world;

	std::string filename_noext;
	std::string map_name;

public:
	void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
		node = transport::NodePtr(new transport::Node());
		world = _parent;
		// Initialize the node with the world name
		node->Init(world->GetName());
		ROS_INFO_STREAM( "Subscribing to: " << "~/collision_map/command");
		commandSubscriber = node->Subscribe("~/collision_map/command",
				&CollisionMapCreator::receive, this);
		imagePub = node->Advertise<msgs::Image>("~/collision_map/image");
	}

public:
	void receive(CollisionMapRequestPtr &msg) {
		ROS_INFO_STREAM( "Received message" );

		filename_noext=msg->filename();
		map_name=filename_noext.substr(filename_noext.find_last_of("/")+1,filename_noext.size()-1);
		ROS_INFO("map_name is: %s", map_name.c_str());


		ROS_INFO_STREAM( "\nCreating collision map with corners at ("
				<< msg->upperleft().x() << ", " << msg->upperleft().y()
				<< "), (" << msg->upperright().x() << ", "
				<< msg->upperright().y() << "), (" << msg->lowerright().x()
				<< ", " << msg->lowerright().y() << "), ("
				<< msg->lowerleft().x() << ", " << msg->lowerleft().y()
				<< ") with collision projected from z = " << msg->height()
				<< "\nResolution = " << msg->resolution() << " m\n"
				<< "Occupied spaces will be filled with: " << msg->threshold()
				);

		double z = msg->height();
		double dX_vertical = msg->upperleft().x() - msg->lowerleft().x();
		double dY_vertical = msg->upperleft().y() - msg->lowerleft().y();
		double mag_vertical = sqrt(
				dX_vertical * dX_vertical + dY_vertical * dY_vertical);
		dX_vertical = msg->resolution() * dX_vertical / mag_vertical;
		dY_vertical = msg->resolution() * dY_vertical / mag_vertical;

		double step_s = msg->resolution();

		double dX_horizontal = msg->upperright().x() - msg->upperleft().x();
		double dY_horizontal = msg->upperright().y() - msg->upperleft().y();
		double mag_horizontal = sqrt(
				dX_horizontal * dX_horizontal + dY_horizontal * dY_horizontal);
		dX_horizontal = msg->resolution() * dX_horizontal / mag_horizontal;
		dY_horizontal = msg->resolution() * dY_horizontal / mag_horizontal;

		int count_vertical = mag_vertical / msg->resolution();
		int count_horizontal = mag_horizontal / msg->resolution();

		if (count_vertical == 0 || count_horizontal == 0) {
			ROS_INFO_STREAM( "Image has a zero dimensions, check coordinates"
					);
			return;
		}
		double x, y;
		double dist;
		std::string entityName;
		math::Vector3 start, end;
		start.z = msg->height();
		end.z = 0.001;

		gazebo::physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
		engine->InitForThread();
		gazebo::physics::RayShapePtr ray = boost::shared_dynamic_cast<
				gazebo::physics::RayShape>(
				engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

		ROS_INFO_STREAM( "Rasterizing model and checking collisions" );

		nav_msgs::OccupancyGrid map;
		map.info.height = count_horizontal;
		map.info.width = count_vertical;
		map.info.resolution = msg->resolution();
		map.info.origin.position.x = msg->lowerleft().x();
		map.info.origin.position.y = msg->lowerleft().y();
		map.info.origin.orientation.w = 1;
		map.data.resize(map.info.height * map.info.width,0);

		ROS_INFO("count_horizontal = %d",count_horizontal);
		ROS_INFO("count_vertical = %d",count_vertical);

		for (int i = 0; i < count_vertical; ++i) {
			ROS_DEBUG_STREAM( "Percent complete: " << i * 100.0 / count_vertical);
			x = i * dX_vertical + msg->lowerleft().x();
			y = i * dY_vertical + msg->lowerleft().y();
			for (int j = 0; j < count_horizontal; ++j) {
				unsigned map_idx = unsigned(j) + unsigned(i) * map.info.width;
				x += dX_horizontal;
				y += dY_horizontal;

				start.x = end.x = x;
				start.y = end.y = y;
				ray->SetPoints(start, end);
				ray->GetIntersection(dist, entityName);
				if (!entityName.empty()) {
					map.data.at(map_idx)=100;
				}

			}
		}

		// unsigned map_idx = unsigned(j) + (map.info.width - unsigned(i) - 1) * map.info.height; // mirrored width
		// unsigned map_idx = unsigned(i) + (map.info.height - unsigned(j) - 1) * map.info.width; //?

		/* ORIGINAL
		 * for (int i = 0; i < count_vertical; ++i) {
			ROS_INFO_STREAM( "Percent complete: " << i * 100.0 / count_vertical);
			x = i * dX_vertical + msg->lowerleft().x();
			y = i * dY_vertical + msg->lowerleft().y();
			for (int j = 0; j < count_horizontal; ++j) {
				unsigned map_idx = unsigned(i) + (map.info.height - unsigned(j) - 1) * map.info.width;
				x += dX_horizontal;
				y += dY_horizontal;

				start.x = end.x = x;
				start.y = end.y = y;
				ray->SetPoints(start, end);
				ray->GetIntersection(dist, entityName);
				if (!entityName.empty()) {
					map.data.at(map_idx)=100;
				}

			}
		}
		 */

		ROS_INFO("Completed calculations, writing to image");
		write(&map);
	}
public:
	// write is based on a copy of mapCallback from map_server/src/map_server.cpp
	void write(const nav_msgs::OccupancyGrid* map) {
		ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width,
				map->info.height, map->info.resolution);

		std::string mapdatafile = filename_noext + ".pgm";
		ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
		FILE* out = fopen(mapdatafile.c_str(), "w");
		if (!out) {
			ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
			return;
		}

		fprintf(out,
				"P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
				map->info.resolution, map->info.width, map->info.height);
		for (unsigned int y = 0; y < map->info.height; y++) {
			for (unsigned int x = 0; x < map->info.width; x++) {
				unsigned int i = x
						+ (map->info.height - y - 1) * map->info.width;
				if (map->data[i] == 0) { //occ [0,0.1)
					fputc(254, out);
				} else if (map->data[i] == +100) { //occ (0.65,1]
					fputc(000, out);
				} else { //occ [0.1,0.65]
					fputc(205, out);
				}
			}
		}

		fclose(out);

		std::string mapmetadatafile = filename_noext + ".yaml";
		ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
		FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


		// resolution: 0.100000
		// origin: [0.000000, 0.000000, 0.000000]
		// #
		// negate: 0
		// occupied_thresh: 0.65
		// free_thresh: 0.196


		geometry_msgs::Quaternion orientation = map->info.origin.orientation;
		tf::Matrix3x3 mat(
				tf::Quaternion(orientation.x, orientation.y, orientation.z,
						orientation.w));
		double yaw, pitch, roll;
		mat.getEulerYPR(yaw, pitch, roll);

		fprintf(yaml,
				"image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
				(map_name + ".pgm").c_str(), map->info.resolution,
				map->info.origin.position.x, map->info.origin.position.y, yaw);

		fclose(yaml);

		ROS_INFO("Done\n");
	}

};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CollisionMapCreator)
}
