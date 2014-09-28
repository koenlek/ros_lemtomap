#ifndef UTILS_H
#define UTILS_H

/*
 * @file utils
 * @brief Contains some utilities/functions that are commonly used by the lemto_topological_mapping package
 * @author Koen Lekkerkerker
 */

#include <math.h>
#include "lemto_topological_mapping/toponav_node.h"
#include "tf/transform_datatypes.h"

/*
 * calcDistance can calculate the distance between two points. It will take several different kinds of inputs...
 */
static inline double calcDistance(const double &x1, const double &y1, const double &x2, const double &y2)
{
  return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

static inline double calcDistance(const tf::Pose &p1, const tf::Pose &p2)
{
  return calcDistance(p1.getOrigin().getX(),p1.getOrigin().getY(),p2.getOrigin().getX(),p2.getOrigin().getY());
}

static inline double calcDistance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
{
  return calcDistance(p1.position.x, p1.position.y, p2.position.x, p2.position.y);
}

static inline double calcDistance(const geometry_msgs::Pose &p1, const tf::Pose &p2)
{
  return calcDistance(p1.position.x, p1.position.y, p2.getOrigin().getX(), p2.getOrigin().getY());
}

static inline double calcDistance(const TopoNavNode &n1, const TopoNavNode &n2)
{
  return calcDistance(n1.getPose(),n2.getPose());
}

static inline double calcDistance(const tf::Point &p1, const tf::Point &p2)
{
  return calcDistance(p1.getX(),p1.getY(),p2.getX(),p2.getY());
}

#endif // UTILS_H

