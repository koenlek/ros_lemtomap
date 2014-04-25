/**
 * @file utils
 * @brief This file specifies some common functions to be used for the Topological Navigation Mapping.
 * @author Koen Lekkerkerker
 */

#include <st_topological_mapping/utils.h>


/*
 * calcDistance can calculate the distance between two points. It will take several different kinds of inputs...
 */
double calcDistance(const double &x1, const double &y1, const double &x2, const double &y2)
{
  return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

double calcDistance(const TopoNavNode &n1, const tf::Pose &p1)
{
  return calcDistance(n1.getPose(),p1);
}

double calcDistance(const tf::Pose &p1, const TopoNavNode &n1)
{
  return calcDistance(p1,n1.getPose());
}

double calcDistance(const TopoNavNode &n1, const TopoNavNode &n2)
{
  return calcDistance(n1.getPose(),n2.getPose());
}

double calcDistance(const tf::Pose &p1, const tf::Pose &p2)
{
  return calcDistance(p1.getOrigin().getX(),p1.getOrigin().getY(),p2.getOrigin().getX(),p2.getOrigin().getY());
}
