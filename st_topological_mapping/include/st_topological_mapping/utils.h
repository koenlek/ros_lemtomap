#ifndef UTILS_H
#define UTILS_H

/*
 * @file utils
 * @brief Contains some utilities/functions that are commonly used by the st_topological_mapping package
 * @author Koen Lekkerkerker
 */

#include <math.h>
#include "toponav_node.h"

double calcDistance(const double &x1, double &y1, double &x2, double &y2); // Calculate Euclidean Distance between two points.
double calcDistance(const TopoNavNode &n1, const tf::Pose &p1);
double calcDistance(const tf::Pose &p1, const TopoNavNode &n1);
double calcDistance(const TopoNavNode &n1, const TopoNavNode &n2);
double calcDistance(const tf::Pose &p1, const tf::Pose &p2);

#endif // UTILS_H

