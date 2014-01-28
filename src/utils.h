#ifndef _UTILS_H_
#define _UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <OGRE/OgreVector3.h>
#include <math.h>

namespace autocp {
geometry_msgs::Point add(
  const geometry_msgs::Point& p,
  const geometry_msgs::Vector3& v);
geometry_msgs::Vector3 copyVector(const geometry_msgs::Vector3& v);
float cosineAngle(
  const geometry_msgs::Vector3& v1,
  const geometry_msgs::Vector3& v2);
float distance(
  const geometry_msgs::Point& p1,
  const geometry_msgs::Point& p2);
float dotProduct(
  const geometry_msgs::Vector3& v1,
  const geometry_msgs::Vector3& v2);
float length(const geometry_msgs::Vector3& v);
float logisticDistance(float x);
float minimumMagnitude(float num, float magnitude);
geometry_msgs::Vector3 setLength(
  const geometry_msgs::Vector3& p1,
  float new_length);
geometry_msgs::Point toPoint(const Ogre::Vector3& v);
geometry_msgs::Vector3 vectorBetween(
  const geometry_msgs::Point& p1,
  const geometry_msgs::Point& p2);
}

#endif
