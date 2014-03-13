#ifndef _UTILS_H_
#define _UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <OGRE/OgreVector3.h>
#include <math.h>

namespace autocp {
geometry_msgs::Point add(const geometry_msgs::Point& p,
                         const geometry_msgs::Vector3& v);
geometry_msgs::Point add(const geometry_msgs::Point& a,
                         const geometry_msgs::Point& b);
geometry_msgs::Vector3 copyVector(const geometry_msgs::Vector3& v);
float cosineAngle(const geometry_msgs::Vector3& v1,
                  const geometry_msgs::Vector3& v2);
float distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
float dotProduct(const geometry_msgs::Vector3& v1,
                 const geometry_msgs::Vector3& v2);
float length(const geometry_msgs::Vector3& v);
float logisticDistance(float x, float scale);
geometry_msgs::Vector3 makeVector3(float x, float y, float z);
float minimumMagnitude(float num, float magnitude);
geometry_msgs::Point scale(const geometry_msgs::Point& point, float scale);
geometry_msgs::Vector3 scale(const geometry_msgs::Vector3& vector, float scale);
geometry_msgs::Vector3 setLength(const geometry_msgs::Vector3& p1,
                                 float new_length);
int sign(float x);
geometry_msgs::Point toPoint(const Ogre::Vector3& v);
geometry_msgs::Point toPoint(const geometry_msgs::Vector3& v);
geometry_msgs::Vector3 toVector3(const geometry_msgs::Point & p);
geometry_msgs::Vector3 vectorBetween(const geometry_msgs::Point& from,
                                     const geometry_msgs::Point& to);
}

#endif
