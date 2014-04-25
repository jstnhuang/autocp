#ifndef _UTILS_H_
#define _UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <math.h>
#include "models/clicked_control.h"
#include "models/viewpoint.h"

namespace autocp {
void ComputeControlProjection(const ClickedControl& control,
                              const Ogre::Vector3& vector,
                              Ogre::Vector3* projection);
void interpolateViewpoint(const Viewpoint& start, const Viewpoint& end,
                          float position_speed, float time_delta,
                          Viewpoint* result);
void interpolatePoint(const Ogre::Vector3& start, const Ogre::Vector3& end,
                      float speed, float time_delta, Ogre::Vector3* result);
float linearInterpolation(float x1, float y1, float x2, float y2, float x);
void QuaternionToFocus(
    const geometry_msgs::Quaternion& quaternion,
    const geometry_msgs::Point& position, geometry_msgs::Point* point);
int sign(float x);
geometry_msgs::Point ToGeometryMsgsPoint(const Ogre::Vector3& vector3);
geometry_msgs::Quaternion ToGeometryMsgsQuaternion(
    const Ogre::Quaternion& quaternion);
Ogre::Quaternion ToOgreQuaternion(const geometry_msgs::Quaternion& quaternion);
Ogre::Vector3 ToOgreVector3(const geometry_msgs::Point& point);
void ViewpointToOrientation(const Viewpoint& viewpoint,
                            Ogre::Quaternion* orientation);
}

#endif
