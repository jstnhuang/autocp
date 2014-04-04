#ifndef _UTILS_H_
#define _UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <OGRE/OgreVector3.h>
#include <math.h>
#include "viewpoint.h"

namespace autocp {
void interpolateViewpoint(const Viewpoint& start, const Viewpoint& end,
                          float position_speed, float focus_speed,
                          float time_delta, Viewpoint* result);
void interpolatePoint(const Ogre::Vector3& start, const Ogre::Vector3& end,
                      float speed, float time_delta, Ogre::Vector3* result);
float linearInterpolation(float x1, float y1, float x2, float y2, float x);
int sign(float x);
void ViewpointToOrientation(const Viewpoint& viewpoint,
                            Ogre::Quaternion* orientation);
}

#endif
