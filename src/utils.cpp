
#include <OGRE/OgreVector3.h>
#include <ros/ros.h>

#include "autocp/utils.h"
#include "autocp/models/clicked_control.h"
#include "autocp/models/control_6dof.h"
#include "autocp/models/viewpoint.h"

namespace autocp {
/**
 * Compute the projection of the vector onto the orthogonal plane or line
 * defined by the current control.
 */
void ComputeControlProjection(
    const ClickedControl& control,
    const Ogre::Vector3& vector,
    Ogre::Vector3* projection) {
  projection->x = vector.x;
  projection->y = vector.y;
  projection->z = vector.z;
  if (control.control == Control6Dof::X) {
    projection->x = 0;
  } else if (control.control == Control6Dof::Y) {
    projection->y = 0;
  } else if (control.control == Control6Dof::Z) {
    projection->z = 0;
  } else if (control.control == Control6Dof::PITCH) {
    projection->x = 0;
    projection->z = 0;
  } else if (control.control == Control6Dof::ROLL) {
    projection->y = 0;
    projection->z = 0;
  } else if (control.control == Control6Dof::YAW) {
    projection->x = 0;
    projection->y = 0;
  } else {
    ROS_ERROR("Tried to compute orthogonal projection of an unknown control.");
  }
}

/**
 * Interpolates between the start and end positions, subject to the camera speed
 * and size of this time step.
 */
void interpolatePoint(const Ogre::Vector3& start,
                      const Ogre::Vector3& end,
                      float speed, float time_delta,
                      Ogre::Vector3* result) {
  float step_distance = speed * time_delta;
  if (step_distance > start.distance(end)) {
    *result = end;
  } else {
    Ogre::Vector3 step = (end - start);
    step.normalise();
    step *= step_distance;
    *result = start + step;
  }
}

void interpolateViewpoint(const Viewpoint& start,
                          const Viewpoint& end,
                          float position_speed,
                          float focus_speed,
                          float time_delta,
                          Viewpoint* result) {
  interpolatePoint(start.position, end.position, position_speed, time_delta,
                   &(result->position));
  interpolatePoint(start.focus, end.focus, focus_speed, time_delta,
                   &(result->focus));
}

/**
 * Returns the y value on the line segment between (x1, y1) and (x2, y2) at the
 * given x position. If x < x1, then it returns y1, and if x > x2, it returns
 * y2.
 */
float linearInterpolation(float x1, float y1, float x2, float y2, float x) {
  if (x < x1) {
    return y1;
  } else if (x > x2) {
    return y2;
  } else {
    float slope = (y2 - y1) / (x2 - x1);
    return y1 + slope * (x - x1);
  }
}

/**
 * Returns the sign of x.
 */
int sign(float x) {
  if (x < 0) {
    return -1;
  } else if (x > 0) {
    return 1;
  } else {
    return 0;
  }
}

/**
 * Converts a position/focus point into an orientation from the position.
 */
void ViewpointToOrientation(const Viewpoint& viewpoint,
                            Ogre::Quaternion* orientation) {
  Ogre::Vector3 diff = viewpoint.focus - viewpoint.position;
  auto ogre_orientation = Ogre::Vector3::UNIT_X.getRotationTo(diff);
  *orientation = Ogre::Vector3::UNIT_X.getRotationTo(diff);
}
}
