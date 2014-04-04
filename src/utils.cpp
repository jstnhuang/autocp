#include "models/viewpoint.h"

namespace autocp {
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
