#include "utils.h"

namespace autocp {
/**
 * Computes the point when adding the given vector to the given point.
 */
geometry_msgs::Point add(
    const geometry_msgs::Point& p,
    const geometry_msgs::Vector3& v) {
  geometry_msgs::Point result;
  result.x = p.x + v.x;
  result.y = p.y + v.y;
  result.z = p.z + v.z;
  return result;
}

/**
 * Computes the result of adding two points.
 */
geometry_msgs::Point add(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b) {
  geometry_msgs::Point result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}

/**
 * Copies v (since Vector3 doesn't have a copy constructor).
 */
geometry_msgs::Vector3 copyVector(const geometry_msgs::Vector3& v) {
  geometry_msgs::Vector3 result;
  result.x = v.x;
  result.y = v.y;
  result.z = v.z;
  return result;
}

/**
 * Returns the cosine of the angle between the two vectors.
 */
float cosineAngle(
    const geometry_msgs::Vector3& v1,
    const geometry_msgs::Vector3& v2) {
  return dotProduct(v1, v2) / (length(v1) * length(v2));
}

float distance(
    const geometry_msgs::Point& p1,
    const geometry_msgs::Point& p2) {
  geometry_msgs::Vector3 difference;
  difference.x = p1.x - p2.x; difference.y = p1.y - p2.y;
  difference.z = p1.z - p2.z;
  return length(difference);
}

/**
 * Returns the dot product of two vectors.
 */
float dotProduct(
    const geometry_msgs::Vector3& v1,
    const geometry_msgs::Vector3& v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

/**
 * Returns the length of the vector.
 */
float length(const geometry_msgs::Vector3& v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * A function that squeezes an unbounded distance x into a (0, 1) value. Scale
 * determines the width of the function. To choose the scale, you want to have
 * an idea of what value of x should produce a result close to 1. If you use
 * 0.99 as an approximation of 1, then scale = 5.2933 / x.
 */
float logisticDistance(float x, float scale) {
  if (scale * x > 6 || scale * x < -6) {
    return 1;
  }
  return fabs(2 / (1 + exp(scale * -x)) - 1);
}

/**
 * Convenience method to create a Vector3, since Vector3 doesn't provide a
 * constructor.
 */
geometry_msgs::Vector3 makeVector3(float x, float y, float z) {
  geometry_msgs::Vector3 result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

/**
 * Like max, but handles negative numbers, e.g.,
 *   minimumMagnitude(-1, 2) = -2
 *   minimumMagnitude(1, 2) = 2
 *   minimumMagnitude(3, 2) = 3
 *   minimumMagnitude(-3, 2) = -3
 */
float minimumMagnitude(float num, float magnitude) {
  if (fabs(num) < magnitude) {
    if (num < 0) {
      num = -magnitude;
    } else {
      num = magnitude;
    }
  }
  return num;
}

/*
 * Returns the scaled point.
 */
geometry_msgs::Point scale(const geometry_msgs::Point& point, float scale) {
  geometry_msgs::Point result;
  result.x = point.x * scale;
  result.y = point.y * scale;
  result.z = point.z * scale;
  return result;
}

/*
 * Returns the scaled vector.
 */
geometry_msgs::Vector3 scale(
  const geometry_msgs::Vector3& vector, float scale) {
  geometry_msgs::Vector3 result;
  result.x = vector.x * scale;
  result.y = vector.y * scale;
  result.z = vector.z * scale;
  return result;
}

/**
 * Returns a new vector that's the same as v, but scaled to the given length.
 */
geometry_msgs::Vector3 setLength(
    const geometry_msgs::Vector3& v, float new_length) {
  geometry_msgs::Vector3 result = copyVector(v);
  float constant = new_length / length(result);
  result.x *= constant;
  result.y *= constant;
  result.z *= constant;
  return result;
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
 * Convert an Ogre Vector3 to a Point.
 */
geometry_msgs::Point toPoint(const Ogre::Vector3& v) {
  geometry_msgs::Point point;
  point.x = v.x;
  point.y = v.y;
  point.z = v.z;
  return point;
}

/**
 * Convert a ROS Vector3 to a Point.
 */
geometry_msgs::Point toPoint(const geometry_msgs::Vector3& v) {
  geometry_msgs::Point point;
  point.x = v.x;
  point.y = v.y;
  point.z = v.z;
  return point;
}

/**
 * Convert an Ogre Vector3 to a Point.
 */
geometry_msgs::Vector3 toVector3(const geometry_msgs::Point& p) {
  geometry_msgs::Vector3 vector;
  vector.x = p.x;
  vector.y = p.y;
  vector.z = p.z;
  return vector;
}

/**
 * Get the vector starting at "from" and ending at "to".
 */
geometry_msgs::Vector3 vectorBetween(
    const geometry_msgs::Point& from,
    const geometry_msgs::Point& to) {
  geometry_msgs::Vector3 result;
  result.x = to.x - from.x;
  result.y = to.y - from.y;
  result.z = to.z - from.z;
  return result;
}
}
