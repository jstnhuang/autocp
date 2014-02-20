#include "landmarks.h"
#include "utils.h"

namespace autocp {
using namespace geometry_msgs;

Landmarks::Landmarks()
  : l_gripper_(),
    r_gripper_(),
    head_focus_(),
    current_marker_(),
    segmented_objects_(),
    gripper_weight_(0),
    segmented_object_weight_(0) {
}

Landmarks::~Landmarks() {
}

/*
 * Returns the weighted center of all the landmarks.
 */
Point Landmarks::Center() {
  Point center;
  std::vector<Landmark> landmarks = landmarksVector();
  float normalizer = 0;
  for (const auto& landmark : landmarks) {
    if (landmark.exists) {
      normalizer += landmark.weight;
      Point weighted = scale(landmark.position, landmark.weight);
      center = add(center, weighted);
    }
  }
  center = scale(center, 1 / normalizer);
  return center;
}

/*
 * Returns all the existing landmarks as a vector.
 */
std::vector<Landmark> Landmarks::landmarksVector() {
  std::vector<Landmark> points;
  if (l_gripper_.exists) {
    points.push_back(l_gripper_);
  }
  if (r_gripper_.exists) {
    points.push_back(r_gripper_);
  }
  if (head_focus_.exists) {
    points.push_back(head_focus_);
  }
  if (current_marker_.exists) {
    points.push_back(current_marker_);
  }
  // If the segmented objects vector is nonempty, the objects are assumed to
  // exist.
  points.insert(points.end(), segmented_objects_.begin(),
    segmented_objects_.end());
  return points;
}

/*
 * Updates the position of the left gripper. A NULL input pointer represents a
 * point that doesn't exist.
 */
void Landmarks::UpdateLeftGripper(const Point* point) {
  if (point != NULL) {
    l_gripper_.position = *point;
    l_gripper_.exists = true;
  } else {
    l_gripper_.exists = false;
  }
}

/*
 * Updates the position of the right gripper. A NULL input pointer represents a
 * point that doesn't exist.
 */
void Landmarks::UpdateRightGripper(const Point* point) {
  if (point != NULL) {
    r_gripper_.position = *point;
    r_gripper_.exists = true;
  } else {
    r_gripper_.exists = false;
  }
}

/*
 * Updates the position of the head focus. A NULL input pointer represents a
 * point that doesn't exist.
 */
void Landmarks::UpdateHeadFocus(const Point* point) {
  if (point != NULL) {
    head_focus_.position = *point;
    head_focus_.exists = true;
  } else {
    head_focus_.exists = false;
  }
}

/*
 * Updates the position of the current marker being interacted with. A NULL
 * input pointer represents a point that doesn't exist.
 */
void Landmarks::UpdateCurrentMarker(const Point* point) {
  if (point != NULL) {
    current_marker_.position = *point;
    current_marker_.exists = true;
  } else {
    current_marker_.exists = false;
  }
}

/*
 * Updates the position of the segmented objects. An empty vector can be used to
 * represent the fact that there are no segmented objects.
 */
void Landmarks::UpdateSegmentedObjects(const std::vector<Point>& objects) {
  float weight = segmented_object_weight_ / segmented_objects_.size();
  segmented_objects_.clear();
  for (const auto& object : objects) {
    segmented_objects_.push_back(Landmark(object, weight, true));
  }
}

/*
 * Updates the weights of the grippers. The provided weight is for both
 * grippers. Each gripper actually gets half the weight.
 */
void Landmarks::UpdateGripperWeight(float weight) {
  gripper_weight_ = weight;
  l_gripper_.weight = gripper_weight_ / 2;
  r_gripper_.weight = gripper_weight_ / 2;
}

/*
 * Updates the weight of the head focus point.
 */
void Landmarks::UpdateHeadFocusWeight(float weight) {
  head_focus_.weight = weight;
}

/*
 * Updates the weight of the marker that is currently being interactived with.
 */
void Landmarks::UpdateCurrentMarkerWeight(float weight) {
  current_marker_.weight = weight;
}

/*
 * Updates the weight of the segmented objects. The provided weight is for all
 * the segmented objects. Each object ends up getting an equal share of the
 * weight.
 */
void Landmarks::UpdateSegmentedObjectWeight(float weight) {
  segmented_object_weight_ = weight;
  float updated_weight = segmented_object_weight_ / segmented_objects_.size();
  for (auto& object : segmented_objects_) {
    object.weight = updated_weight;
  }
}
}
