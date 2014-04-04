#include "landmarks.h"
#include "utils.h"

namespace autocp {
using namespace geometry_msgs;

Landmarks::Landmarks()
    : l_gripper_(),
      r_gripper_(),
      head_(),
      head_focus_(),
      segmented_objects_(),
      gripper_weight_(0),
      segmented_object_weight_(0),
      num_landmarks_(0) {
}

Landmarks::~Landmarks() {
}

/*
 * Returns the weighted center of all the landmarks.
 */
 Ogre::Vector3 Landmarks::Center() {
  Ogre::Vector3 center;
  std::vector<Landmark> landmarks;
  LandmarksVector(&landmarks);
  float normalizer = 0;
  for (const auto& landmark : landmarks) {
    if (landmark.exists && landmark.weight > 0) {
      normalizer += landmark.weight;
      center += landmark.weight * landmark.position;
    }
  }
  center /= normalizer;
  return center;
}

/*
 * Returns all the existing landmarks as a vector.
 */
void Landmarks::LandmarksVector(std::vector<Landmark>* landmarks) {
  if (l_gripper_.exists && l_gripper_.weight > 0) {
    landmarks->push_back(l_gripper_);
  }
  if (r_gripper_.exists && r_gripper_.weight > 0) {
    landmarks->push_back(r_gripper_);
  }
  if (head_.exists && head_.weight > 0) {
    landmarks->push_back(head_);
  }
  if (head_focus_.exists && head_focus_.weight > 0) {
    landmarks->push_back(head_focus_);
  }
  // If the segmented objects vector is nonempty, the objects are assumed to
  // exist.
  landmarks->insert(landmarks->end(), segmented_objects_.begin(),
                    segmented_objects_.end());
}

/*
 * Updates the position of the left gripper. A NULL input pointer represents a
 * point that doesn't exist.
 */
void Landmarks::UpdateLeftGripper(const Ogre::Vector3* point) {
  if (point != NULL) {
    l_gripper_.position = *point;
    if (!l_gripper_.exists) {
      num_landmarks_++;
    }
    l_gripper_.exists = true;
  } else {
    if (l_gripper_.exists) {
      num_landmarks_--;
    }
    l_gripper_.exists = false;
  }
  UpdateGripperWeight(gripper_weight_);
}

/*
 * Updates the position of the right gripper. A NULL input pointer represents a
 * point that doesn't exist.
 */
void Landmarks::UpdateRightGripper(const Ogre::Vector3* point) {
  if (point != NULL) {
    r_gripper_.position = *point;
    if (!r_gripper_.exists) {
      num_landmarks_++;
    }
    r_gripper_.exists = true;
  } else {
    if (r_gripper_.exists) {
      num_landmarks_--;
    }
    r_gripper_.exists = false;
  }
  UpdateGripperWeight(gripper_weight_);
}

void Landmarks::UpdateHead(const Ogre::Vector3* point) {
  if (point != NULL) {
    head_.position = *point;
    if (!head_.exists) {
      num_landmarks_++;
    }
    head_.exists = true;
  } else {
    if (head_.exists) {
      num_landmarks_--;
    }
    head_.exists = false;
  }
}

/*
 * Updates the position of the head focus. A NULL input pointer represents a
 * point that doesn't exist.
 */
void Landmarks::UpdateHeadFocus(const Ogre::Vector3* point) {
  if (point != NULL) {
    head_focus_.position = *point;
    if (!head_focus_.exists) {
      num_landmarks_++;
    }
    head_focus_.exists = true;
  } else {
    if (head_focus_.exists) {
      num_landmarks_--;
    }
    head_focus_.exists = false;
  }
}

/*
 * Updates the position of the segmented objects. An empty vector can be used to
 * represent the fact that there are no segmented objects.
 */
void Landmarks::UpdateSegmentedObjects(
    const std::vector<Ogre::Vector3>& objects) {
  float weight = segmented_object_weight_ / objects.size();
  num_landmarks_ -= segmented_objects_.size();
  num_landmarks_ += objects.size();
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
  // If one gripper doesn't exist, then the other gets all the weight.
  // Otherwise, they share the weight equally.
  if (!l_gripper_.exists || !r_gripper_.exists) {
    l_gripper_.weight = gripper_weight_;
    r_gripper_.weight = gripper_weight_;
  } else {
    l_gripper_.weight = gripper_weight_ / 2;
    r_gripper_.weight = gripper_weight_ / 2;
  }
}

/*
 * Updates the weight of the head focus point.
 */
void Landmarks::UpdateHeadFocusWeight(float weight) {
  head_focus_.weight = weight;
}

/*
 * Updates the weight of the head.
 */
void Landmarks::UpdateHeadWeight(float weight) {
  head_.weight = weight;
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

float Landmarks::GripperWeight() {
  return gripper_weight_;
}

float Landmarks::HeadWeight() {
  return head_.weight;
}

float Landmarks::HeadFocusWeight() {
  return head_focus_.weight;
}

float Landmarks::SegmentedObjectWeight() {
  return segmented_object_weight_;
}

int Landmarks::NumLandmarks() {
  return num_landmarks_;
}
}
