#ifndef AUTOCP_LANDMARKS_H
#define AUTOCP_LANDMARKS_H

#include <functional>
#include <vector>
#include <OGRE/OgreVector3.h>

namespace autocp {

struct Landmark {
  Ogre::Vector3 position;
  float weight;
  bool exists;
  Landmark()
      : position(),
        weight(0),
        exists(false) {
  }
  Landmark(Ogre::Vector3 position, float weight, bool exists)
      : position(position),
        weight(weight),
        exists(exists) {
  }
};

/*
 * Represents the set of landmarks for automatic camera placement. A landmark is
 * a position that also has a weight attached to it. The weights will
 * automatically be normalized such that the sum of weights for all existing
 * landmarks add is 1.
 */
class Landmarks {
 public:
  Landmarks();
  ~Landmarks();
  Ogre::Vector3 Center();
  void LandmarksVector(std::vector<Landmark>* landmarks);
  template<typename MetricFunc>
  float ComputeMetric(MetricFunc metric);
  void UpdateLeftGripper(const Ogre::Vector3* point);
  void UpdateRightGripper(const Ogre::Vector3* point);
  void UpdateHead(const Ogre::Vector3* point);
  void UpdateHeadFocus(const Ogre::Vector3* point);
  void UpdateSegmentedObjects(const std::vector<Ogre::Vector3>& objects);
  void UpdateLeftGripperWeight(float weight);
  void UpdateRightGripperWeight(float weight);
  void UpdateHeadWeight(float weight);
  void UpdateHeadFocusWeight(float weight);
  void UpdateSegmentedObjectWeight(float weight);
  float LeftGripperWeight();
  float RightGripperWeight();
  float HeadWeight();
  float HeadFocusWeight();
  float SegmentedObjectWeight();
  int NumLandmarks();
 private:
  Landmark l_gripper_;
  Landmark r_gripper_;
  Landmark head_;
  Landmark head_focus_;
  std::vector<Landmark> segmented_objects_;
  // We save weights that are distributed between multiple objects. Weights
  // for single objects are just stored in the Landmark data structure.
  float segmented_object_weight_;
};

template<typename MetricFunc>
float Landmarks::ComputeMetric(MetricFunc metric) {
  float result = 0;
  float normalizer = 0;
  std::vector<Landmark> landmarks;
  LandmarksVector(&landmarks);
  for (const auto& landmark : landmarks) {
    if (landmark.exists && landmark.weight > 0) {
      normalizer += landmark.weight;
      result += landmark.weight * metric(landmark.position);
    }
  }
  // TODO: we currently assume normalizer is never 0.
  result /= normalizer;
  return result;
}

}

#endif
