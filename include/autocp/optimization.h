#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneManager.h>

#include "models/score.h"
#include "models/viewpoint.h"
#include "autocp_sensing.h"
#include "visibility.h"
#include "visualization.h"

namespace autocp {

static const float kR2 = 0.70710678118; // sqrt(2) / 2

class Optimization {
 public:
  Optimization(AutoCPSensing* sensing, VisibilityChecker* visibility_checker,
               Ogre::Camera* camera);
  ~Optimization();
  void ChooseViewpoint(const Viewpoint& current_viewpoint,
                       Viewpoint* target_viewpoint);
  void set_visibility_weight(float weight);
  void set_centering_weight(float weight);
  void set_view_angle_weight(float weight);
  void set_zoom_weight(float weight);
  void set_max_visibility_checks(int max_visibility_checks);
  void set_score_threshold(float threshold);
  void set_min_zoom(float min_zoom);
  void set_max_zoom(float max_zoom);
  void set_max_travel(float max_travel);

 private:
  AutoCPSensing* sensing_;
  Ogre::Camera* camera_;
  Visualization* visualization_;
  VisibilityChecker* visibility_checker_;
  std::vector<Ogre::Vector3> standard_offsets_;
  int offset_index_;

  // Property weights.
  float visibility_weight_;
  float centering_weight_;
  float view_angle_weight_;
  float zoom_weight_;
  float min_zoom_;
  float max_zoom_;
  float max_travel_;

  int max_visibility_checks_;
  float score_threshold_;

  void InitializeStandardOffsets();
  void SelectViewpoints(std::vector<Viewpoint>* viewpoints);
  void ComputeViewpointScore(const Viewpoint& viewpoint, Score* score);

  // Score functions.
  float VisibilityScore(const Viewpoint& viewpoint);
  float CenteringScore(const Viewpoint& viewpoint);
  float ViewAngleScore(const Viewpoint& viewpoint,
                       const ClickedControl& control);
  float ZoomScore(const Viewpoint& viewpoint);
};

}

#endif
