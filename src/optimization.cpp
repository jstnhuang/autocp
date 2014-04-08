#include "optimization.h"

#include <algorithm>

#include <OGRE/OgreVector3.h>
#include <ros/ros.h>

#include "utils.h"

namespace autocp {

Optimization::Optimization(AutoCPSensing* sensing,
                           Ogre::SceneManager* scene_manager,
                           Ogre::Camera* camera,
                           Visualization* visualization)
    : standard_offsets_(),
      visibility_weight_(0),
      view_angle_weight_(0),
      zoom_weight_(0),
      travel_weight_(0),
      crossing_weight_(0),
      max_visibility_checks_(1000),
      only_move_on_idle_(false),
      score_threshold_(1.05),
      min_zoom_(0.5),
      max_zoom_(5),
      max_travel_(1) {
  sensing_ = sensing;
  camera_ = camera;
  visualization_ = visualization;
  visibility_checker_ = new VisibilityChecker(scene_manager, camera);
  InitializeStandardOffsets();
}

Optimization::~Optimization() {
}

void Optimization::ChooseViewpoint(const Viewpoint& current_viewpoint,
                                   Viewpoint* target_viewpoint) {
  std::vector<Viewpoint> test_viewpoints;
  std::vector<Score> scores;

  // Assume that the current position is the best, to start with.
  //Score current_score;
  //ComputeViewpointScore(current_viewpoint, current_viewpoint, &current_score);
  //Viewpoint best_viewpoint = current_viewpoint;
  //Score best_score = current_score;
  //test_viewpoints.push_back(current_viewpoint);
  //scores.push_back(current_score);

  // The current viewpoint doesn't need to overcome the score threshold.
  //*target_viewpoint = current_viewpoint;

  Score target_score;
  ComputeViewpointScore(*target_viewpoint, current_viewpoint, &target_score);
  Score best_score = target_score;
  Viewpoint best_viewpoint;
  test_viewpoints.push_back(*target_viewpoint);
  scores.push_back(target_score);
//  if (target_score.score > best_score.score) {
    best_score = target_score;
    best_viewpoint = *target_viewpoint;
 // }

  std::vector<Viewpoint> viewpoints;
  SelectViewpoints(&viewpoints);

  for (const auto& viewpoint : viewpoints) {
    Score score;
    ComputeViewpointScore(viewpoint, current_viewpoint, &score);
    test_viewpoints.push_back(viewpoint);
    scores.push_back(score);

    if (score.score > best_score.score) {
      best_viewpoint = viewpoint;
      best_score = score;
    }
  }

//  if (best_score.score > score_threshold_ * current_score.score) {
  if (best_score.score > score_threshold_ * target_score.score) {
    visualization_->ShowViewpoints(test_viewpoints, scores);
    auto best_position = best_viewpoint.position;
    ROS_INFO("Moving to (%.2f, %.2f, %.2f), score=%s, prev=(%.2f, %.2f, %.2f)=%.2f",
             best_position.x, best_position.y, best_position.z,
             best_score.toString().c_str(), target_viewpoint->position.x,
             target_viewpoint->position.y, target_viewpoint->position.z,
             target_score.score);
    *target_viewpoint = best_viewpoint;
  } else {
    auto target_position = target_viewpoint->position;
    ROS_INFO("Continuing to (%.2f, %.2f, %.2f), score=%s",
             target_position.x, target_position.y, target_position.z,
             target_score.toString().c_str());
  }
}

void Optimization::set_visibility_weight(float weight) {
  visibility_weight_ = weight;
}

void Optimization::set_view_angle_weight(float weight) {
  view_angle_weight_ = weight;
}

void Optimization::set_zoom_weight(float weight) {
  zoom_weight_ = weight;
}

void Optimization::set_travel_weight(float weight) {
  travel_weight_ = weight;
}

void Optimization::set_crossing_weight(float weight) {
  crossing_weight_ = weight;
}

void Optimization::set_max_visibility_checks(int max_visibility_checks) {
  max_visibility_checks_ = max_visibility_checks;
}

void Optimization::set_only_move_on_idle(bool only_move_on_idle) {
  only_move_on_idle_ = only_move_on_idle;
}

void Optimization::set_score_threshold(float threshold) {
  score_threshold_ = threshold;
}

void Optimization::set_min_zoom(float min_zoom) {
  min_zoom_ = min_zoom;
}

void Optimization::set_max_zoom(float max_zoom) {
  max_zoom_ = max_zoom;
}

void Optimization::set_max_travel(float max_travel) {
  max_travel_ = max_travel;
}

void Optimization::InitializeStandardOffsets() {
  // Lower plane.
  standard_offsets_.push_back(Ogre::Vector3(1, 0, 0));
  standard_offsets_.push_back(Ogre::Vector3(kR2, kR2, 0));
  standard_offsets_.push_back(Ogre::Vector3(0, 1, 0));
  standard_offsets_.push_back(Ogre::Vector3(-kR2, kR2, 0));
  standard_offsets_.push_back(Ogre::Vector3(-1, 0, 0));
  standard_offsets_.push_back(Ogre::Vector3(-kR2, -kR2, 0));
  standard_offsets_.push_back(Ogre::Vector3(0, -1, 0));
  standard_offsets_.push_back(Ogre::Vector3(kR2, -kR2, 0));
  // 45 degrees up.
  standard_offsets_.push_back(Ogre::Vector3(kR2, 0, kR2));
  standard_offsets_.push_back(Ogre::Vector3(0.5, 0.5, kR2));
  standard_offsets_.push_back(Ogre::Vector3(0, kR2, kR2));
  standard_offsets_.push_back(Ogre::Vector3(-0.5, 0.5, kR2));
  standard_offsets_.push_back(Ogre::Vector3(-kR2, 0, kR2));
  standard_offsets_.push_back(Ogre::Vector3(-0.5, -0.5, kR2));
  standard_offsets_.push_back(Ogre::Vector3(0, -kR2, kR2));
  standard_offsets_.push_back(Ogre::Vector3(0.5, -0.5, kR2));

  // Add scaled versions of the standard viewpoints.
  int num_standard = standard_offsets_.size();
  for (int i = 0; i < num_standard; i++) {
    auto viewpoint = standard_offsets_[i];
    for (float scale = 0.5; scale < 2; scale += 0.1) {
      if (scale == 1) {
        continue;
      }
      standard_offsets_.push_back(viewpoint * scale);
    }
  }

  std::random_shuffle(standard_offsets_.begin(), standard_offsets_.end());
}

void Optimization::SelectViewpoints(std::vector<Viewpoint>* viewpoints) {
  // Get landmark positions, including the center.
  auto landmarks_object = sensing_->landmarks();
  std::vector<Landmark> landmarks;
  landmarks_object->LandmarksVector(&landmarks);
  std::vector<Ogre::Vector3> landmark_positions;
  for (const auto& landmark : landmarks) {
    landmark_positions.push_back(landmark.position);
  }
  Ogre::Vector3 center = landmarks_object->Center();
  landmark_positions.push_back(center);

  int num_landmarks = landmark_positions.size();
  int num_viewpoints =
      static_cast<int>(static_cast<float>(max_visibility_checks_)
          / num_landmarks);
  if (num_viewpoints < 1) {
    num_viewpoints = 1;
  } else if (num_viewpoints > num_landmarks * standard_offsets_.size()) {
    num_viewpoints = num_landmarks * standard_offsets_.size();
  }

  // Select a random subset of viewpoints.
  for (int i = 0; i < num_viewpoints; i++) {
    int rand_num = rand();
    int viewpoint_index = rand_num % standard_offsets_.size();
    int landmark_index = rand_num % num_landmarks;

    auto offset = standard_offsets_[viewpoint_index];
    auto landmark_position = landmark_positions[landmark_index];

    Viewpoint viewpoint(landmark_position + offset, landmark_position);
    viewpoints->push_back(viewpoint);
  }
}

void Optimization::ComputeViewpointScore(const Viewpoint& viewpoint,
                                         const Viewpoint& current_viewpoint,
                                         Score* score) {
  float score_numerator = 0;
  float score_denominator = 0;

  // Visibility score.
  float visibility_score = VisibilityScore(viewpoint);
  score_numerator += visibility_weight_ * visibility_score;
  score_denominator += visibility_weight_;
  score->visibility = visibility_score;

  // Orthogonality score.
  auto current_control = sensing_->current_control(only_move_on_idle_);
  if (current_control != NULL) {
    float ortho_score = ViewAngleScore(viewpoint, *current_control);
    score_numerator += view_angle_weight_ * ortho_score;
    score_denominator += view_angle_weight_;
    score->orthogonality = ortho_score;
  } else {
    score->orthogonality = -1;
  }

  // Zoom score.
  float zoom_score = ZoomScore(viewpoint);
  score_numerator += zoom_weight_ * zoom_score;
  score_denominator += zoom_weight_;
  score->zoom = zoom_score;

  // Travel score.
  float travel_score = TravelingScore(current_viewpoint, viewpoint);
  score_numerator += travel_weight_ * travel_score;
  score_denominator += travel_weight_;
  score->travel = travel_score;

  // Crossing score.
  if (current_control != NULL) {
    float crossing_score = CrossingScore(viewpoint, *current_control);
    score_numerator += crossing_weight_ * crossing_score;
    score_denominator += crossing_weight_;
    score->crossing = crossing_score;
  } else {
    score->crossing = -1;
  }

  if (score_denominator != 0) {
    score->score = score_numerator / score_denominator;
  } else {
    ROS_INFO("Warning: score function took nothing into account.");
    score->score = -1;
  }
}

float Optimization::VisibilityScore(const Viewpoint& viewpoint) {
  auto occlusion_metric = [&] (const Ogre::Vector3& point) -> float {
    bool is_visible = visibility_checker_->IsVisible(point, viewpoint);
    if (is_visible) {
      return 1;
    } else {
      return 0;
    }
  };
  return sensing_->landmarks()->ComputeMetric(occlusion_metric);
}

float Optimization::ViewAngleScore(const Viewpoint& viewpoint,
                                   const ClickedControl& control) {
  auto candidate_position_vector =
      viewpoint.position - control.world_position;
  Ogre::Vector3 projection;
  ComputeControlProjection(control, candidate_position_vector,
                           &projection);
  float cosineAngle = candidate_position_vector.dotProduct(projection);
  float magProd = candidate_position_vector.length() * projection.length();
  return fabs(cosineAngle / magProd);
}

float Optimization::ZoomScore(const Viewpoint& viewpoint) {
  auto zoom_metric = [&] (const Ogre::Vector3& point) -> float {
    auto distance = point.distance(viewpoint.position);
    if (distance < min_zoom_) {
      return 0;
    }
    return linearInterpolation(min_zoom_, 1, max_zoom_, 0, distance);
  };
  return sensing_->landmarks()->ComputeMetric(zoom_metric);
}

float Optimization::TravelingScore(const Viewpoint& current_viewpoint,
                                   const Viewpoint& candidate_viewpoint) {
  auto position_distance = current_viewpoint.position.distance(
      candidate_viewpoint.position);
  //auto focus_distance = current_viewpoint.focus.distance(
  //    candidate_viewpoint.focus);
  //auto average_distance = (position_distance + focus_distance) / 2;
  return linearInterpolation(0, 1, max_travel_, 0, position_distance);
}

float Optimization::CrossingScore(const Viewpoint& viewpoint,
                                  const ClickedControl& control) {
  auto camera_position = camera_->getPosition();
  auto control_position = control.world_position;
  int current_x_sign = sign(camera_position.x - control_position.x);
  int current_y_sign = sign(camera_position.y - control_position.y);
  int current_z_sign = sign(camera_position.z - control_position.z);
  int candidate_x_sign = sign(viewpoint.position.x - control_position.x);
  int candidate_y_sign = sign(viewpoint.position.y - control_position.y);
  int candidate_z_sign = sign(viewpoint.position.z - control_position.z);

  // If you're using an x control, don't cross the y=0 plane
  // If you're using a y control, don't cross the x=0 plane
  if (control.control == Control6Dof::X
      || control.control == Control6Dof::PITCH) {
    if (candidate_y_sign != current_y_sign) {
      return 0;
    }
  } else if (control.control == Control6Dof::Y
      || control.control == Control6Dof::ROLL) {
    if (candidate_x_sign != current_x_sign) {
      return 0;
    }
  } else if (control.control == Control6Dof::YAW){
    if (candidate_z_sign != current_z_sign) {
      return 0;
    }
  }
  return 1;
}

}
