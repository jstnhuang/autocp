
#include <algorithm>

#include <OGRE/OgreVector3.h>
#include <ros/ros.h>

#include "autocp/optimization.h"
#include "autocp/utils.h"

namespace autocp {

Optimization::Optimization(AutoCPSensing* sensing,
                           VisibilityChecker* visibility_checker,
                           Ogre::Camera* camera)
    : standard_offsets_(),
      offset_index_(0),
      visibility_weight_(0),
      centering_weight_(0),
      view_angle_weight_(0),
      zoom_weight_(0),
      max_visibility_checks_(1000),
      score_threshold_(1.05),
      min_zoom_(0.5),
      max_zoom_(5),
      max_travel_(1) {
  sensing_ = sensing;
  visibility_checker_ = visibility_checker;
  camera_ = camera;
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
  //*target_viewpoint = current_viewpoint;

  Score best_score;
  Viewpoint best_viewpoint;

  Score target_score;
  ComputeViewpointScore(*target_viewpoint, &target_score);
  test_viewpoints.push_back(*target_viewpoint);
  scores.push_back(target_score);
  best_score = target_score;
  best_viewpoint = *target_viewpoint;

  std::vector<Viewpoint> viewpoints;
  SelectViewpoints(&viewpoints);

  for (const auto& viewpoint : viewpoints) {
    Score score;
    ComputeViewpointScore(viewpoint, &score);
    test_viewpoints.push_back(viewpoint);
    scores.push_back(score);

    if (score.score > best_score.score) {
      best_viewpoint = viewpoint;
      best_score = score;
    }
  }

  if (best_score.score > score_threshold_ * target_score.score) {
    visualization_->ShowViewpoints(test_viewpoints, scores);
    auto best_position = best_viewpoint.position();
    ROS_INFO("Moving to (%.2f, %.2f, %.2f), score=%s, prev=(%.2f, %.2f, %.2f)=%s",
             best_position.x, best_position.y, best_position.z,
             best_score.toString().c_str(), target_viewpoint->position().x,
             target_viewpoint->position().y, target_viewpoint->position().z,
             target_score.toString().c_str());
    *target_viewpoint = best_viewpoint;
  }
}

void Optimization::set_visibility_weight(float weight) {
  visibility_weight_ = weight;
}

void Optimization::set_centering_weight(float weight) {
  centering_weight_ = weight;
}

void Optimization::set_view_angle_weight(float weight) {
  view_angle_weight_ = weight;
}

void Optimization::set_zoom_weight(float weight) {
  zoom_weight_ = weight;
}

void Optimization::set_max_visibility_checks(int max_visibility_checks) {
  max_visibility_checks_ = max_visibility_checks;
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
  // 70 degrees up.
  standard_offsets_.push_back(Ogre::Vector3(0.34, 0, 0.94));
  standard_offsets_.push_back(Ogre::Vector3(-0.34, 0, 0.94));
  standard_offsets_.push_back(Ogre::Vector3(0, 0.34, 0.94));
  standard_offsets_.push_back(Ogre::Vector3(0, -0.34, 0.94));
  standard_offsets_.push_back(Ogre::Vector3(0.24, 0.24, 0.94));
  standard_offsets_.push_back(Ogre::Vector3(0.24, -0.24, 0.94));
  standard_offsets_.push_back(Ogre::Vector3(-0.24, 0.24, 0.94));
  standard_offsets_.push_back(Ogre::Vector3(-0.24, -0.24, 0.94));

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

  // Select a random subset of viewpoints. Each landmark gets an equal number of
  // offsets. The standard offsets are already randomly shuffled, so we just
  // iterate through them, starting at offset_index_.
  int num_offsets = num_viewpoints / num_landmarks;
  for (const auto& landmark_position : landmark_positions) {
    for (int num_added = 0; num_added < num_offsets; num_added++) {
      auto offset = standard_offsets_[offset_index_];
      Viewpoint viewpoint(landmark_position + offset, landmark_position);
      viewpoints->push_back(viewpoint);
      offset_index_++;
      if (offset_index_ >= standard_offsets_.size()) {
        offset_index_ = 0;
      }
    }
  }
}

void Optimization::ComputeViewpointScore(const Viewpoint& viewpoint,
                                         Score* score) {
  float score_numerator = 0;
  float score_denominator = 0;

  // Visibility score.
  float visibility_score = VisibilityScore(viewpoint);
  score_numerator += visibility_weight_ * visibility_score;
  score_denominator += visibility_weight_;
  score->visibility = visibility_weight_ * visibility_score;

  // Centering score.
  float centering_score = CenteringScore(viewpoint);
  score_numerator += centering_weight_ * centering_score;
  score_denominator += centering_weight_;
  score->centering = centering_weight_ * centering_score;

  // Orthogonality score.
  auto previous_control = sensing_->previous_control();
  if (previous_control != NULL) {
    float ortho_score = ViewAngleScore(viewpoint, *previous_control);
    score_numerator += view_angle_weight_ * ortho_score;
    score_denominator += view_angle_weight_;
    score->orthogonality = view_angle_weight_ * ortho_score;
  } else {
    score->orthogonality = 0;
  }

  // Zoom score.
  float zoom_score = ZoomScore(viewpoint);
  score_numerator += zoom_weight_ * zoom_score;
  score_denominator += zoom_weight_;
  score->zoom = zoom_weight_ * zoom_score;

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

float Optimization::CenteringScore(const Viewpoint& viewpoint) {
  auto centering_metric = [&] (const Ogre::Vector3& point) -> float {
    float screen_x;
    float screen_y;
    visibility_checker_->GetScreenPosition(point, &screen_x, &screen_y);
    float x_dist = screen_x - 0.5;
    float y_dist = screen_y - 0.5;
    float squared_dist = x_dist * x_dist + y_dist * y_dist;
    return linearInterpolation(0, 1, 0.5, 0, squared_dist);
  };
  return sensing_->landmarks()->ComputeMetric(centering_metric);
}

float Optimization::ViewAngleScore(const Viewpoint& viewpoint,
                                   const ClickedControl& control) {
  auto candidate_position_vector =
      viewpoint.position() - control.world_position;
  Ogre::Vector3 projection;
  ComputeControlProjection(control, candidate_position_vector,
                           &projection);
  float cosineAngle = candidate_position_vector.dotProduct(projection);
  float magProd = candidate_position_vector.length() * projection.length();
  return fabs(cosineAngle / magProd);
}

float Optimization::ZoomScore(const Viewpoint& viewpoint) {
  auto zoom_metric = [&] (const Ogre::Vector3& point) -> float {
    auto distance = point.distance(viewpoint.position());
    if (distance < min_zoom_) {
      return 0;
    }
    return linearInterpolation(min_zoom_, 1, max_zoom_, 0, distance);
  };
  return sensing_->landmarks()->ComputeMetric(zoom_metric);
}

}
