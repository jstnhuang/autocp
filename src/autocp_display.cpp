#include "autocp_display.h"
#include "utils.h"

#include <algorithm>
#include <string>
#include <vector>
#include <sys/time.h>

namespace autocp {

/**
 * Constructor. Hooks up the display properties.
 */
AutoCPDisplay::AutoCPDisplay()
    : root_nh_(""), current_viewpoint_(), target_viewpoint_(), sensing_(NULL),
      visualization_(NULL) {
  topic_prop_ =
      new rviz::RosTopicProperty(
          "Command topic",
          "/rviz/camera_placement",
          QString::fromStdString(
              ros::message_traits::datatype<
                  view_controller_msgs::CameraPlacement>()),
          "Topic on which to send out camera placement messages.", this,
          SLOT(updateTopic()));

  // Landmark weights.
  gripper_weight_ = new rviz::FloatProperty(
      "Gripper focus weight", 0.25,
      "How much weight to assign to the grippers' locations.", this,
      SLOT(updateWeights()));
  gripper_weight_->setMin(0);
  gripper_weight_->setMax(1);

  head_weight_ = new rviz::FloatProperty(
      "Head weight", 0.25,
      "How much weight to the location of the robot's head.", this,
      SLOT(updateWeights()));
  head_weight_->setMin(0);
  head_weight_->setMax(1);

  head_focus_weight_ = new rviz::FloatProperty(
      "Head focus point weight", 0.25,
      "How much weight to assign to the location the robot is looking.", this,
      SLOT(updateWeights()));
  head_focus_weight_->setMin(0);
  head_focus_weight_->setMax(1);

  segmented_object_weight_ = new rviz::FloatProperty(
      "Segmented object weight", 0.25,
      "How much weight to assign to the locations of segmented objects", this,
      SLOT(updateWeights()));
  segmented_object_weight_->setMin(0);
  segmented_object_weight_->setMax(1);

  // Weights on location.
  stay_in_place_weight_ = new rviz::FloatProperty(
      "Movement moderation weight", 0.1,
      "How much weight to points close to the current location.", this,
      SLOT(updateWeights()));
  stay_in_place_weight_->setMin(0);
  stay_in_place_weight_->setMax(1);

  be_orthogonal_weight_ = new rviz::FloatProperty(
      "Marker orthogonality weight", 0.5,
      "How much weight to assign to points orthogonal to the current marker.",
      this, SLOT(updateWeights()));
  be_orthogonal_weight_->setMin(0);
  be_orthogonal_weight_->setMax(1);

  stay_visible_weight_ = new rviz::FloatProperty(
      "Marker visibility weight", 0.2,
      "How much weight to assign to points where the current marker is "
      "visible.",
      this, SLOT(updateWeights()));
  stay_visible_weight_->setMin(0);
  stay_visible_weight_->setMax(1);

  zoom_weight_ = new rviz::FloatProperty(
      "Zoom weight", 0.1,
      "How much weight to assign to being close to landmarks.", this,
      SLOT(updateWeights()));
  zoom_weight_->setMin(0);
  zoom_weight_->setMax(1);

  crossing_weight_property_ = new rviz::FloatProperty(
      "Line crossing weight", 0.1,
      "Weight for not flipping the user interface when using a control.", this,
      SLOT(updateWeights()));
  crossing_weight_property_->setMin(0);
  crossing_weight_property_->setMax(1);

  // Other properties.
  score_threshold_ = new rviz::FloatProperty(
      "Score improvement threshold", 1.05,
      "Factor by which the score must improve before adjusting the position.",
      this, SLOT(updateWeights()));
  score_threshold_->setMin(0);
  score_threshold_->setMax(30);

  camera_speed_ = new rviz::FloatProperty(
      "Camera speed", 3, "How many meters per second the camera can move.",
      this, SLOT(updateCameraOptions()));
  camera_speed_->setMin(0);
  camera_speed_->setMax(10);

  only_move_on_idle_ = new rviz::BoolProperty(
      "Don't move when using a marker", false,
      "Restricts camera repositioning to when no marker is being used.", this,
      SLOT(updateSmoothnessOption()));

  occlusion_check_limit_ = new rviz::IntProperty(
      "Occlusion check limit", 1000,
      "The number of occlusions to check each frame.", this,
      SLOT(updateCameraOptions()));
  occlusion_check_limit_->setMin(0);
  occlusion_check_limit_->setMax(10000);

  show_fps_ = new rviz::BoolProperty(
      "Show FPS", false, "Whether or not to show the frames per second.", this,
      SLOT(updateCameraOptions()));
}

/**
 * Destructor.
 */
AutoCPDisplay::~AutoCPDisplay() {
}

/**
 * Initialization. Attaches callbacks to subscribers.
 */
void AutoCPDisplay::onInitialize() {
  Display::onInitialize();
  sensing_ = new AutoCPSensing(root_nh_, &tf_listener_,
                               fixed_frame_.toStdString());
  sensing_->Initialize();
  updateTopic();
  updateWeights();

  vm_ = static_cast<rviz::VisualizationManager*>(context_);
  sm_ = vm_->getSceneManager();
  camera_ = vm_->getRenderPanel()->getCamera();
  visibility_checker_ = new VisibilityChecker(sm_, camera_);

  current_viewpoint_ = Viewpoint(camera_->getPosition(),
                                 *(sensing_->head_position()));
  target_viewpoint_ = current_viewpoint_;

  visualization_ = new Visualization(root_nh_, fixed_frame_.toStdString());
  initializeStandardViewpoints();
}

void AutoCPDisplay::initializeStandardViewpoints() {
  // Lower plane.
  standard_viewpoints_.push_back(Ogre::Vector3(1, 0, 0));
  standard_viewpoints_.push_back(Ogre::Vector3(R2, R2, 0));
  standard_viewpoints_.push_back(Ogre::Vector3(0, 1, 0));
  standard_viewpoints_.push_back(Ogre::Vector3(-R2, R2, 0));
  standard_viewpoints_.push_back(Ogre::Vector3(-1, 0, 0));
  standard_viewpoints_.push_back(Ogre::Vector3(-R2, -R2, 0));
  standard_viewpoints_.push_back(Ogre::Vector3(0, -1, 0));
  standard_viewpoints_.push_back(Ogre::Vector3(R2, -R2, 0));
  // 45 degrees up.
  standard_viewpoints_.push_back(Ogre::Vector3(R2, 0, R2));
  standard_viewpoints_.push_back(Ogre::Vector3(0.5, 0.5, R2));
  standard_viewpoints_.push_back(Ogre::Vector3(0, R2, R2));
  standard_viewpoints_.push_back(Ogre::Vector3(-0.5, 0.5, R2));
  standard_viewpoints_.push_back(Ogre::Vector3(-R2, 0, R2));
  standard_viewpoints_.push_back(Ogre::Vector3(-0.5, -0.5, R2));
  standard_viewpoints_.push_back(Ogre::Vector3(0, -R2, R2));
  standard_viewpoints_.push_back(Ogre::Vector3(0.5, -0.5, R2));

  // Add scaled versions of the standard viewpoints.
  int num_standard = standard_viewpoints_.size();
  std::vector<float> scales = { 0.5, 1.5, 2, 2.5, 3 };
  for (int i = 0; i < num_standard; i++) {
    auto viewpoint = standard_viewpoints_[i];
    for (int j = 0; j < scales.size(); j++) {
      standard_viewpoints_.push_back(viewpoint *scales[j]);
    }
  }

  std::random_shuffle(standard_viewpoints_.begin(), standard_viewpoints_.end());
}

// Parameter update handlers ---------------------------------------------------
/**
 * Set the topic to publish camera placement commands to.
 */
void AutoCPDisplay::updateTopic() {
  camera_placement_publisher_ = root_nh_
      .advertise<view_controller_msgs::CameraPlacement>(
      topic_prop_->getStdString(), 5);
}

/**
 * No-op for camera options.
 */
void AutoCPDisplay::updateCameraOptions() {
  return;
}

/**
 * Updates the weights.
 */
void AutoCPDisplay::updateWeights() {
  auto landmarks = sensing_->landmarks();
  landmarks->UpdateGripperWeight(gripper_weight_->getFloat());
  landmarks->UpdateHeadWeight(head_weight_->getFloat());
  landmarks->UpdateHeadFocusWeight(head_focus_weight_->getFloat());
  landmarks->UpdateSegmentedObjectWeight(segmented_object_weight_->getFloat());
}

// Camera placement logic ------------------------------------------------------
/**
 * Main loop that alternates between sensing and placing the camera.
 */
void AutoCPDisplay::update(float wall_dt, float ros_dt) {
  sensing_->Update();
  if (sensing_->landmarks()->NumLandmarks() == 0) {
    return;
  }
  current_viewpoint_.position = camera_->getPosition();
  chooseCameraPlacement(wall_dt);

  if (show_fps_->getBool()) {
    ROS_INFO("FPS: %f", 1 / wall_dt);
  }
}

/**
 * Get the final focus point and location for the camera, then push the camera
 * placement.
 */
void AutoCPDisplay::chooseCameraPlacement(float time_delta) {
  if (!only_move_on_idle_->getBool() || sensing_->IsControlActive()) {
    chooseCameraViewpoint(&target_viewpoint_);
  }
  Viewpoint next_viewpoint;
  interpolateViewpoint(current_viewpoint_, target_viewpoint_,
                       camera_speed_->getFloat(), 0.5, time_delta,
                       &next_viewpoint);
  visualization_->ShowFocus(next_viewpoint.focus);

  view_controller_msgs::CameraPlacement camera_placement;
  setCameraPlacement(next_viewpoint, ros::Duration(time_delta),
                     &camera_placement);
  camera_placement_publisher_.publish(camera_placement);
  current_viewpoint_ = next_viewpoint;
}

/**
 * Visibility score. Returns the weighted average visibility of all the
 * landmarks.
 */
float AutoCPDisplay::visibilityScore(const Viewpoint& viewpoint) {
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

/**
 * Orthogonality score. Returns 1 if the current control is orthogonal to the
 * movement of the current control, 0 if it is perfectly parallel, and the
 * absolute value of the cosine of the angle otherwise.
 */
float AutoCPDisplay::orthogonalityScore(const Viewpoint& candidate_position,
                                        const ClickedControl& control) {
  auto candidate_position_vector =
      candidate_position.position - control.world_position;
  Ogre::Vector3 projection;
  computeControlProjection(control, candidate_position_vector,
                           &projection);
  float cosineAngle = candidate_position_vector.dotProduct(projection);
  float magProd = candidate_position_vector.length() * projection.length();
  return fabs(cosineAngle / magProd);
}

/**
 * Zoom score. Returns 1 if we are very close, 0 if we are far away, and a
 * linearly interpolated value if we are in between. Close and far are defined
 * by MIN_DISTANCE and MAX_DISTANCE.
 */
float AutoCPDisplay::zoomScore(const Viewpoint& viewpoint) {
  auto zoom_metric = [&] (const Ogre::Vector3& point) -> float {
    auto distance = point.distance(viewpoint.position);
    if (distance < MIN_DISTANCE) {
      return 0;
    }
    return linearInterpolation(MIN_DISTANCE, 1, MAX_DISTANCE, 0, distance);
  };
  return sensing_->landmarks()->ComputeMetric(zoom_metric);
}

/**
 * Smoothness score. Returns 1 if the camera has not moved much, 0 if it has
 * moved a lot, and a number in between mediated by the logisticDistance
 * function.
 */
float AutoCPDisplay::travelingScore(const Viewpoint& current_viewpoint,
                                    const Viewpoint& candidate_viewpoint) {
  auto position_distance = current_viewpoint_.position.distance(
      candidate_viewpoint.position);
  auto focus_distance = current_viewpoint_.focus.distance(
      candidate_viewpoint.focus);
  auto average_distance = (position_distance + focus_distance) / 2;
  return linearInterpolation(0, 1, 2, 0, average_distance);
}

/**
 * Line crossing score. The score is 0 if the candidate position would "flip"
 * the direction of motion when a user is using an interactive marker, and 1
 * otherwise.
 */
float AutoCPDisplay::crossingScore(const Viewpoint& viewpoint,
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

/**
 * Computes a score for the location.
 */
void AutoCPDisplay::computeViewpointScore(const Viewpoint& viewpoint,
                                          Score* score) {
  float score_numerator = 0;
  float score_denominator = 0;

  // Visibility score.
  float visibility_score = visibilityScore(viewpoint);
  float visibility_weight = stay_visible_weight_->getFloat();
  score_numerator += visibility_weight * visibility_score;
  score_denominator += visibility_weight;
  score->visibility = visibility_score;

  // Orthogonality score.
  auto current_control = sensing_->current_control(
      only_move_on_idle_->getBool());
  if (current_control != NULL) {
    float ortho_score = orthogonalityScore(viewpoint, *current_control);
    float ortho_weight = be_orthogonal_weight_->getFloat();
    score_numerator += ortho_weight * ortho_score;
    score_denominator += ortho_weight;
    score->orthogonality = ortho_score;
  } else {
    score->orthogonality = -1;
  }

  // Zoom score.
  float zoom_score = zoomScore(viewpoint);
  float zoom_weight = zoom_weight_->getFloat();
  score_numerator += zoom_weight * zoom_score;
  score_denominator += zoom_weight;
  score->zoom = zoom_score;

  // Travel score.
  float travel_score = travelingScore(current_viewpoint_, viewpoint);
  float travel_weight = stay_in_place_weight_->getFloat();
  score_numerator += travel_weight * travel_score;
  score_denominator += travel_weight;
  score->travel = travel_score;

  // Crossing score.
  if (current_control != NULL) {
    float crossing_score = crossingScore(viewpoint, *current_control);
    float crossing_weight = crossing_weight_property_->getFloat();
    score_numerator += crossing_weight * crossing_score;
    score_denominator += crossing_weight;
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

/*
 * Randomly select some viewpoints. The number of
 * viewpoints to select should be occlusion_check_limit_ / (# of landmarks)
 */
void AutoCPDisplay::selectViewpoints(std::vector<Viewpoint>* viewpoints) {
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
  int check_limit = occlusion_check_limit_->getInt();
  int num_viewpoints = static_cast<int>(
      static_cast<float>(check_limit) / num_landmarks);
  if (num_viewpoints < 1) {
    num_viewpoints = 1;
  } else if (num_viewpoints > standard_viewpoints_.size()) {
    num_viewpoints = standard_viewpoints_.size();
  }

  // Select a random subset of viewpoints.
  for (int i = 0; i < num_viewpoints; i++) {
    int rand_num = rand();
    int viewpoint_index = rand_num % standard_viewpoints_.size();
    int landmark_index = rand_num % num_landmarks;

    auto offset = standard_viewpoints_[viewpoint_index];
    auto landmark_position = landmark_positions[landmark_index];

    Viewpoint viewpoint(landmark_position + offset, landmark_position);
    viewpoints->push_back(viewpoint);
  }
}

/**
 * Choose a location for the camera. It searches through the list of standard
 * viewpoints and selects the highest scoring one. Returns true if a new
 * location was found.
 */
void AutoCPDisplay::chooseCameraViewpoint(Viewpoint* target_viewpoint) {
  std::vector<Viewpoint> test_viewpoints;
  std::vector<Score> scores;

  // Assume that the current position is the best, to start with.
  Score current_score;
  computeViewpointScore(current_viewpoint_, &current_score);
  Score best_score = current_score;
  Viewpoint best_viewpoint = current_viewpoint_;
  test_viewpoints.push_back(current_viewpoint_);
  scores.push_back(current_score);

  // The current viewpoint doesn't need to overcome the score threshold.
  *target_viewpoint = best_viewpoint;

  Score target_score;
  computeViewpointScore(*target_viewpoint, &target_score);
  test_viewpoints.push_back(*target_viewpoint);
  scores.push_back(target_score);
  if (target_score.score > best_score.score) {
    best_score = target_score;
    best_viewpoint = *target_viewpoint;
  }

  std::vector<Viewpoint> viewpoints;
  selectViewpoints(&viewpoints);

  for (const auto& viewpoint : viewpoints) {
    Score score;
    computeViewpointScore(viewpoint, &score);
    test_viewpoints.push_back(viewpoint);
    scores.push_back(score);

    if (score.score > best_score.score) {
      best_viewpoint = viewpoint;
      best_score = score;
    }
  }

  if (best_score.score > score_threshold_->getFloat() * current_score.score) {
    visualization_->ShowViewpoints(test_viewpoints, scores);
    auto best_position = best_viewpoint.position;
    ROS_INFO("Moving to (%.2f, %.2f, %.2f), score=%s, prev=%.2f",
             best_position.x, best_position.y, best_position.z,
             best_score.toString().c_str(), current_score.score);
    *target_viewpoint = best_viewpoint;
  }
}

// Utilities -------------------------------------------------------------------
/**
 * Convenience method to set the camera placement.
 */
void AutoCPDisplay::setCameraPlacement(
    const Viewpoint& viewpoint,
    const ros::Duration& time_from_start,
    view_controller_msgs::CameraPlacement* camera_placement) {
  camera_placement->target_frame = "<Fixed Frame>";

  camera_placement->time_from_start = time_from_start;

  camera_placement->eye.header.stamp = ros::Time::now();
  camera_placement->eye.header.frame_id = "<Fixed Frame>";
  camera_placement->focus.header.stamp = ros::Time::now();
  camera_placement->focus.header.frame_id = "<Fixed Frame>";
  camera_placement->up.header.stamp = ros::Time::now();
  camera_placement->up.header.frame_id = "<Fixed Frame>";

  camera_placement->eye.point.x = viewpoint.position.x;
  camera_placement->eye.point.y = viewpoint.position.y;
  camera_placement->eye.point.z = viewpoint.position.z;

  camera_placement->focus.point.x = viewpoint.focus.x;
  camera_placement->focus.point.y = viewpoint.focus.y;
  camera_placement->focus.point.z = viewpoint.focus.z;

  camera_placement->up.vector.x = 0.0;
  camera_placement->up.vector.y = 0.0;
  camera_placement->up.vector.z = 1.0;
}

/**
 * Compute the projection of the vector onto the orthogonal plane or line
 * defined by the current control.
 */
void AutoCPDisplay::computeControlProjection(
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

}  // namespace autocp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autocp::AutoCPDisplay, rviz::Display)
