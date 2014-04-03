#include "autocp_display.h"
#include "utils.h"

#include <algorithm>
#include <string>
#include <vector>
#include <sys/time.h>

namespace autocp {
using geometry_msgs::Point;
using geometry_msgs::Vector3;
using geometry_msgs::Quaternion;
using visualization_msgs::Marker;

/**
 * Constructor. Hooks up the display properties.
 */
AutoCPDisplay::AutoCPDisplay()
    : root_nh_("") {
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

  initializeStandardViewpoints();
  candidate_marker_pub_ = root_nh_.advertise<Marker>("autocp_markers", 1);
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
  updateTopic();
  updateWeights();

  point_head_subscriber_ = root_nh_.subscribe(
      "head_traj_controller/point_head_action/goal", 5,
      &AutoCPDisplay::pointHeadCallback, this);

  marker_feedback_subscriber_ = root_nh_.subscribe(
      "/pr2_marker_control_transparent/feedback", 5,
      &AutoCPDisplay::markerCallback, this);

  object_segmentation_subscriber_ = root_nh_.subscribe(
      "/interactive_object_recognition_result", 5,
      &AutoCPDisplay::objectSegmentationCallback, this);

  // Just read the first message to get the initial head focus point.
  full_marker_subscriber_ = root_nh_.subscribe(
      "/pr2_marker_control_transparent/update_full", 5,
      &AutoCPDisplay::fullMarkerCallback, this);

  // Get the head location.
  Point head_position;
  getHeadPosition(&head_position);
  landmarks_.UpdateHead(&head_position);

  vm_ = static_cast<rviz::VisualizationManager*>(context_);
  sm_ = vm_->getSceneManager();
  camera_ = vm_->getRenderPanel()->getCamera();
  visibility_checker_ = new VisibilityChecker(sm_, camera_);
  viewport_ = camera_->getViewport();
  target_position_ = getCameraPosition();
  current_control_ = NULL;
  active_control_ = NULL;
  previous_control_ = NULL;
  num_prev_markers_ = 0;
}

void AutoCPDisplay::initializeStandardViewpoints() {
  // Lower plane.
  standard_viewpoints_.push_back(makeVector3(1, 0, 0));
  standard_viewpoints_.push_back(makeVector3(R2, R2, 0));
  standard_viewpoints_.push_back(makeVector3(0, 1, 0));
  standard_viewpoints_.push_back(makeVector3(-R2, R2, 0));
  standard_viewpoints_.push_back(makeVector3(-1, 0, 0));
  standard_viewpoints_.push_back(makeVector3(-R2, -R2, 0));
  standard_viewpoints_.push_back(makeVector3(0, -1, 0));
  standard_viewpoints_.push_back(makeVector3(R2, -R2, 0));
  // 45 degrees up.
  standard_viewpoints_.push_back(makeVector3(R2, 0, R2));
  standard_viewpoints_.push_back(makeVector3(0.5, 0.5, R2));
  standard_viewpoints_.push_back(makeVector3(0, R2, R2));
  standard_viewpoints_.push_back(makeVector3(-0.5, 0.5, R2));
  standard_viewpoints_.push_back(makeVector3(-R2, 0, R2));
  standard_viewpoints_.push_back(makeVector3(-0.5, -0.5, R2));
  standard_viewpoints_.push_back(makeVector3(0, -R2, R2));
  standard_viewpoints_.push_back(makeVector3(0.5, -0.5, R2));

  // Add scaled versions of the standard viewpoints.
  int num_standard = standard_viewpoints_.size();
  std::vector<float> scales = { 0.5, 1.5, 2, 2.5, 3 };
  for (int i = 0; i < num_standard; i++) {
    auto viewpoint = standard_viewpoints_[i];
    for (int j = 0; j < scales.size(); j++) {
      standard_viewpoints_.push_back(scale(viewpoint, scales[j]));
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
 * Update the weight vector, and normalize so that the max is 1.
 */
void AutoCPDisplay::updateWeights() {
  landmarks_.UpdateGripperWeight(gripper_weight_->getFloat());
  landmarks_.UpdateHeadWeight(head_weight_->getFloat());
  landmarks_.UpdateHeadFocusWeight(head_focus_weight_->getFloat());
  landmarks_.UpdateSegmentedObjectWeight(segmented_object_weight_->getFloat());
  crossing_weight_ = crossing_weight_property_->getFloat();
}

/**
 * If we are only going to move the camera when no controls are being used, then
 * the control we care about is the last control that was used. If we will move
 * the camera when a control is being used, then that's the control we care
 * about.
 */
void AutoCPDisplay::updateSmoothnessOption() {
  if (only_move_on_idle_) {
    current_control_ = previous_control_;
  } else {
    current_control_ = active_control_;
  }
}

// Sensing ---------------------------------------------------------------------
/**
 * Get the origin of the given transform relative to the fixed frame.
 */
void AutoCPDisplay::getTransformOrigin(std::string frame, Point* origin) {
  ros::Duration timeout(5);
  tf_listener_.waitForTransform(fixed_frame_.toStdString(), frame, ros::Time(0),
                                timeout);
  tf::StampedTransform transform;
  tf_listener_.lookupTransform(fixed_frame_.toStdString(), frame, ros::Time(0),
                               transform);
  tf::Vector3 transform_origin = transform.getOrigin();

  origin->x = transform_origin.x();
  origin->y = transform_origin.y();
  origin->z = transform_origin.z();
}

/**
 * Gets the position of the head relative to the fixed frame.
 */
void AutoCPDisplay::getHeadPosition(Point* head_position) {
  getTransformOrigin("/head_tilt_link", head_position);
}

/**
 * Get the target point for the head.
 */
void AutoCPDisplay::pointHeadCallback(
    const pr2_controllers_msgs::PointHeadActionGoal& action_goal) {
  head_focus_point_ = action_goal.goal.target.point;
  landmarks_.UpdateHeadFocus(&head_focus_point_);
}

/**
 * When an object is segmented, compute the average point in the point cloud and
 * update the segmented_object_positions_ vector.
 */
void AutoCPDisplay::objectSegmentationCallback(
    const manipulation_msgs::GraspableObjectList& list) {
  segmented_object_positions_.clear();
  for (const auto& obj : list.graspable_objects) {
    Point obj_location;
    for (const auto& point : obj.cluster.points) {
      obj_location.x += point.x;
      obj_location.y += point.y;
      obj_location.z += point.z;
    }
    obj_location.x /= obj.cluster.points.size();
    obj_location.y /= obj.cluster.points.size();
    obj_location.z /= obj.cluster.points.size();
    segmented_object_positions_.push_back(obj_location);
  }
  landmarks_.UpdateSegmentedObjects(segmented_object_positions_);
}

/**
 * Get the location of the interactive marker currently being used.
 */
void AutoCPDisplay::markerCallback(
    const visualization_msgs::InteractiveMarkerFeedback& feedback) {
  if (feedback.event_type
      != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    return;
  }
  std::string marker_name = static_cast<std::string>(feedback.marker_name);
  std::string control_name = static_cast<std::string>(feedback.control_name);

  Control6Dof control;
  Point world_position = feedback.pose.position;
  try {
    if (marker_name == "head_point_goal") {
      control = POINT_HEAD_CONTROLS.at(control_name);
    } else if (marker_name == "l_gripper_control") {
      control = GRIPPER_CONTROLS.at(control_name);
      world_position = left_gripper_origin_;
    } else if (marker_name == "r_gripper_control") {
      control = GRIPPER_CONTROLS.at(control_name);
      world_position = right_gripper_origin_;
    } else if (marker_name == "l_posture_control") {
      control = Control6Dof::ROLL;
    } else if (marker_name == "r_posture_control") {
      control = Control6Dof::ROLL;
    } else {
      ROS_INFO("Unknown marker %s", marker_name.c_str());
      return;
    }

    active_control_ = new ClickedControl { marker_name, control, feedback.pose,
        world_position };
    updateSmoothnessOption();
  } catch (const std::out_of_range& e) {
    ROS_INFO("Unknown control %s for marker %s", control_name.c_str(),
             marker_name.c_str());
  }
}

/**
 * Get the state of all the markers. This differs from the regular callback
 * because we get data from this topic on startup, while markerCallback only
 * gets called when a marker is being used.
 */
void AutoCPDisplay::fullMarkerCallback(
    const visualization_msgs::InteractiveMarkerInit& im_init) {
  landmarks_.UpdateHeadFocus(NULL);
  landmarks_.UpdateRightGripper(NULL);
  landmarks_.UpdateLeftGripper(NULL);
  for (const auto& marker : im_init.markers) {
    if (marker.name == "head_point_goal") {
      landmarks_.UpdateHeadFocus(&marker.pose.position);
    } else if (marker.name == "r_gripper_control") {
      landmarks_.UpdateRightGripper(&marker.pose.position);
    } else if (marker.name == "l_gripper_control") {
      landmarks_.UpdateLeftGripper(&marker.pose.position);
    }
  }
}

// Camera placement logic ------------------------------------------------------
/**
 * Main loop that alternates between sensing and placing the camera.
 */
void AutoCPDisplay::update(float wall_dt, float ros_dt) {
  if (landmarks_.NumLandmarks() == 0) {
    return;
  }
  chooseCameraPlacement(wall_dt);

  if (show_fps_->getBool()) {
    ROS_INFO("FPS: %f", 1 / wall_dt);
  }

  if (active_control_ != NULL) {
    if (previous_control_ != NULL) {
      delete previous_control_;
    }
    previous_control_ = active_control_;
    active_control_ = NULL;
  }
  updateSmoothnessOption();
}

/**
 * Get the final focus point and location for the camera, then push the camera
 * placement.
 */
void AutoCPDisplay::chooseCameraPlacement(float time_delta) {
  if (!only_move_on_idle_->getBool() || active_control_ == NULL) {
    chooseCameraLocation(&target_position_, time_delta);
  }
  Point next_position = interpolatePosition(getCameraPosition(),
                                            target_position_, time_delta);

  view_controller_msgs::CameraPlacement camera_placement;
  setCameraPlacement(next_position, target_focus_, ros::Duration(time_delta),
                     &camera_placement);

  camera_placement_publisher_.publish(camera_placement);
}

/**
 * Visibility score. Returns the weighted average visibility of all the
 * landmarks.
 */
float AutoCPDisplay::visibilityScore(const Point& candidate_position,
                                     const Point& candidate_focus) {
  Ogre::Vector3 position(candidate_position.x, candidate_position.y,
                         candidate_position.z);
  Ogre::Vector3 focus(candidate_focus.x, candidate_focus.y, candidate_focus.z);
  Viewpoint viewpoint(position, focus);
  auto occlusion_metric = [&] (const Point& point) -> float {
    Ogre::Vector3 p(point.x, point.y, point.z);
    Ogre::Vector3 hit;

    bool is_visible = visibility_checker_->IsVisible(p, viewpoint);
    Marker marker;
    makePointMarker(toPoint(hit), 0, &marker);
    candidate_marker_pub_.publish(marker);
    if (is_visible) {
      return 1;
    } else {
      return 0;
    }
  };
  return landmarks_.ComputeMetric(occlusion_metric);
}

/**
 * Orthogonality score. Returns 1 if the current control is orthogonal to the
 * movement of the current control, 0 if it is perfectly parallel, and the
 * absolute value of the cosine of the angle otherwise.
 */
float AutoCPDisplay::orthogonalityScore(
    const Point& candidate_position,
    const Point& control_position) {
  if (current_control_ == NULL) {
    ROS_INFO("Warning: tried to get orthogonality score without a control.");
    return 0;
  }
  Vector3 candidate_position_vector = vectorBetween(control_position,
                                                    candidate_position);
  Vector3 projection = computeControlProjection(*current_control_,
                                                candidate_position_vector);
  return fabs(cosineAngle(candidate_position_vector, projection));
}

/**
 * Zoom score. Returns 1 if we are very close, 0 if we are far away, and a
 * linearly interpolated value if we are in between. Close and far are defined
 * by MIN_DISTANCE and MAX_DISTANCE.
 */
float AutoCPDisplay::zoomScore(const Point& candidate_position) {
  auto zoom_metric = [&] (const Point& point) -> float {
    float dist = distance(point, candidate_position);
    if (dist < MIN_DISTANCE) {
      return 0;
    }
    return linearInterpolation(MIN_DISTANCE, 1, MAX_DISTANCE, 0, dist);
  };
  return landmarks_.ComputeMetric(zoom_metric);
}

/**
 * Smoothness score. Returns 1 if the camera has not moved much, 0 if it has
 * moved a lot, and a number in between mediated by the logisticDistance
 * function.
 */
float AutoCPDisplay::travelingScore(const Point& candidate_position) {
  auto camera_position = toPoint(camera_->getPosition());
  auto dist = distance(candidate_position, camera_position);
  return linearInterpolation(0, 1, 5, 0, dist);
}

/**
 * Line crossing score. The score is 0 if the candidate position would "flip"
 * the direction of motion when a user is using an interactive marker, and 1
 * otherwise.
 */
float AutoCPDisplay::crossingScore(const Point& candidate_position) {
  if (current_control_ == NULL) {
    ROS_INFO("Warning: tried to compute crossing score without a control.");
    return 0;
  }
  Point camera_position = getCameraPosition();
  Point control_position = current_control_->world_position;
  int current_x_sign = sign(camera_position.x - control_position.x);
  int current_y_sign = sign(camera_position.y - control_position.y);
  int current_z_sign = sign(camera_position.z - control_position.z);
  int candidate_x_sign = sign(candidate_position.x - control_position.x);
  int candidate_y_sign = sign(candidate_position.y - control_position.y);
  int candidate_z_sign = sign(candidate_position.z - control_position.z);

  // If you're using an x control, don't cross the y=0 plane
  // If you're using a y control, don't cross the x=0 plane
  if (current_control_->control == Control6Dof::X
      || current_control_->control == Control6Dof::PITCH) {
    if (candidate_y_sign != current_y_sign) {
      return 0;
    }
  } else if (current_control_->control == Control6Dof::Y
      || current_control_->control == Control6Dof::ROLL) {
    if (candidate_x_sign != current_x_sign) {
      return 0;
    }
  } else if (current_control_->control == Control6Dof::YAW){
    if (candidate_z_sign != current_z_sign) {
      return 0;
    }
  }
  return 1;
}

/**
 * Computes a score for the location.
 */
Score AutoCPDisplay::computeLocationScore(const Point& candidate_position,
                                          const Point& candidate_focus) {
  Score result;
  float score_numerator = 0;
  float score_denominator = 0;

  // Visibility score.
  float visibility_score = visibilityScore(candidate_position, candidate_focus);
  float visibility_weight = stay_visible_weight_->getFloat();
  score_numerator += visibility_weight * visibility_score;
  score_denominator += visibility_weight;
  result.visibility = visibility_score;

  // Orthogonality score.
  Point control_location;
  if (current_control_ != NULL) {
    control_location = current_control_->world_position;
    float ortho_score = orthogonalityScore(candidate_position,
                                           control_location);
    float ortho_weight = be_orthogonal_weight_->getFloat();
    score_numerator += ortho_weight * ortho_score;
    score_denominator += ortho_weight;
    result.orthogonality = ortho_score;
  } else {
    result.orthogonality = -1;
  }

  // Zoom score.
  float zoom_score = zoomScore(candidate_position);
  float zoom_weight = zoom_weight_->getFloat();
  score_numerator += zoom_weight * zoom_score;
  score_denominator += zoom_weight;
  result.zoom = zoom_score;

  // Travel score.
  float travel_score = travelingScore(candidate_position);
  float travel_weight = stay_in_place_weight_->getFloat();
  score_numerator += travel_weight * travel_score;
  score_denominator += travel_weight;
  result.travel = travel_score;

  // Crossing score.
  if (current_control_ != NULL) {
    float crossing_score = crossingScore(candidate_position);
    score_numerator += crossing_weight_ * crossing_score;
    score_denominator += crossing_weight_;
    result.crossing = crossing_score;
  } else {
    result.crossing = -1;
  }

  if (score_denominator != 0) {
    result.score = score_numerator / score_denominator;
    return result;
  } else {
    ROS_INFO("Warning: score function took nothing into account.");
    result.score = -1;
    return result;
  }
}

/*
 * Randomly select some viewpoints. The number of
 * viewpoints to select should be occlusion_check_limit_ / (# of landmarks)^2.
 */
void AutoCPDisplay::selectViewpoints(std::vector<Vector3>* viewpoints) {
  int num_landmarks = landmarks_.NumLandmarks() + 1; // +1 for center.
  num_landmarks *= num_landmarks;
  int num_viewpoints = static_cast<int>(
      static_cast<double>(occlusion_check_limit_->getInt()) / num_landmarks);
  if (num_viewpoints < 1) {
    num_viewpoints = 1;
  } else if (num_viewpoints > standard_viewpoints_.size()) {
    num_viewpoints = standard_viewpoints_.size();
  }
  for (int i = 0; i < num_viewpoints; i++) {
    int index = rand() % standard_viewpoints_.size();
    auto viewpoint = standard_viewpoints_[index];
    viewpoints->push_back(standard_viewpoints_[index]);
  }
}

/**
 * Choose a location for the camera. It searches through the list of standard
 * viewpoints and selects the highest scoring one. Returns true if a new
 * location was found.
 */
bool AutoCPDisplay::chooseCameraLocation(Point* location, float time_delta) {
  bool new_location_found = false;

  Score current_score = computeLocationScore(target_position_, target_focus_);

  // DEBUG
  //auto camera_position = getCameraPosition();
  //Score current_score = computeLocationScore(camera_position, target_focus_);
  //location->x = camera_position.x;
  //location->y = camera_position.y;
  //location->z = camera_position.z;
  //return true;

  Score best_score = current_score;
  Point best_position = target_position_;
  Point best_focus = target_focus_;

  std::vector<Point> test_points;
  std::vector<Point> test_foci;

  std::vector<Vector3> viewpoints;
  selectViewpoints(&viewpoints);

  std::vector<Landmark> landmarks;
  landmarks_.LandmarksVector(&landmarks);
  std::vector<Point> landmark_positions;
  for (const auto& landmark : landmarks) {
    landmark_positions.push_back(landmark.position);
  }
  Point center = landmarks_.Center();
  landmark_positions.push_back(center);

  std::vector<Score> scores;

  for (const auto& landmark_position : landmark_positions) {
    for (const auto& viewpoint_vector : viewpoints) {
      auto test_point = add(landmark_position, viewpoint_vector);
      test_points.push_back(test_point);
      test_foci.push_back(landmark_position);

      Score score = computeLocationScore(test_point, landmark_position);
      scores.push_back(score);

      if (score.score > best_score.score) {
        best_position = test_point;
        best_focus = landmark_position;
        best_score = score;
      }
    }
  }

  if (best_score.score > score_threshold_->getFloat() * current_score.score) {
    publishCandidateMarkers(test_points, test_foci, scores, time_delta);

    ROS_INFO("Moving to (%f, %f, %f), score=%f, prev=%f", best_position.x,
             best_position.y, best_position.z, best_score.score,
             current_score.score);
    ROS_INFO("viewpoints were: ");
    for (int i = 0; i < test_points.size(); i++) {
      auto test_point = test_points[i];
      auto vpscore = scores[i];
      ROS_INFO("  %f %f %f (v: %f, o: %f, z: %f, t: %f, c: %f = %f)",
               test_point.x, test_point.y, test_point.z, vpscore.visibility,
               vpscore.orthogonality, vpscore.zoom, vpscore.travel,
               vpscore.crossing, vpscore.score);
    }
    new_location_found = true;
    target_position_ = best_position;
    target_focus_ = best_focus;
  }

  return new_location_found;
}

// Utilities -------------------------------------------------------------------
/**
 * Convenience method to set the camera placement.
 */
void AutoCPDisplay::setCameraPlacement(
    const Point& location, const Point& focus,
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

  camera_placement->eye.point.x = location.x;
  camera_placement->eye.point.y = location.y;
  camera_placement->eye.point.z = location.z;

  camera_placement->focus.point.x = focus.x;
  camera_placement->focus.point.y = focus.y;
  camera_placement->focus.point.z = focus.z;

  camera_placement->up.vector.x = 0.0;
  camera_placement->up.vector.y = 0.0;
  camera_placement->up.vector.z = 1.0;
}

/**
 * Publish markers for each candidate viewpoint.
 */
void AutoCPDisplay::publishCandidateMarkers(
    const std::vector<Point>& test_points,
    const std::vector<Point>& test_foci,
    const std::vector<Score>& scores,
    float time_delta) {
  flushMarkers("candidates", num_prev_markers_);
  flushMarkers("candidate_scores", num_prev_markers_);

  for (int i = 0; i < test_points.size(); i++) {
    Marker marker;
    Marker text;
    makeCameraMarker(test_points[i], test_foci[i], scores[i], i, time_delta,
                     &marker);
    makeTextMarker(test_points[i], scores[i], i, &text);
    candidate_marker_pub_.publish(marker);
    candidate_marker_pub_.publish(text);
  }
  num_prev_markers_ = test_points.size();
}

/**
 * Delete all markers in the given namespace, with ids = 0, 1, ..., max_id.
 */
void AutoCPDisplay::flushMarkers(std::string ns, int max_id) {
  for (int i = 0; i < max_id; i++) {
    Marker marker;
    marker.header.frame_id = fixed_frame_.toStdString();
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = i;
    marker.action = Marker::DELETE;
    candidate_marker_pub_.publish(marker);
  }
}

/**
 * Creates an arrow marker pointing towards the focus point. It is colored green
 * if the score is 1, red if the score is 0, and something in between otherwise.
 */
void AutoCPDisplay::makeCameraMarker(const Point& position, const Point& focus,
                                     const Score& score, int id,
                                     float time_delta, Marker* marker) {
  Quaternion orientation;
  focusToOrientation(position, focus, &orientation);
  marker->header.frame_id = fixed_frame_.toStdString();
  marker->header.stamp = ros::Time::now();
  marker->ns = "candidates";
  marker->id = id;
  marker->type = Marker::ARROW;
  marker->action = Marker::ADD;
  marker->pose.position.x = position.x;
  marker->pose.position.y = position.y;
  marker->pose.position.z = position.z;
  marker->pose.orientation.x = orientation.x;
  marker->pose.orientation.y = orientation.y;
  marker->pose.orientation.z = orientation.z;
  marker->pose.orientation.w = orientation.w;
  marker->scale.x = 0.05;
  marker->scale.y = 0.05;
  marker->scale.z = 0.05;
  if (score.score < 0) {
    marker->color.r = 0;
    marker->color.g = 0;
    marker->color.b = 0;
  } else {
    marker->color.r = 1.0 - score.score;
    marker->color.g = score.score;
    marker->color.b = 0.0f;
  }
  marker->color.a = 1.0;
  marker->lifetime = ros::Duration();
}

void AutoCPDisplay::makeRayMarker(const Ogre::Ray& ray, int id, Marker* marker) {
  Quaternion orientation;
  auto origin = toPoint(ray.getOrigin());
  auto direction = toPoint(ray.getDirection());
  marker->header.frame_id = fixed_frame_.toStdString();
  marker->header.stamp = ros::Time::now();
  marker->ns = "rays";
  marker->id = id;
  marker->type = Marker::ARROW;
  marker->action = Marker::ADD;
  marker->points = std::vector<Point> {origin, add(origin, direction)};
  marker->scale.x = 0.05;
  marker->scale.y = 0.05;
  marker->scale.z = 0.05;
  marker->color.r = 1.0;
  marker->color.g = 0.0;
  marker->color.b = 0.0;
  marker->color.a = 1.0;
  marker->lifetime = ros::Duration();
}

void AutoCPDisplay::makePointMarker(const Point& point, int id, Marker* marker) {
  marker->header.frame_id = fixed_frame_.toStdString();
  marker->header.stamp = ros::Time::now();
  marker->ns = "points";
  marker->id = id;
  marker->type = Marker::SPHERE;
  marker->action = Marker::ADD;
  marker->pose.position = point;
  marker->scale.x = 0.1;
  marker->scale.y = 0.1;
  marker->scale.z = 0.1;
  marker->color.r = 0.0;
  marker->color.g = 0.0;
  marker->color.b = 1.0;
  marker->color.a = 1.0;
  marker->lifetime = ros::Duration();
}

void AutoCPDisplay::makeTextMarker(const Point& position, const Score& score,
                                   int id, Marker* marker) {
  marker->header.frame_id = fixed_frame_.toStdString();
  marker->header.stamp = ros::Time::now();
  marker->ns = "candidate_scores";
  marker->id = id;
  marker->type = Marker::TEXT_VIEW_FACING;
  marker->action = Marker::ADD;
  marker->pose.position.x = position.x;
  marker->pose.position.y = position.y;
  marker->pose.position.z = position.z - 0.1;
  marker->scale.z = 0.05;
  marker->color.r = 0;
  marker->color.g = 0.48;
  marker->color.b = 0.68;
  marker->color.a = 1.0;
  marker->lifetime = ros::Duration();
  marker->text = score.toString();
}

/**
 * Returns the current camera position.
 */
Point AutoCPDisplay::getCameraPosition() {
  Ogre::Vector3 camera_position = camera_->getPosition();
  return toPoint(camera_position);
}

/**
 * Interpolates between the start and end positions, subject to the camera speed
 * and size of this time step.
 */
Point AutoCPDisplay::interpolatePosition(const Point& start, const Point& end,
                                         float time_delta) {
  float max_distance = camera_speed_->getFloat() * time_delta;
  if (max_distance > distance(start, end)) {
    return end;
  } else {
    Vector3 v = vectorBetween(start, end);
    v = setLength(v, max_distance);
    Point next = add(start, v);
    return next;
  }
}

/**
 * Compute the projection of the vector onto the orthogonal plane or line
 * defined by the current control.
 */
Vector3 AutoCPDisplay::computeControlProjection(const ClickedControl& control,
                                                const Vector3& vector) {
  if (current_control_ == NULL) {
    ROS_INFO("Error: tried to compute projection of null control.");
  }
  Vector3 projection;
  projection.x = vector.x;
  projection.y = vector.y;
  projection.z = vector.z;
  if (current_control_->control == Control6Dof::X) {
    projection.x = 0;
  } else if (current_control_->control == Control6Dof::Y) {
    projection.y = 0;
  } else if (current_control_->control == Control6Dof::Z) {
    projection.z = 0;
  } else if (current_control_->control == Control6Dof::PITCH) {
    projection.x = 0;
    projection.z = 0;
  } else if (current_control_->control == Control6Dof::ROLL) {
    projection.y = 0;
    projection.z = 0;
  } else if (current_control_->control == Control6Dof::YAW) {
    projection.x = 0;
    projection.y = 0;
  } else {
    ROS_ERROR("Tried to compute orthogonal projection of an unknown control.");
    return projection;
  }
  return projection;
}

/**
 * Converts a position/focus point into an orientation from the position.
 */
void AutoCPDisplay::focusToOrientation(const Point& position,
                                       const Point& focus,
                                       Quaternion* orientation) {
  Ogre::Vector3 start(position.x, position.y, position.z);
  Ogre::Vector3 end(focus.x, focus.y, focus.z);
  Ogre::Vector3 diff = end - start;
  auto ogre_orientation = Ogre::Vector3::UNIT_X.getRotationTo(diff);
  orientation->x = ogre_orientation.x;
  orientation->y = ogre_orientation.y;
  orientation->z = ogre_orientation.z;
  orientation->w = ogre_orientation.w;
}

}  // namespace autocp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autocp::AutoCPDisplay, rviz::Display)
