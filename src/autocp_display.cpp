#include "autocp_display.h"
#include "utils.h"
#include <string>
#include <vector>
#include <sys/time.h>

namespace autocp {
using geometry_msgs::Point;

/**
 * Constructor. Hooks up the display properties.
 */
AutoCPDisplay::AutoCPDisplay(): root_nh_("") {
  topic_prop_ = new rviz::RosTopicProperty(
    "Command topic",
    "/rviz/camera_placement",
    QString::fromStdString(
      ros::message_traits::datatype<view_controller_msgs::CameraPlacement>()),
    "Topic on which to send out camera placement messages.",
    this,
    SLOT(updateTopic()));
  
  // Landmark weights.
  gripper_weight_= new rviz::FloatProperty(
    "Gripper focus weight",
    0.25,
    "How much weight to assign to the grippers' locations.",
    this,
    SLOT(updateWeights()));
  gripper_weight_->setMin(0);
  gripper_weight_->setMax(1);

  head_focus_weight_= new rviz::FloatProperty(
    "Head focus point weight",
    0.25,
    "How much weight to assign to the location the robot is looking.",
    this,
    SLOT(updateWeights()));
  head_focus_weight_->setMin(0);
  head_focus_weight_->setMax(1);

  segmented_object_weight_ = new rviz::FloatProperty(
    "Segmented object weight",
    0.25,
    "How much weight to assign to the locations of segmented objects",
    this,
    SLOT(updateWeights()));
  segmented_object_weight_->setMin(0);
  segmented_object_weight_->setMax(1);

  current_marker_weight_ = new rviz::FloatProperty(
    "Current marker",
    0,
    "How much weight to assign to the location of the current marker.",
    this,
    SLOT(updateWeights()));
  current_marker_weight_->setMin(0);
  current_marker_weight_->setMax(1);

  // Weights on location.
  stay_in_place_weight_ = new rviz::FloatProperty(
    "Movement moderation weight",
    0.1,
    "How much weight to points close to the current location.",
    this,
    SLOT(updateWeights()));
  stay_in_place_weight_->setMin(0);
  stay_in_place_weight_->setMax(1);

  be_orthogonal_weight_ = new rviz::FloatProperty(
    "Marker orthogonality weight",
    0.6,
    "How much weight to assign to points orthogonal to the current marker.",
    this,
    SLOT(updateWeights()));
  be_orthogonal_weight_->setMin(0);
  be_orthogonal_weight_->setMax(1);

  stay_visible_weight_ = new rviz::FloatProperty(
    "Marker visibility weight",
    0.2,
    "How much weight to assign to points where the current marker is visible.",
    this,
    SLOT(updateWeights()));
  stay_visible_weight_->setMin(0);
  stay_visible_weight_->setMax(1);

  zoom_weight_ = new rviz::FloatProperty(
    "Zoom weight",
    0.1,
    "How much weight to assign to being close to landmarks.",
    this,
    SLOT(updateWeights()));
  zoom_weight_->setMin(0);
  zoom_weight_->setMax(1);

  // Other properties.
  score_threshold_ = new rviz::FloatProperty(
    "Score improvement threshold",
    1.05,
    "Factor by which the score must improve before adjusting the position.",
    this,
    SLOT(updateWeights()));
  score_threshold_->setMin(0);
  score_threshold_->setMax(30);

  camera_speed_ = new rviz::FloatProperty(
    "Camera speed",
    3,
    "How many meters per second the camera can move.",
    this,
    SLOT(updateCameraOptions()));
  camera_speed_->setMin(0);
  camera_speed_->setMax(10);

  show_fps_ = new rviz::BoolProperty(
    "Show FPS",
    false,
    "Whether or not to show the frames per second.",
    this,
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
  Display:: onInitialize();
  updateTopic();
  updateWeights();

  point_head_subscriber_ = root_nh_.subscribe(
    "head_traj_controller/point_head_action/goal",
    5,
    &AutoCPDisplay::pointHeadCallback,
    this);

  marker_subscriber_ = root_nh_.subscribe(
    "/pr2_marker_control_transparent/feedback",
    5,
    &AutoCPDisplay::markerCallback,
    this);

  object_segmentation_subscriber_ = root_nh_.subscribe(
    "/interactive_object_recognition_result",
    5,
    &AutoCPDisplay::objectSegmentationCallback,
    this);

  vm_ = static_cast<rviz::VisualizationManager*>(context_);
  camera_ = vm_->getRenderPanel()->getCamera();
  viewport_ = camera_->getViewport();
  target_position_ = getCameraPosition();
  current_control_ = NULL;

  initializeStandardViewpoints();
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
  const int num_scales = 4;
  float scales[num_scales] = {2, 3, 4};
  for (int i=0; i<num_standard; i++) {
    auto viewpoint = standard_viewpoints_[i];
    for (int j=0; j<num_scales; j++) {
      standard_viewpoints_.push_back(scale(viewpoint, scales[j]));
    }
  }
}

// Parameter update handlers ---------------------------------------------------
/**
 * Set the topic to publish camera placement commands to.
 */
void AutoCPDisplay::updateTopic() {
  camera_placement_publisher_ =
    root_nh_.advertise<view_controller_msgs::CameraPlacement>(
      topic_prop_->getStdString(),
      5);
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
  landmarks_.UpdateHeadFocusWeight(head_focus_weight_->getFloat());
  landmarks_.UpdateCurrentMarkerWeight(current_marker_weight_->getFloat());
  landmarks_.UpdateSegmentedObjectWeight(segmented_object_weight_->getFloat());
}

// Sensing ---------------------------------------------------------------------
/**
 * Update the points of interest.
 */
void AutoCPDisplay::sense() {
  getTransformOrigin("/l_wrist_roll_link", &left_gripper_origin_);
  getTransformOrigin("/r_wrist_roll_link", &right_gripper_origin_);
  landmarks_.UpdateLeftGripper(&left_gripper_origin_);
  landmarks_.UpdateRightGripper(&right_gripper_origin_);
}

/**
 * Get the origin of the given transform.
 */
void AutoCPDisplay::getTransformOrigin(
  std::string frame,
  Point* origin
) {
  ros::Duration timeout(5);
  tf_listener_.waitForTransform(
    fixed_frame_.toStdString(), frame, ros::Time(0), timeout);
  tf::StampedTransform transform;
  tf_listener_.lookupTransform(
    fixed_frame_.toStdString(), frame, ros::Time(0), transform);
  tf::Vector3 transform_origin = transform.getOrigin();

  origin->x = transform_origin.x();
  origin->y = transform_origin.y();
  origin->z = transform_origin.z();
}

/**
 * Get the target point for the head.
 */
void AutoCPDisplay::pointHeadCallback(
  const pr2_controllers_msgs::PointHeadActionGoal& action_goal
) {
  head_focus_point_ = action_goal.goal.target.point;
  landmarks_.UpdateHeadFocus(&head_focus_point_);
}

/**
 * When an object is segmented, compute the average point in the point cloud and
 * update the segmented_object_positions_ vector.
 */
void AutoCPDisplay::objectSegmentationCallback(
  const manipulation_msgs::GraspableObjectList& list
) {
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
  const visualization_msgs::InteractiveMarkerFeedback& feedback
) {
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

    current_control_ = new ClickedControl {
      marker_name,
      control,
      feedback.pose,
      world_position
    };
    landmarks_.UpdateCurrentMarker(&world_position);
  } catch (const std::out_of_range& e) {
    ROS_INFO(
      "Unknown control %s for marker %s",
      control_name.c_str(),
      marker_name.c_str());
  }
}

// Camera placement logic ------------------------------------------------------
/**
 * Main loop that alternates between sensing and placing the camera.
 */
void AutoCPDisplay::update(float wall_dt, float ros_dt) {
  sense();
  chooseCameraPlacement(wall_dt);

  if (show_fps_->getBool()) {
    ROS_INFO("FPS: %f", 1 / wall_dt);
  }

  delete current_control_;
  current_control_ = NULL;
  landmarks_.UpdateCurrentMarker(NULL);
}

/**
 * Get the final focus point and location for the camera, then push the camera
 * placement.
 */
void AutoCPDisplay::chooseCameraPlacement(float time_delta) {
  chooseCameraFocus(&camera_focus_);
  chooseCameraLocation(&target_position_);
  Point next_position = interpolatePosition(
    getCameraPosition(), target_position_, time_delta);

  view_controller_msgs::CameraPlacement camera_placement;
  setCameraPlacement(
    next_position, camera_focus_,
    ros::Duration(time_delta),
    &camera_placement);

  camera_placement_publisher_.publish(camera_placement);
}

/**
 * Choose a final focus point for the camera. The final focus point is the
 * weighted mean of the current focus points.
 */
void AutoCPDisplay::chooseCameraFocus(Point* focus) {
  Point center = landmarks_.Center();
  *focus = center;
}

/**
 * Visibility score. Returns the weighted average visibility of all the
 * landmarks.
 */
float AutoCPDisplay::visibilityScore(const Point& location) {
  auto occlusion_metric = [&] (const Point& point) -> float {
    if (isVisibleFrom(point, location, camera_focus_)) {
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
float AutoCPDisplay::orthogonalityScore(const Point& location,
    const Point& control_location) {
  if (current_control_ == NULL) {
    ROS_INFO("Warning: tried to get orthogonality score without a control.");
    return 0;
  }
  geometry_msgs::Vector3 location_vector = vectorBetween(
    control_location, location);
  geometry_msgs::Vector3 projection = computeControlProjection(
    *current_control_,
    location_vector);
  return fabs(cosineAngle(location_vector, projection));
}

/**
 * Zoom score. Returns 1 if we are very close, 0 if we are far away, and a
 * linearly interpolated value if we are in between. Close and far are defined
 * by MIN_DISTANCE and MAX_DISTANCE.
 */
float AutoCPDisplay::zoomScore(const Point& location) {
  auto zoom_metric = [&] (const Point& point) -> float {
    float dist = distance(point, location);
    if (dist < MIN_DISTANCE || dist > MAX_DISTANCE) {
      return 0;
    } else {
      float slope = 1.0 / (MAX_DISTANCE - MIN_DISTANCE);
      return -slope * dist + (1 + MIN_DISTANCE * slope);
    }
  };
  return landmarks_.ComputeMetric(zoom_metric);
}

/**
 * Smoothness score. Returns 1 if the camera has not moved much, 0 if it has
 * moved a lot, and a number in between mediated by the logisticDistance
 * function.
 */
float AutoCPDisplay::smoothnessScore(const Point& location) {
  Ogre::Camera camera ("test", vm_->getSceneManager());
  camera.setPosition(location.x, location.y, location.z);
  camera.lookAt(camera_focus_.x, camera_focus_.y, camera_focus_.z);
  auto sim_orientation = camera.getOrientation();
  auto our_orientation = camera_->getOrientation();
  float x_diff = target_position_.x - location.x;
  float y_diff = target_position_.y - location.y;
  float z_diff = target_position_.z - location.z;
  float orientation_w_diff = sim_orientation.w - our_orientation.w;
  float orientation_x_diff = sim_orientation.x - our_orientation.x;
  float orientation_y_diff = sim_orientation.y - our_orientation.y;
  float orientation_z_diff = sim_orientation.z - our_orientation.z;
  float distance = sqrt(
    x_diff * x_diff
    + y_diff * y_diff
    + z_diff * z_diff
    + orientation_w_diff * orientation_w_diff
    + orientation_x_diff * orientation_x_diff
    + orientation_y_diff * orientation_y_diff
    + orientation_z_diff * orientation_z_diff
  );
  return 1 - logisticDistance(distance, 1);
}

/**
 * Computes a score for the location.
 */
Score AutoCPDisplay::computeLocationScore(
    const Point& location) {
  Score result;
  float score_numerator = 0;
  float score_denominator = 0;
  
  // Visibility score.
  float visibility_score = visibilityScore(location);
  float visibility_weight = stay_visible_weight_->getFloat();
  score_numerator += visibility_weight * visibility_score;
  score_denominator += visibility_weight;
  result.visibility = visibility_score;

  // Orthogonality score.
  if (current_control_ != NULL) {
    Point control_location = current_control_->world_position;
    float ortho_score = orthogonalityScore(location, control_location);
    float ortho_weight = be_orthogonal_weight_->getFloat();
    score_numerator += ortho_weight * ortho_score;
    score_denominator += ortho_weight;
    result.orthogonality = ortho_score;
  } else {
    result.orthogonality = -1;
  }

  // Zoom score.
  float zoom_score = zoomScore(location);
  float zoom_weight = zoom_weight_->getFloat();
  score_numerator += zoom_weight * zoom_score;
  score_denominator += zoom_weight;
  result.zoom = zoom_score;

  // Movement moderation.
  float smoothness_score = smoothnessScore(location);
  float smoothness_weight = stay_in_place_weight_->getFloat();
  score_numerator += smoothness_weight * smoothness_score;
  score_denominator += smoothness_weight;
  result.smoothness = smoothness_score;

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
 * viewpoints to select should be OCCLUSION_CHECK_LIMIT / # of landmarks.
 */
void AutoCPDisplay::selectViewpoints(
    std::vector<geometry_msgs::Vector3>* viewpoints) {
  std::vector<Landmark> landmarks;
  landmarks_.LandmarksVector(&landmarks);
  int num_landmarks = landmarks.size(); 
  int num_viewpoints = static_cast<int>(
    static_cast<double>(OCCLUSION_CHECK_LIMIT) / num_landmarks);
  if (num_viewpoints < 2) {
    ROS_INFO("Sampling less than two viewpoints per frame. "
      "There may be too many landmarks.");
    num_viewpoints = 2;
  }
  for (int i=0; i<num_viewpoints; i++) {
    int index = rand() % standard_viewpoints_.size();
    viewpoints->push_back(standard_viewpoints_[index]);
  }
}

/**
 * Choose a location for the camera. It searches through the list of standard
 * viewpoints and selects the highest scoring one. Returns true if a new
 * location was found.
 */
bool AutoCPDisplay::chooseCameraLocation(Point* location) {
//  Point camera_position = getCameraPosition();
  bool new_location_found = false;

  Score current_score = computeLocationScore(target_position_);
  Score best_score = current_score;
  Point best_location = target_position_;

  // If the user is using a control, precompute the quadrant the camera is in
  // relative to the control. The camera must stay in this quadrant.
  Point control_position;
  int x_sign = 0;
  int y_sign = 0;
  if (current_control_ != NULL) {
    control_position = current_control_->world_position;
    x_sign = sign(target_position_.x - control_position.x);
    y_sign = sign(target_position_.y - control_position.y);
  }

  std::vector<geometry_msgs::Vector3> viewpoints;
  std::vector<Score> scores;
  selectViewpoints(&viewpoints);
  Score null_score;
  null_score.visibility = -1;
  null_score.orthogonality = -1;
  null_score.zoom = -1;
  null_score.smoothness = -1;
  null_score.score = -1;
  for (const auto& test_vector : viewpoints) {
    Point test_point = add(camera_focus_, test_vector);

    // Constraints
    // Never go below the ground plane
    if (test_point.z < 0) {
      scores.push_back(null_score);
      continue;
    }

    // If the user is using a control, then we automatically place some
    // additional constraints on the viewpoint.
    if (current_control_ != NULL) {
      int test_x_sign = sign(test_point.x - control_position.x);
      int test_y_sign = sign(test_point.y - control_position.y);
      int test_z_sign = sign(test_point.z - control_position.z);
      
      // Never go below the control.
      if (test_z_sign < 0) {
        scores.push_back(null_score);
        continue;
      }

      // If you're using an x control, don't cross the y=0 plane
      // If you're using a y control, don't cross the x=0 plane
      if (current_control_->control == Control6Dof::X) {
        if (test_y_sign != y_sign) {
          scores.push_back(null_score);
          continue;
        }
      } else if (current_control_->control == Control6Dof::Y) {
        if (test_x_sign != x_sign) {
          scores.push_back(null_score);
          continue;
        }
      }
    }

    Score score = computeLocationScore(test_point);
    scores.push_back(score);
    if (score.score > best_score.score) {
      best_location = test_point;
      best_score = score;
    }
  }

  if (best_score.score > score_threshold_->getFloat() * current_score.score) {
    ROS_INFO("Moving to (%f, %f, %f), score=%f, prev=%f",
      best_location.x, best_location.y, best_location.z,
      best_score.score, current_score.score);
    ROS_INFO("viewpoints were: ");
    ROS_INFO("  %f %f %f (v: %f, o: %f, z: %f, s: %f = %f)",
      target_position_.x, target_position_.y, target_position_.z,
      current_score.visibility, current_score.orthogonality, current_score.zoom,
      current_score.smoothness, current_score.score);
    for (int i=0; i<viewpoints.size(); i++) {
      auto viewpoint = add(camera_focus_, viewpoints[i]);
      auto vpscore = scores[i];
      ROS_INFO("  %f %f %f (v: %f, o: %f, z: %f, s: %f = %f)",
        viewpoint.x, viewpoint.y, viewpoint.z,
        vpscore.visibility, vpscore.orthogonality, vpscore.zoom,
        vpscore.smoothness, vpscore.score);
    }
    new_location_found = true;
    target_position_ = best_location;
  }

  return new_location_found;
}

// Utilities -------------------------------------------------------------------
/**
 * Convenience method to set the camera placement.
 */
void AutoCPDisplay::setCameraPlacement(
  const Point& location,
  const Point& focus,
  const ros::Duration& time_from_start,
  view_controller_msgs::CameraPlacement* camera_placement
) {
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
Point AutoCPDisplay::interpolatePosition(
    const Point& start, const Point& end,
    float time_delta) {
  float max_distance = camera_speed_->getFloat() * time_delta;
  if (max_distance > distance(start, end)) {
    return end;
  } else {
    geometry_msgs::Vector3 v = vectorBetween(start, end);
    v = setLength(v, max_distance);
    Point next = add(start, v);
    return next;
  }
}

/**
 * Returns the x, y coordinates on the viewport of the given point. The origin
 * is in the top left corner.
 */
void AutoCPDisplay::projectWorldToViewport(
    const Point& point,
    int* screen_x,
    int* screen_y) {
  // This projection returns x and y in the range of [-1, 1]. The (-1, -1) point
  // is in the bottom left corner.
  Ogre::Vector4 point4(point.x, point.y, point.z, 1);
  Ogre::Vector4 projected =
    camera_->getProjectionMatrix() * camera_->getViewMatrix() * point4;
  projected = projected / projected.w;

  // Using the current viewport.
  *screen_x = static_cast<int>(
    round(viewport_->getActualWidth() * (projected.x + 1) / 2));
  *screen_y = static_cast<int>(
    round(viewport_->getActualHeight() * (1 - projected.y) / 2));
}

/*
 * Returns true if the given (x, y) coordinates are visible on screen.
 */
bool AutoCPDisplay::isOnScreen(int screen_x, int screen_y) {
  if (screen_x < 0 || screen_y < 0 || screen_x > viewport_->getActualWidth()
      || screen_y > viewport_->getActualHeight()) {
    return false;
  } else {
    return true;
  }
}

/*
 * Returns true if the given point is visible from the current camera pose.
 */
bool AutoCPDisplay::isVisible(const Point& point) {
  int screen_x;
  int screen_y;
  projectWorldToViewport(point, &screen_x, &screen_y);
  if (isOnScreen(screen_x, screen_y)) {
    float occlusion_distance = 0;
    Ogre::Vector3 occluding_vector;
    bool success = context_->getSelectionManager()->get3DPoint(
      viewport_,
      screen_x,
      screen_y,
      occluding_vector);
    Point occluding_point = toPoint(occluding_vector);
    if (success) {
      occlusion_distance = distance(occluding_point, point);
    }
    if (occlusion_distance < OCCLUSION_THRESHOLD) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/*
 * Returns true if the given point is visible from the given camera position and
 * focus point.
 */
bool AutoCPDisplay::isVisibleFrom(const Point& point,
    const Point& camera_position,
    const Point& camera_focus) {
  Ogre::Vector3 old_position = camera_->getPosition();
  Ogre::Vector3 old_direction = camera_->getDirection();
  camera_->setPosition(camera_position.x, camera_position.y, camera_position.z);
  camera_->lookAt(camera_focus.x, camera_focus.y, camera_focus.z);
  bool result = isVisible(point);
  camera_->setPosition(old_position);
  camera_->setDirection(old_direction);
  return result;
}

/**
 * Compute the projection of the vector onto the orthogonal plane or line
 * defined by the current control.
 */
geometry_msgs::Vector3 AutoCPDisplay::computeControlProjection(
    const ClickedControl& control,
    const geometry_msgs::Vector3& vector) {
  geometry_msgs::Vector3 projection;
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

}  // namespace autocp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autocp::AutoCPDisplay, rviz::Display)
