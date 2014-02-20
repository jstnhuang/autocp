#include "autocp_display.h"
#include "utils.h"
#include <string>
#include <vector>

namespace autocp {
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
    0.25,
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

  updateWeights();
  
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

  current_control_ = NULL;

  show_fps_ = new rviz::BoolProperty(
    "Show FPS",
    false,
    "Whether or not to show the frames per second.",
    this,
    SLOT(updateCameraOptions()));

  standard_viewpoints_[0] = makeVector3(2, 0, 2);
  standard_viewpoints_[1] = makeVector3(2, -1, 2);
  standard_viewpoints_[2] = makeVector3(0, -1, 2);
  standard_viewpoints_[3] = makeVector3(-1, -1, 2);
  standard_viewpoints_[4] = makeVector3(-1, 0, 2);
  standard_viewpoints_[5] = makeVector3(-1, 1, 2);
  standard_viewpoints_[6] = makeVector3(0, 1, 2);
  standard_viewpoints_[7] = makeVector3(2, 1, 2);
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
  geometry_msgs::Point* origin
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
  for (auto obj : list.graspable_objects) {
    geometry_msgs::Point obj_location;
    for (auto point : obj.cluster.points) {
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
  geometry_msgs::Point world_position = feedback.pose.position;
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
  } catch (std::out_of_range e) {
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
  geometry_msgs::Point next_position = interpolatePosition(
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
void AutoCPDisplay::chooseCameraFocus(geometry_msgs::Point* focus) {
  std::vector<geometry_msgs::Point*> points = {
    &head_focus_point_,
    &left_gripper_origin_,
    &right_gripper_origin_
  };

  geometry_msgs::Point center = landmarks_.Center();
  *focus = center;
}

/**
 * Computes a score for the location.
 */
float AutoCPDisplay::computeLocationScore(
    const geometry_msgs::Point& location) {
  float score_numerator = 0;
  float score_denominator = 0;

  geometry_msgs::Point control_location;
  if (current_control_ != NULL) {
    control_location = current_control_->world_position;
  }

  // Occlusion score.
  int num_visible = 0;
  int num_points = 0;

  // Occlusion score for current control.
  if (current_control_ != NULL) {
    float occlusion_distance = occlusionDistanceFrom(control_location,
      location, camera_focus_);
    if (occlusion_distance < 0.25) {
      num_visible++;
    }
    num_points++;
  }
  
  // Occlusion score for segmented objects.
  for (auto point : segmented_object_positions_) {
    float occlusion_distance = occlusionDistanceFrom(point, location,
      camera_focus_);
    if (occlusion_distance < 0.25) {
      num_visible++;
    }
    num_points++;
  }

  if (num_points != 0) {
    score_numerator += stay_visible_weight_->getFloat()
      * num_visible / num_points;
    score_denominator += stay_visible_weight_->getFloat();
  }

  // Movement moderation.
  float movement_moderation_score =
    1 - logisticDistance(distance(getCameraPosition(), location), 1);
  score_numerator +=
    stay_in_place_weight_->getFloat() * movement_moderation_score;
  score_denominator += stay_in_place_weight_->getFloat();

  // Orthogonality score.
  float ortho_score = 0;
  if (current_control_ != NULL) {
    geometry_msgs::Vector3 location_vector = vectorBetween(
      control_location, location);
    geometry_msgs::Vector3 projection = computeControlProjection(
      *current_control_,
      location_vector);
    ortho_score = fabs(cosineAngle(location_vector, projection));
    score_numerator += be_orthogonal_weight_->getFloat() * ortho_score;
    score_denominator += be_orthogonal_weight_->getFloat();
  }

  if (score_denominator != 0) {
    return score_numerator / score_denominator;
  } else {
    return 0;
  }
}

/**
 * Choose a location for the camera. It searches through the list of standard
 * viewpoints and selects the highest scoring one. Returns true if a new
 * location was found.
 */
bool AutoCPDisplay::chooseCameraLocation(geometry_msgs::Point* location) {
  geometry_msgs::Point camera_position = getCameraPosition();
  bool new_location_found = false;

  float current_score = computeLocationScore(camera_position);
  float best_score = current_score;
  geometry_msgs::Point best_location = camera_position;

  // If the user is using a control, precompute the quadrant the camera is in
  // relative to the control. The camera must stay in this quadrant.
  geometry_msgs::Point control_position;
  int x_sign = 0;
  int y_sign = 0;
  int z_sign = 0;
  if (current_control_ != NULL) {
    control_position = current_control_->world_position;
    x_sign = sign(camera_position.x - control_position.x);
    y_sign = sign(camera_position.y - control_position.y);
    z_sign = sign(camera_position.z - control_position.z);
  }

  for (auto test_vector : standard_viewpoints_) {
    geometry_msgs::Point test_point = toPoint(test_vector);

    // Constraints
    // Never go below the ground plane
    if (test_point.z < 0) {
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
        continue;
      }

      // If you're using an x control, don't cross the y=0 plane
      // If you're using a y control, don't cross the x=0 plane
      if (current_control_->control == Control6Dof::X) {
        if (test_y_sign != y_sign) {
          continue;
        }
      } else if (current_control_->control == Control6Dof::Y) {
        if (test_x_sign != x_sign) {
          continue;
        }
      }
    }

    float score = computeLocationScore(test_point);
    if (score > score_threshold_->getFloat() * best_score) {
      best_location = test_point;
      best_score = score;
      new_location_found = true;
    }
  }

  if (new_location_found) {
    ROS_INFO("Moving to (%f, %f, %f), score=%f, prev=%f",
      best_location.x, best_location.y, best_location.z,
      best_score, current_score);
  }
  target_position_ = best_location;
  return new_location_found;
}

// Utilities -------------------------------------------------------------------
/**
 * Convenience method to set the camera placement.
 */
void AutoCPDisplay::setCameraPlacement(
  const geometry_msgs::Point& location,
  const geometry_msgs::Point& focus,
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
geometry_msgs::Point AutoCPDisplay::getCameraPosition() {
  Ogre::Vector3 camera_position = camera_->getPosition();
  return toPoint(camera_position);
}

/**
 * Interpolates between the start and end positions, subject to the camera speed
 * and size of this time step.
 */
geometry_msgs::Point AutoCPDisplay::interpolatePosition(
    const geometry_msgs::Point& start, const geometry_msgs::Point& end,
    float time_delta) {
  float max_distance = camera_speed_->getFloat() * time_delta;
  if (max_distance > distance(start, end)) {
    return end;
  } else {
    geometry_msgs::Vector3 v = vectorBetween(start, end);
    v = setLength(v, max_distance);
    geometry_msgs::Point next = add(start, v);
    return next;
  }
}

/**
 * Returns the x, y coordinates on the viewport of the given point. The origin
 * is in the top left corner.
 */
void AutoCPDisplay::projectWorldToViewport(
    const geometry_msgs::Point& point,
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

/**
 * Compute the distance between the given point and the closest visible point on * the ray pointed towards that point.
 */
float AutoCPDisplay::occlusionDistance(const geometry_msgs::Point& point) {
  int screen_x;
  int screen_y;
  projectWorldToViewport(point, &screen_x, &screen_y);
  Ogre::Vector3 occluding_vector;
  bool success = context_->getSelectionManager()->get3DPoint(
    viewport_,
    screen_x,
    screen_y,
    occluding_vector);
  geometry_msgs::Point occluding_point = toPoint(occluding_vector);
  if (success) {
    return distance(occluding_point, point);
  } else {
    return 0;
  }
}

/**
 * Computes the amount of occlusion from the given point, given the desired
 * camera position and camera focus. Returns the distance in meters.
 */
float AutoCPDisplay::occlusionDistanceFrom(
    const geometry_msgs::Point& point,
    const geometry_msgs::Point& camera_position,
    const geometry_msgs::Point& camera_focus) {
  Ogre::Vector3 old_position = camera_->getPosition();
  Ogre::Vector3 old_direction = camera_->getDirection();
  camera_->setPosition(camera_position.x, camera_position.y, camera_position.z);
  camera_->lookAt(camera_focus.x, camera_focus.y, camera_focus.z);
  float occlusion = occlusionDistance(point);
  camera_->setPosition(old_position);
  camera_->setDirection(old_direction);
  return occlusion;
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
