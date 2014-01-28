#include "autocp_display.h"
#include "utils.h"
#include <string>
#include <vector>
#include <random>

namespace autocp {
/**
 * Constructor. Hooks up the display properties.
 */
AutoCPDisplay::AutoCPDisplay(): root_nh_(""), distribution_(0.0, 1) {
  topic_prop_ = new rviz::RosTopicProperty(
    "Command topic",
    "/rviz/camera_placement",
    QString::fromStdString(
      ros::message_traits::datatype<view_controller_msgs::CameraPlacement>()),
    "Topic on which to send out camera placement messages.",
    this,
    SLOT(updateTopic()));
  
  gripper_weight_property_ = new rviz::FloatProperty(
    "Gripper focus weight",
    1.0,
    "How much weight to assign to the grippers' locations.",
    this,
    SLOT(updateWeights()));
  gripper_weight_property_->setMin(0);
  gripper_weight_property_->setMax(100);
  point_head_weight_property_ = new rviz::FloatProperty(
    "Head focus point weight",
    1.0,
    "How much weight to assign to the location the robot is looking.",
    this,
    SLOT(updateWeights()));
  point_head_weight_property_->setMin(0);
  point_head_weight_property_->setMax(100);

  // Weights on location.
  stay_in_place_weight_ = new rviz::FloatProperty(
    "Movement moderation weight",
    0.15,
    "How much weight to points close to the current location.",
    this,
    SLOT(updateWeights()));
  stay_in_place_weight_->setMin(0);
  stay_in_place_weight_->setMax(1);
  be_orthogonal_weight_ = new rviz::FloatProperty(
    "Marker orthogonality weight",
    0.25,
    "How much weight to assign to points orthogonal to the current marker.",
    this,
    SLOT(updateWeights()));
  be_orthogonal_weight_->setMin(0);
  be_orthogonal_weight_->setMax(1);
  stay_visible_weight_ = new rviz::FloatProperty(
    "Marker visibility weight",
    0.6,
    "How much weight to assign to points where the current marker is visible.",
    this,
    SLOT(updateWeights()));
  stay_visible_weight_->setMin(0);
  stay_visible_weight_->setMax(1);
  movement_time_ = new rviz::FloatProperty(
    "Movement timer",
    1,
    "How many seconds to wait to move the camera again.",
    this,
    SLOT(updateMovementTime()));
  movement_time_->setMin(0);
  movement_time_->setMax(30);

  updateWeights();
  updateMovementTime();
  current_control_ = NULL;

  l_gripper_cp_enabled_ = new rviz::BoolProperty(
    "Move camera with left gripper",
    true,
    "Whether or not to move the camera when using the left gripper.",
    this,
    SLOT(updateCameraOptions()));
  r_gripper_cp_enabled_ = new rviz::BoolProperty(
    "Move camera with right gripper",
    true,
    "Whether or not to move the camera when using the right gripper.",
    this,
    SLOT(updateCameraOptions()));
  point_head_cp_enabled_ = new rviz::BoolProperty(
    "Move camera with head target point",
    true,
    "Whether or not to move the camera when using the head target point.",
    this,
    SLOT(updateCameraOptions()));
  l_posture_cp_enabled_ = new rviz::BoolProperty(
    "Move camera with left shoulder control",
    true,
    "Whether or not to move the camera when using the left shoulder control.",
    this,
    SLOT(updateCameraOptions()));
  r_posture_cp_enabled_ = new rviz::BoolProperty(
    "Move camera with right shoulder control",
    true,
    "Whether or not to move the camera when using the right shoulder control.",
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

  vm_ = static_cast<rviz::VisualizationManager*>(context_);
  camera_ = vm_->getRenderPanel()->getCamera();
  viewport_ = camera_->getViewport();
  generator_.seed(std::time(0));
  is_moving_ = false;
}

/**
 * Main loop that alternates between sensing and placing the camera.
 */
void AutoCPDisplay::update(float wall_dt, float ros_dt) {
  time_until_move_ -= wall_dt;
  time_until_move_complete_ -= wall_dt;
  sense();
  chooseCameraPlacement(wall_dt);
  if (time_until_move_ < 0) {
    updateMovementTime();
  }
}

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
 * Update the points of interest.
 */
void AutoCPDisplay::sense() {
  getTransformOrigin("/l_wrist_roll_link", &left_gripper_origin_);
  getTransformOrigin("/r_wrist_roll_link", &right_gripper_origin_);
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
}

/**
 * Update the weight vector, and normalize so that the max is 1.
 * TODO(jstn): normalize location weights. Rename weights_ to be focus_weights_.
 */
void AutoCPDisplay::updateWeights() {
  weights_ = {
    point_head_weight_property_->getFloat(),
    gripper_weight_property_->getFloat(),
    gripper_weight_property_->getFloat()
  };
  float max = 0;
  for (float weight : weights_) {
    if (weight > max) {
      max = weight;
    }
  }
  for (unsigned i = 0; i < weights_.size(); i++) {
    weights_[i] = weights_[i] / max;
  }
}

void AutoCPDisplay::updateMovementTime() {
  time_until_move_ = movement_time_->getFloat();
}

/**
 * Get interactive marker locations.
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
    if (marker_name == "head_point_goal" && point_head_cp_enabled_->getBool()) {
      control = POINT_HEAD_CONTROLS.at(control_name);
    } else if (marker_name == "l_gripper_control"
      && l_gripper_cp_enabled_->getBool()) {
      control = GRIPPER_CONTROLS.at(control_name);
      world_position = left_gripper_origin_;
    } else if (marker_name == "r_gripper_control"
      && r_gripper_cp_enabled_->getBool()) {
      control = GRIPPER_CONTROLS.at(control_name);
      world_position = right_gripper_origin_;
    } else if (marker_name == "l_posture_control"
      && l_posture_cp_enabled_->getBool()) {
      control = Control6Dof::ROLL;
    } else if (marker_name == "r_posture_control"
      && r_posture_cp_enabled_->getBool()) {
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
  } catch (std::out_of_range e) {
    ROS_INFO(
      "Unknown control %s for marker %s",
      control_name.c_str(),
      marker_name.c_str());
  }
}

/**
 * Get the final focus point and location for the camera, then push the camera
 * placement.
 */
void AutoCPDisplay::chooseCameraPlacement(float time_delta) {
  chooseCameraFocus(&camera_focus_);

  geometry_msgs::Point updated_position;
  float placement_time = time_delta;

  if (is_moving_) {
    if (current_control_ != NULL) {
      updated_position = camera_position_;
    } else {
      updated_position = getCameraPosition();
    }
    if (time_until_move_complete_ < 0) {
      is_moving_ = false;
    }
  } else {
    updated_position = getCameraPosition();
    placement_time = time_delta;
    if (time_until_move_ < 0) {
      bool new_position_found = chooseCameraLocation(&camera_position_);
      time_until_move_complete_ = movement_time_->getFloat();
      is_moving_ = true;
    }

  }

  view_controller_msgs::CameraPlacement camera_placement;
  setCameraPlacement(
    updated_position, camera_focus_,
    ros::Duration(placement_time),
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

  float mean_x = 0;
  float mean_y = 0;
  float mean_z = 0;
  int num_points = points.size();
  for (int i = 0; i < num_points; i++) {
    geometry_msgs::Point* point = points[i];
    float weight = weights_[i];
    mean_x += weight * point->x;
    mean_y += weight * point->y;
    mean_z += weight * point->z;
  }

  mean_x /= num_points;
  mean_y /= num_points;
  mean_z /= num_points;

  focus->x = mean_x;
  focus->y = mean_y;
  focus->z = mean_z;
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

/**
 * Returns the current camera position.
 */
geometry_msgs::Point AutoCPDisplay::getCameraPosition() {
  Ogre::Vector3 camera_position = camera_->getPosition();
  return toPoint(camera_position);
}

/**
 * Computes a score for the location.
 */
float AutoCPDisplay::computeLocationScore(
    const geometry_msgs::Point& location) {
  if (current_control_ == NULL) {
    return -1;
  }
  geometry_msgs::Point control_location = current_control_->world_position;

  // Occlusion score.
  float occlusion_distance = occlusionDistanceFrom(control_location,
    location, camera_focus_);
  float occlusion = logisticDistance(occlusion_distance);
  float occlusion_score = 1 - occlusion;

  // Distance score.
  geometry_msgs::Point camera_position = getCameraPosition();
  float distance_score =
    1 - logisticDistance(distance(camera_position, location));

  // Orthogonality score.
  geometry_msgs::Vector3 location_vector = vectorBetween(
    control_location, location);
  geometry_msgs::Vector3 projection = computeControlProjection(
    *current_control_,
    location_vector);
  float ortho_score = fabs(cosineAngle(location_vector, projection));

  return (
    stay_in_place_weight_->getFloat() * distance_score
    + be_orthogonal_weight_->getFloat() * ortho_score
    + stay_visible_weight_->getFloat() * occlusion_score);
}

/**
 * Randomly perturb the given vector while maintaining the length.
 */
geometry_msgs::Vector3 AutoCPDisplay::getRandomPerturbation(
    const geometry_msgs::Vector3& vector) {
  float x_diff = static_cast<float>(distribution_(generator_));
  float y_diff = static_cast<float>(distribution_(generator_));
  float z_diff = static_cast<float>(distribution_(generator_));
  geometry_msgs::Vector3 result;
  result.x = vector.x + x_diff;
  result.y = vector.y + y_diff;
  result.z = vector.z + z_diff;
  result = setLength(result, length(vector));
  return result;
}

/**
 * Choose a location for the camera. Balances between:
 * - Not moving too much
 * - Being orthogonal to the active marker
 * - Being able to see the active marker
 * Returns true if a new location was found.
 */
bool AutoCPDisplay::chooseCameraLocation(geometry_msgs::Point* location) {
  geometry_msgs::Point camera_position = getCameraPosition();
  if (current_control_ != NULL) {
    geometry_msgs::Point control_position = current_control_->world_position;
    float z_sign = sign(camera_position.z - control_position.z);
    float best_score = computeLocationScore(camera_position);
    geometry_msgs::Point best_location = camera_position;
    geometry_msgs::Vector3 vector = vectorBetween(
      control_position, camera_position);

    // Strategy: add random perturbations, normalize distance.
    bool new_location_found = false;
    for (int tries = 10; tries > 0; tries--) {
      geometry_msgs::Vector3 test_vector = getRandomPerturbation(vector);
      geometry_msgs::Point test_point = add(control_position, test_vector);

      // Don't go below the marker if we started out above and vice versa.
      float test_sign = sign(test_point.z - control_position.z);
      if (test_sign != z_sign) {
        continue;
      }
      
      float score = computeLocationScore(test_point);
      if (score > best_score) {
        best_score = score;
        best_location.x = test_point.x;
        best_location.y = test_point.y;
        best_location.z = test_point.z;
        new_location_found = true;
      }
    }
    location->x = best_location.x;
    location->y = best_location.y;
    location->z = best_location.z;

    delete current_control_;
    current_control_ = NULL;
    return new_location_found;
  } else {
    location->x = camera_position.x;
    location->y = camera_position.y;
    location->z = camera_position.z;
    return false;
  }
}

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
}  // namespace autocp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autocp::AutoCPDisplay, rviz::Display)
