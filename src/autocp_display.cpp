#include "autocp_display.h"
#include <vector>
#include <string>
#include <math.h>
#include <OGRE/OgreCamera.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>

namespace autocp {
/**
 * Constructor. Hooks up the display properties.
 */
AutoCPDisplay::AutoCPDisplay(): root_nh_("") {
  topic_prop_ = new rviz::RosTopicProperty(
    "Command topic",
    "/rviz/camera_placement",
    QString::fromStdString(
      ros::message_traits::datatype<view_controller_msgs::CameraPlacement>()
    ),
    "Topic on which to send out camera placement messages.",
    this,
    SLOT(updateTopic())
  );
  gripper_weight_property_ = new rviz::FloatProperty(
    "Gripper focus weight",
    1.0,
    "How much weight to assign to the grippers' locations.",
    this,
    SLOT(updateWeights())
  );
  gripper_weight_property_->setMin(0.1);
  gripper_weight_property_->setMax(10);
  point_head_weight_property_ = new rviz::FloatProperty(
    "Head focus point weight",
    1.0,
    "How much weight to assign to the location the robot is looking.",
    this,
    SLOT(updateWeights())
  );
  point_head_weight_property_->setMin(0.1);
  point_head_weight_property_->setMax(10);
  updateWeights();
  current_control_ = NULL;

  l_gripper_cp_enabled_ = new rviz::BoolProperty(
    "Move camera with left gripper",
    true,
    "Whether or not to move the camera when using the left gripper.",
    this,
    SLOT(updateCameraOptions())
  );
  r_gripper_cp_enabled_ = new rviz::BoolProperty(
    "Move camera with right gripper",
    true,
    "Whether or not to move the camera when using the right gripper.",
    this,
    SLOT(updateCameraOptions())
  );
  point_head_cp_enabled_ = new rviz::BoolProperty(
    "Move camera with head target point",
    true,
    "Whether or not to move the camera when using the head target point.",
    this,
    SLOT(updateCameraOptions())
  );
  l_posture_cp_enabled_ = new rviz::BoolProperty(
    "Move camera with left shoulder control",
    true,
    "Whether or not to move the camera when using the left shoulder control.",
    this,
    SLOT(updateCameraOptions())
  );
  r_posture_cp_enabled_ = new rviz::BoolProperty(
    "Move camera with right shoulder control",
    true,
    "Whether or not to move the camera when using the right shoulder control.",
    this,
    SLOT(updateCameraOptions())
  );
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
    this
  );

  marker_subscriber_ = root_nh_.subscribe(
    "/pr2_marker_control_transparent/feedback",
    5,
    &AutoCPDisplay::markerCallback,
    this
  );

  vm_ = static_cast<rviz::VisualizationManager*>(context_);
}

/**
 * Main loop that alternates between sensing and placing the camera.
 */
void AutoCPDisplay::update(float wall_dt, float ros_dt) {
  sense();
  chooseCameraPlacement(ros_dt);
}

/**
 * Set the topic to publish camera placement commands to.
 */
void AutoCPDisplay::updateTopic() {
  camera_placement_publisher_ =
    root_nh_.advertise<view_controller_msgs::CameraPlacement>(
      topic_prop_->getStdString(),
      5
    );
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

/**
 * Get interactive marker locations.
 */
void AutoCPDisplay::markerCallback(
  const visualization_msgs::InteractiveMarkerFeedback& feedback
) {
  if (feedback.event_type
    != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE
  ) {
    return;
  }
  std::string marker_name = static_cast<std::string>(feedback.marker_name);
  std::string control_name = static_cast<std::string>(feedback.control_name);

  Control6Dof control;
  // TODO(jstn): finish boolean flags.
  try {
    if (marker_name == "head_point_goal" && point_head_cp_enabled_->getBool()) {
      control = POINT_HEAD_CONTROLS.at(control_name);
    } else if (marker_name == "l_gripper_control"
      && l_gripper_cp_enabled_->getBool()) {
      control = GRIPPER_CONTROLS.at(control_name);
    } else if (marker_name == "r_gripper_control"
      && r_gripper_cp_enabled_->getBool()) {
      control = GRIPPER_CONTROLS.at(control_name);
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
      feedback.pose
    };
  } catch (std::out_of_range e) {
    ROS_INFO(
      "Unknown control %s for marker %s",
      control_name.c_str(),
      marker_name.c_str()
    );
  }
}

/**
 * Get the final focus point and location for the camera, then push the camera
 * placement.
 */
void AutoCPDisplay::chooseCameraPlacement(float time_delta) {
  geometry_msgs::Point focus;
  chooseCameraFocus(&focus);

  geometry_msgs::Point location;
  chooseCameraLocation(&location);

  view_controller_msgs::CameraPlacement camera_placement;
  setCameraPlacement(
    location, focus,
    ros::Duration(time_delta),
    &camera_placement
  );

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
 * Choose a location for the camera. If an interactive marker is being used,
 * move to a position orthogonal to the control. Otherwise, don't change the
 * location.
 */
void AutoCPDisplay::chooseCameraLocation(geometry_msgs::Point* location) {
  Ogre::Vector3 position = vm_->getRenderPanel()->getCamera()->getPosition();

  // Each axis on the 6 dof marker has a plane orthogonal to it. We project the
  // camera onto the plane, and scale the remaining components so that the
  // distance to the marker is unchanged. Each ring defines a line orthogonal to
  // it, so we similarly project the camera onto that line, and scale the
  // last component so that the distance from the marker remains the same. The
  // formula for the scaling constant is 
  // sqrt((squared length of deleted components) / (squared length of remaining 
  // components) + 1)
  // TODO(jstn): Make camera always have positive z?
  if (current_control_ != NULL) {
    float marker_x = current_control_->pose.position.x;
    float marker_y = current_control_->pose.position.y;
    float marker_z = current_control_->pose.position.z;
    float x_diff = position.x - marker_x;
    float y_diff = position.y - marker_y;
    float z_diff = position.z - marker_z;
    float squared_x = x_diff * x_diff;
    float squared_y = y_diff * y_diff;
    float squared_z = z_diff * z_diff;

    float projected_x = x_diff;
    float projected_y = y_diff;
    float projected_z = z_diff;
    float deleted_distance = 0;
    float remaining_distance = 0;

    if (current_control_->control == Control6Dof::X) {
      deleted_distance = squared_x;
      remaining_distance = squared_y + squared_z;
      projected_x = 0;
      // Special case for X/Y to prevent the camera from flipping around when
      // it's directly overhead the marker.
      projected_y = setMinimumMagnitude(projected_y, 0.01 * remaining_distance);
    } else if (current_control_->control == Control6Dof::Y) {
      deleted_distance = squared_y;
      remaining_distance = squared_x + squared_z;
      projected_y = 0;
      projected_x = setMinimumMagnitude(projected_x, 0.01 * remaining_distance);
    } else if (current_control_->control == Control6Dof::Z) {
      deleted_distance = squared_z;
      remaining_distance = squared_x + squared_y;
      projected_z = 0;
    } else if (current_control_->control == Control6Dof::PITCH) {
      deleted_distance = squared_x + squared_z;
      remaining_distance = squared_y;
      projected_x = 0;
      projected_z = 0;
    } else if (current_control_->control == Control6Dof::ROLL) {
      deleted_distance = squared_y + squared_z;
      remaining_distance = squared_x;
      projected_y = 0;
      projected_z = 0;
    } else if (current_control_->control == Control6Dof::YAW) {
      deleted_distance = squared_x + squared_y;
      remaining_distance = squared_z;
      projected_x = 0;
      projected_y = 0;
    } else {
      ROS_INFO("Unknown control was used.");
      deleted_distance = 0;
      remaining_distance = 1;  // Just to make scaler = 1.
      return;
    }

    float scaler = sqrt(deleted_distance / remaining_distance + 1);
    location->x = marker_x + projected_x * scaler;
    location->y = marker_y + projected_y * scaler;
    location->z = marker_z + projected_z * scaler;

    delete current_control_;
    current_control_ = NULL;
  } else {
    location->x = position.x;
    location->y = position.y;
    location->z = position.z;
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
