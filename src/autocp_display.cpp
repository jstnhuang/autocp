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
    "Gripper weight",
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
  try {
    if (marker_name == "point_head_goal") {
      control = POINT_HEAD_CONTROLS.at(control_name);
    } else if (
      marker_name == "r_gripper_control"
      || marker_name == "l_gripper_control"
    ) {
      control = GRIPPER_CONTROLS.at(control_name);
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
  // TODO(jstn): keep distance from the marker the same as we move around.
  // TODO(jstn): implement location shifting for pitch/row/yaw.
  // TODO(jstn): clean this up
  if (current_control_ != NULL) {
    float marker_x = current_control_->pose.position.x;
    float marker_y = current_control_->pose.position.y;
    float marker_z = current_control_->pose.position.z;
    if (current_control_->control == Control6Dof::X) {
      float x_diff = position.x - marker_x;
      float y_diff = position.y - marker_y;
      float horizontal_distance = sqrt(x_diff * x_diff + y_diff * y_diff);
      // TODO(jstn): this should be position.y < marker.y
      if (position.y < 0) {
        horizontal_distance *= -1;
      }
      location->x = marker_x;
      location->y = marker_y + horizontal_distance;
      location->z = position.z;
    } else if (current_control_->control == Control6Dof::Y) {
      float x_diff = position.x - marker_x;
      float y_diff = position.y - marker_y;
      float horizontal_distance = sqrt(x_diff * x_diff + y_diff * y_diff);
      // TODO(jstn): ditto
      if (position.x < 0) {
        horizontal_distance *= -1;
      }
      location->x = marker_x + horizontal_distance;
      location->y = marker_y;
      location->z = position.z;
    } else if (current_control_->control == Control6Dof::Z) {
      location->x = position.x;
      location->y = position.y;
      location->z = marker_z;;
    } else if (current_control_->control == Control6Dof::PITCH) {
      location->x = position.x;
      location->y = position.y;
      location->z = position.z;
    } else if (current_control_->control == Control6Dof::ROLL) {
      location->x = position.x;
      location->y = position.y;
      location->z = position.z;
    } else if (current_control_->control == Control6Dof::YAW) {
      location->x = position.x;
      location->y = position.y;
      location->z = position.z;
    } else {
      ROS_INFO("Unknown control was used.");
      location->x = position.x;
      location->y = position.y;
      location->z = position.z;
    }
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
