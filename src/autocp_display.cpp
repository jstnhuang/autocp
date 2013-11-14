#include <pr2_controllers_msgs/PointHeadAction.h>
#include <ros/ros.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <view_controller_msgs/CameraPlacement.h>

#include "autocp_display.h"

namespace autocp {

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
}

AutoCPDisplay::~AutoCPDisplay() {
}

void AutoCPDisplay::onInitialize() {
  Display::onInitialize();
  updateTopic();
  point_head_sub_ = root_nh_.subscribe(
    "head_traj_controller/point_head_action/goal",
    5,
    &AutoCPDisplay::pointHeadCallback,
    this
  );
}

// Topic property.
void AutoCPDisplay::updateTopic() {
  pub_ = root_nh_.advertise<view_controller_msgs::CameraPlacement>(
    topic_prop_->getStdString(),
    5
  );
}

// Utility method to set the camera placement.
void AutoCPDisplay::setCameraPlacement(
  float eye_x, float eye_y, float eye_z,
  float focus_x, float focus_y, float focus_z,
  ros::Duration time_from_start,
  view_controller_msgs::CameraPlacement& camera_placement
) {
  camera_placement.target_frame = "<Fixed Frame>";

  camera_placement.time_from_start = time_from_start;

  camera_placement.eye.header.stamp = ros::Time(0);
  camera_placement.eye.header.frame_id = "torso_lift_link";
  camera_placement.focus.header.stamp = ros::Time(0);
  camera_placement.focus.header.frame_id = "torso_lift_link";
  camera_placement.up.header.stamp = ros::Time(0);
  camera_placement.up.header.frame_id = "torso_lift_link";

  camera_placement.eye.point.x = eye_x;
  camera_placement.eye.point.y = eye_y;
  camera_placement.eye.point.z = eye_z;

  camera_placement.focus.point.x = focus_x;
  camera_placement.focus.point.y = focus_y;
  camera_placement.focus.point.z = focus_z;

  camera_placement.up.vector.x = 0.0;
  camera_placement.up.vector.y = 0.0;
  camera_placement.up.vector.z = 1.0;
}

// Factor based on where the robot's looking.
void AutoCPDisplay::pointHeadCallback(
  const pr2_controllers_msgs::PointHeadActionGoal& action_goal
) {
  view_controller_msgs::CameraPlacement camera_placement;
  setCameraPlacement(
    3,
    3,
    0.8,
    action_goal.goal.target.point.x,
    action_goal.goal.target.point.y,
    action_goal.goal.target.point.z,
    action_goal.goal.min_duration,
    camera_placement
  );

  pub_.publish(camera_placement);
}


} // namespace autocp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autocp::AutoCPDisplay, rviz::Display)
