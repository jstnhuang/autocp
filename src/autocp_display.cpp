#include <tf/transform_listener.h>

#include <cmath>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/frame_manager.h>
#include <view_controller_msgs/CameraPlacement.h>

#include "autocp_display.h"

namespace autocp {

AutoCPDisplay::AutoCPDisplay(): root_nh_(""), camera_radians_(0.0) {
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

  rpm_prop_ = new rviz::FloatProperty(
    "Rotations per minute",
    6.0,
    "Number of rotations per minute.",
    this,
    SLOT(updateRpm())
  );

  fps_prop_ = new rviz::FloatProperty(
    "Updates per second",
    60.0,
    "Number of updates to perform per second.",
    this,
    SLOT(updateRpm())
  );
}

AutoCPDisplay::~AutoCPDisplay() {
}

void AutoCPDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateTopic();
  float fps = fps_prop_->getFloat();
  timer_ = root_nh_.createTimer(
    ros::Duration(1 / fps),
    &AutoCPDisplay::updateCamera,
    this
  );
}

void AutoCPDisplay::updateTopic() {
  MFDClass::updateTopic();
  pub_ = root_nh_.advertise<view_controller_msgs::CameraPlacement>(
    topic_prop_->getStdString(),
    5
  );
}

void AutoCPDisplay::updateRpm() {
}

void AutoCPDisplay::processMessage(const sensor_msgs::Imu::ConstPtr& msg) {
}

void AutoCPDisplay::updateCamera(const ros::TimerEvent& e) {
  view_controller_msgs::CameraPlacement cp;
  cp.target_frame = "<Fixed Frame>";
  float rpm = rpm_prop_->getFloat();
  float fps = fps_prop_->getFloat();
  camera_radians_ += (rpm * 2 * 3.14) / (60 * fps);
  
  cp.time_from_start = ros::Duration(1 / fps);

  cp.eye.header.stamp = ros::Time(0);
  cp.eye.header.frame_id = "torso_lift_link";
  cp.focus.header.stamp = ros::Time(0);
  cp.focus.header.frame_id = "torso_lift_link";
  cp.up.header.stamp = ros::Time(0);
  cp.up.header.frame_id = "torso_lift_link";

  cp.eye.point.x = 3 * sin(camera_radians_);
  cp.eye.point.y = 3 * cos(camera_radians_);
  cp.eye.point.z = 0.8;

  cp.focus.point.x = 0.35;
  cp.focus.point.y = 0.0;
  cp.focus.point.z = 0.8;

  cp.up.vector.x = 0.0;
  cp.up.vector.y = 0.0;
  cp.up.vector.z = 1.0;

  pub_.publish(cp);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autocp::AutoCPDisplay, rviz::Display)
