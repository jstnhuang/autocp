#include <geometry_msgs/Point.h>
#include <OGRE/OgreVector3.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
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

/**
 * Attach subscribers.
 */
void AutoCPDisplay::onInitialize() {
  Display::onInitialize();
  updateTopic();

  point_head_subcriber_ = root_nh_.subscribe(
    "head_traj_controller/point_head_action/goal",
    5,
    &AutoCPDisplay::pointHeadCallback,
    this
  );

  camera_placement_subscriber_ = root_nh_.subscribe(
    "/rviz/camera_placement",
    5,
    &AutoCPDisplay::cameraPlacementCallback,
    this
  );
}

// Topic property.
void AutoCPDisplay::updateTopic() {
  camera_placement_publisher_ =
    root_nh_.advertise<view_controller_msgs::CameraPlacement>(
      topic_prop_->getStdString(),
      5
    );
}

void AutoCPDisplay::cameraPlacementCallback(
  const view_controller_msgs::CameraPlacement& camera_placement
) {
  camera_placement_ = camera_placement;
}

/**
 * Get the final focus point and location for the camera, then push the camera
 * placement.
 */
void AutoCPDisplay::chooseCameraPlacement() {
  geometry_msgs::Point focus;
  chooseCameraFocus(&focus);

  geometry_msgs::Point location;
  chooseCameraLocation(&location);

  view_controller_msgs::CameraPlacement camera_placement;
  setCameraPlacement(location, focus, ros::Duration(1.0), &camera_placement);
  
  camera_placement_publisher_.publish(camera_placement);
}

/**
 * Choose a final focus point for the camera. The final focus point is the mean
 * of the current focus points.
 */
void AutoCPDisplay::chooseCameraFocus(geometry_msgs::Point* focus) {
  focus->x = point_head_focus_.x;
  focus->y = point_head_focus_.y;
  focus->z = point_head_focus_.z;
}

/**
 * Choose a location for the camera. The camera should be located at some point
 * on a hemisphere, centered on the robot. We will pick some random points near
 * the current camera position, and pick one which can see all the focus points.
 * Ties are broken based on which is closest to the current camera position.
 */
void AutoCPDisplay::chooseCameraLocation(geometry_msgs::Point* location) {
  Ogre::Vector3 candidate_position(3, 3, 3);
  Ogre::Vector3 point_head_position(point_head_focus_.x, point_head_focus_.y, point_head_focus_.z);
  Ogre::Camera candidate ("candidate", scene_manager_);
  candidate.setPosition(candidate_position);
  candidate.lookAt(point_head_position);

  bool visible = candidate.isVisible(point_head_position);
  if (visible) {
    ROS_INFO("Visible");
  } else {
    ROS_INFO("Not visible");
  }
  location->x = 3;
  location->y = 3;
  location->z = 3;
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

  camera_placement->eye.header.stamp = ros::Time(0);
  camera_placement->eye.header.frame_id = "torso_lift_link";
  camera_placement->focus.header.stamp = ros::Time(0);
  camera_placement->focus.header.frame_id = "torso_lift_link";
  camera_placement->up.header.stamp = ros::Time(0);
  camera_placement->up.header.frame_id = "torso_lift_link";

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

// Factor based on where the robot's looking.
void AutoCPDisplay::pointHeadCallback(
  const pr2_controllers_msgs::PointHeadActionGoal& action_goal
) {
  point_head_focus_ = action_goal.goal.target.point;
  chooseCameraPlacement();
}

} // namespace autocp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autocp::AutoCPDisplay, rviz::Display)
