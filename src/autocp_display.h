/**
 * Automatic Camera Placement
 * 
 * Automatically places the camera in Rviz depending on the following factors:
 * - Interactive marker visibility.
 * - Orthogonality to an interactive marker axis.
 * - Where the robot is looking / what the robot sees.
 * - Gripper visibility.
 * - Closeness to the current camera position.
 */
#ifndef AUTOCP_DISPLAY_H
#define AUTOCP_DISPLAY_H

#include <geometry_msgs/Point.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <ros/ros.h>
#include <rviz/display.h>
#include <view_controller_msgs/CameraPlacement.h>

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;
}

namespace autocp {
class AutoCPDisplay: public rviz::Display {
  Q_OBJECT

 public:
  AutoCPDisplay();
  virtual ~AutoCPDisplay();

 protected:
  virtual void onInitialize();

 private Q_SLOTS:
  void updateTopic();

 private:
  ros::NodeHandle root_nh_;

  // Camera placement.
  ros::Publisher camera_placement_publisher_;
  ros::Subscriber camera_placement_subscriber_;
  view_controller_msgs::CameraPlacement camera_placement_;
  void cameraPlacementCallback(
    const view_controller_msgs::CameraPlacement& camera_placement
  );
  rviz::RosTopicProperty* topic_prop_;
  void chooseCameraPlacement();
  void chooseCameraFocus(geometry_msgs::Point* focus);
  void chooseCameraLocation(geometry_msgs::Point* location);
  void setCameraPlacement(
    const geometry_msgs::Point& location,
    const geometry_msgs::Point& focus,
    const ros::Duration& time_from_start,
    view_controller_msgs::CameraPlacement* camera_placement
  );

  // Point head factor.
  ros::Subscriber point_head_subcriber_;
  geometry_msgs::Point point_head_focus_;
  void pointHeadCallback(
    const pr2_controllers_msgs::PointHeadActionGoal& goal
  );
};
} // namespace autocp

#endif // AUTOCP_DISPLAY_H
