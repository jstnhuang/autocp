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
#include <rviz/display_context.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>
#include <view_controller_msgs/CameraPlacement.h>

#include <string>

namespace rviz {
class RosTopicProperty;
class VisualizationManager;
}

namespace autocp {
class AutoCPDisplay: public rviz::Display {
  Q_OBJECT

 public:
  AutoCPDisplay();
  virtual ~AutoCPDisplay();

 protected:
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);

 private Q_SLOTS:
  void updateTopic();

 private:
  ros::NodeHandle root_nh_;
  tf::TransformListener tf_listener_;
  rviz::VisualizationManager* vm_;

  // Sensing.
  void sense();
  void getTransformOrigin(std::string frame, geometry_msgs::Point* origin);

  // Point head factor.
  ros::Subscriber point_head_subscriber_;
  geometry_msgs::Point head_focus_point_;
  void pointHeadCallback(
    const pr2_controllers_msgs::PointHeadActionGoal& goal
  );

  // Gripper factors.
  geometry_msgs::Point left_gripper_origin_;
  geometry_msgs::Point right_gripper_origin_;

  // Camera placement.
  rviz::RosTopicProperty* topic_prop_;
  ros::Publisher camera_placement_publisher_;
  geometry_msgs::Point* last_position_;
  void chooseCameraPlacement(float time_delta);
  void chooseCameraFocus(geometry_msgs::Point* focus);
  void chooseCameraLocation(geometry_msgs::Point* location);
  void setCameraPlacement(
    const geometry_msgs::Point& location,
    const geometry_msgs::Point& focus,
    const ros::Duration& time_from_start,
    view_controller_msgs::CameraPlacement* camera_placement
  );
};
}  // namespace autocp

#endif  // AUTOCP_DISPLAY_H
