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
#include <rviz/properties/float_property.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>
#include <view_controller_msgs/CameraPlacement.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <math.h>
#include <map>
#include <string>
#include <vector>

namespace rviz {
class RosTopicProperty;
class FloatProperty;
class VisualizationManager;
}

namespace autocp {
enum class Control6Dof { X, Y, Z, PITCH, ROLL, YAW };

struct ClickedControl {
  std::string marker;
  Control6Dof control;
  geometry_msgs::Pose pose;

  ClickedControl(std::string marker, Control6Dof control,
    geometry_msgs::Pose pose
  ) {
    this->marker = marker;
    this->control = control;
    this->pose = pose;
  }
  ~ClickedControl() {
  }
};

const static std::map<std::string, Control6Dof> POINT_HEAD_CONTROLS = {
  {"_u3", Control6Dof::X},
  {"_u5", Control6Dof::Y},
  {"_u1", Control6Dof::Z},
  {"_u4", Control6Dof::PITCH},
  {"_u2", Control6Dof::ROLL},
  {"_u0", Control6Dof::YAW},
};

const static std::map<std::string, Control6Dof> GRIPPER_CONTROLS = {
  {"_u0", Control6Dof::X},
  {"_u4", Control6Dof::Y},
  {"_u2", Control6Dof::Z},
  {"_u3", Control6Dof::PITCH},
  {"", Control6Dof::ROLL},
  {"_u1", Control6Dof::YAW},
};

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
  void updateWeights();

 private:
  ros::NodeHandle root_nh_;
  tf::TransformListener tf_listener_;
  rviz::VisualizationManager* vm_;
  std::vector<float> weights_;

  // Sensing.
  void sense();
  void getTransformOrigin(std::string frame, geometry_msgs::Point* origin);

  // Point head factor.
  ros::Subscriber point_head_subscriber_;
  geometry_msgs::Point head_focus_point_;
  void pointHeadCallback(
    const pr2_controllers_msgs::PointHeadActionGoal& goal
  );
  rviz::FloatProperty* point_head_weight_property_;

  // Gripper factors.
  geometry_msgs::Point left_gripper_origin_;
  geometry_msgs::Point right_gripper_origin_;
  rviz::FloatProperty* gripper_weight_property_;

  // Interactive marker factors.
  ros::Subscriber marker_subscriber_;
  void markerCallback(
    const visualization_msgs::InteractiveMarkerFeedback& feedback
  );
  ClickedControl* current_control_;

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
  
  // Utilities
  inline float setMinimumMagnitude(float num, float magnitude) {
    if (abs(num) < magnitude) {
      if (num < 0) {
        num = -magnitude;
      } else {
        num = magnitude;
      }
    } else {
      return num;
    }
  }
};
}  // namespace autocp

#endif  // AUTOCP_DISPLAY_H
