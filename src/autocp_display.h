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
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreViewport.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/render_panel.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>
#include <view_controller_msgs/CameraPlacement.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <manipulation_msgs/GraspableObjectList.h>
#include "landmarks.h"

#include <math.h>
#include <map>
#include <string>
#include <vector>
#include <array>

namespace autocp {
enum class Control6Dof { X, Y, Z, PITCH, ROLL, YAW };

struct ClickedControl {
  std::string marker;
  Control6Dof control;
  geometry_msgs::Pose pose;
  // Position of the object the marker is controlling.
  geometry_msgs::Point world_position;

  ClickedControl(std::string marker, Control6Dof control,
    geometry_msgs::Pose pose, geometry_msgs::Point world_position
  ) {
    this->marker = marker;
    this->control = control;
    this->pose = pose;
    this->world_position = world_position;
  }
  ~ClickedControl() {
  }
};

// Maps built by observing /pr2_marker_control_transparent/feedback
// TODO(jstn): Are these fixed or do they depend on other factors?
static const  std::map<std::string, Control6Dof> POINT_HEAD_CONTROLS = {
  {"_u1", Control6Dof::X},
  {"_u5", Control6Dof::Y},
  {"_u3", Control6Dof::Z},
  {"_u4", Control6Dof::PITCH},
  {"_u0", Control6Dof::ROLL},
  {"_u2", Control6Dof::YAW},
};

static const std::map<std::string, Control6Dof> GRIPPER_CONTROLS = {
  {"_u0", Control6Dof::X},
  {"_u4", Control6Dof::Y},
  {"_u2", Control6Dof::Z},
  {"_u3", Control6Dof::PITCH},
  {"", Control6Dof::ROLL},
  {"_u1", Control6Dof::YAW},
};


static const float MIN_DISTANCE = 0.5;
static const float MAX_DISTANCE = 10;

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
  void updateCameraOptions();
  void updateCameraSpeed();

 private:
  ros::NodeHandle root_nh_;
  tf::TransformListener tf_listener_;
  rviz::VisualizationManager* vm_;
  std::vector<float> weights_;
  Ogre::Camera* camera_;
  Ogre::Viewport* viewport_;
  geometry_msgs::Point target_position_;
  geometry_msgs::Point camera_focus_;

  // Canonical viewpoint locations, expressed as an offset from the robot's
  // origin.
  std::array<geometry_msgs::Vector3, 8> standard_viewpoints_;

  // Sensing.
  void sense();
  void getTransformOrigin(std::string frame, geometry_msgs::Point* origin);
  rviz::BoolProperty* show_fps_;

  // Landmarks container.
  Landmarks landmarks_;

  // Grippers.
  geometry_msgs::Point left_gripper_origin_;
  geometry_msgs::Point right_gripper_origin_;
  rviz::FloatProperty* gripper_weight_;

  // Head focus.
  ros::Subscriber point_head_subscriber_;
  geometry_msgs::Point head_focus_point_;
  void pointHeadCallback(
    const pr2_controllers_msgs::PointHeadActionGoal& goal);
  rviz::FloatProperty* head_focus_weight_;

  // Current marker.
  ros::Subscriber marker_subscriber_;
  void markerCallback(
    const visualization_msgs::InteractiveMarkerFeedback& feedback);
  rviz::FloatProperty* current_marker_weight_;
  ClickedControl* current_control_;

  // Segmented objects factor.
  ros::Subscriber object_segmentation_subscriber_;
  rviz::FloatProperty* segmented_object_weight_;
  std::vector<geometry_msgs::Point> segmented_object_positions_;
  void objectSegmentationCallback(
    const manipulation_msgs::GraspableObjectList& list);

  // Zoom factors.
  rviz::FloatProperty* zoom_weight_;
  
  // Smoothness factors.
  rviz::FloatProperty* camera_speed_;
  rviz::FloatProperty* score_threshold_;

  // Location weights
  rviz::FloatProperty* stay_in_place_weight_;
  rviz::FloatProperty* be_orthogonal_weight_;
  rviz::FloatProperty* stay_visible_weight_;

  // Visibility factors.
  void projectWorldToViewport(
    const geometry_msgs::Point& point,
    int* screen_x,
    int* screen_y);
  float occlusionDistance(const geometry_msgs::Point& point);
  float occlusionDistanceFrom(
    const geometry_msgs::Point& point,
    const geometry_msgs::Point& camera_position,
    const geometry_msgs::Point& focus);
  bool isOccludedFrom(
    const geometry_msgs::Point& point,
    const geometry_msgs::Point& camera_position,
    const geometry_msgs::Point& focus);

  // Camera placement.
  rviz::RosTopicProperty* topic_prop_;
  ros::Publisher camera_placement_publisher_;
  geometry_msgs::Point getCameraPosition();
  void chooseCameraPlacement(float time_delta);
  void chooseCameraFocus(geometry_msgs::Point* focus);
  geometry_msgs::Vector3 computeControlProjection(
    const ClickedControl& control,
    const geometry_msgs::Vector3& vector);
  float computeLocationScore(const geometry_msgs::Point& location);
  bool chooseCameraLocation(geometry_msgs::Point* location);
  static void setCameraPlacement(
    const geometry_msgs::Point& location,
    const geometry_msgs::Point& focus,
    const ros::Duration& time_from_start,
    view_controller_msgs::CameraPlacement* camera_placement);
  geometry_msgs::Point interpolatePosition(
    const geometry_msgs::Point& start, const geometry_msgs::Point& end,
    float time_delta);
};
}  // namespace autocp

#endif  // AUTOCP_DISPLAY_H
