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
#include <OGRE/OgreManualObject.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/render_panel.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>
#include <view_controller_msgs/CameraPlacement.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <manipulation_msgs/GraspableObjectList.h>

#include "landmarks.h"
#include "score.h"
#include "viewpoint.h"
#include "visibility.h"
#include "visualization.h"

#include <math.h>
#include <map>
#include <string>
#include <vector>
#include <stdio.h>

namespace autocp {
using geometry_msgs::Point;
using geometry_msgs::Quaternion;
using geometry_msgs::Vector3;
using visualization_msgs::Marker;

enum class Control6Dof {
  X,
  Y,
  Z,
  PITCH,
  ROLL,
  YAW
};

struct ClickedControl {
  std::string marker;
  Control6Dof control;
  geometry_msgs::Pose pose;
  // Position of the object the marker is controlling.
  Point world_position;

  ClickedControl(std::string marker, Control6Dof control,
                 geometry_msgs::Pose pose, Point world_position) {
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
static const std::map<std::string, Control6Dof> POINT_HEAD_CONTROLS = {
  { "_u1", Control6Dof::X },
  { "_u5", Control6Dof::Y },
  { "_u3", Control6Dof::Z },
  { "_u2", Control6Dof::PITCH },
  { "_u0", Control6Dof::ROLL },
  { "_u4", Control6Dof::YAW }
};

static const std::map<std::string, Control6Dof> GRIPPER_CONTROLS = {
  { "_u0", Control6Dof::X },
  { "_u4", Control6Dof::Y },
  { "_u2", Control6Dof::Z },
  { "_u3", Control6Dof::PITCH },
  { "", Control6Dof::ROLL },
  { "_u1", Control6Dof::YAW }
};

static const float MIN_DISTANCE = 0.5;
static const float MAX_DISTANCE = 5;

static const float R2 = 0.70710678118;  // sqrt(2) / 2

class AutoCPDisplay : public rviz::Display {
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
  void updateSmoothnessOption();

 private:
  ros::NodeHandle root_nh_;
  tf::TransformListener tf_listener_;
  rviz::VisualizationManager* vm_;
  Ogre::SceneManager* sm_;
  Ogre::Camera* camera_;
  Ogre::Viewport* viewport_;
  Point target_position_;
  Point current_focus_;
  Point target_focus_;
  Visualization* visualization_;

  // Canonical viewpoint locations, expressed as an offset from the current
  // focus point.
  void initializeStandardViewpoints();
  std::vector<Vector3> standard_viewpoints_;

  // Sensing.
  void getTransformOrigin(std::string frame, Point* origin);
  rviz::BoolProperty* show_fps_;

  // Landmarks container.
  Landmarks landmarks_;

  // Weights
  rviz::FloatProperty* crossing_weight_property_;
  float crossing_weight_;

  // Grippers.
  Point left_gripper_origin_;
  Point right_gripper_origin_;
  rviz::FloatProperty* gripper_weight_;

  // Head focus.
  ros::Subscriber point_head_subscriber_;
  Point head_focus_point_;
  void pointHeadCallback(const pr2_controllers_msgs::PointHeadActionGoal& goal);
  rviz::FloatProperty* head_focus_weight_;

  // Head position.
  rviz::FloatProperty* head_weight_;
  void getHeadPosition(Point* point);

  // Markers.
  ros::Subscriber full_marker_subscriber_;
  ros::Subscriber marker_feedback_subscriber_;
  void markerCallback(
      const visualization_msgs::InteractiveMarkerFeedback& feedback);
  void fullMarkerCallback(
      const visualization_msgs::InteractiveMarkerInit& im_init);
  // current_control_ is either the active control or the previous control,
  // depending on whether or not we move when a control is active.
  ClickedControl* current_control_;
  ClickedControl* active_control_;
  ClickedControl* previous_control_;

  // Segmented objects factor.
  ros::Subscriber object_segmentation_subscriber_;
  rviz::FloatProperty* segmented_object_weight_;
  std::vector<Point> segmented_object_positions_;
  void objectSegmentationCallback(
      const manipulation_msgs::GraspableObjectList& list);

  // Zoom factors.
  rviz::FloatProperty* zoom_weight_;

  // Smoothness factors.
  rviz::FloatProperty* camera_speed_;
  rviz::FloatProperty* score_threshold_;
  rviz::BoolProperty* only_move_on_idle_;
  rviz::IntProperty* occlusion_check_limit_;

  // Location weights
  rviz::FloatProperty* stay_in_place_weight_;
  rviz::FloatProperty* be_orthogonal_weight_;
  rviz::FloatProperty* stay_visible_weight_;

  // Visibility factors.
  VisibilityChecker* visibility_checker_;

  // Camera placement.
  rviz::RosTopicProperty* topic_prop_;
  ros::Publisher camera_placement_publisher_;
  Point getCameraPosition();
  void chooseCameraPlacement(float time_delta);
  Vector3 computeControlProjection(const ClickedControl* control,
                                   const Vector3& vector);

  // Score functions.
  float visibilityScore(const Point& candidate_position,
                        const Point& candidate_focus);
  float orthogonalityScore(const Point& candidate_position,
                           const Point& control_location);
  float zoomScore(const Point& candidate_position);
  float travelingScore(const Viewpoint& viewpoint);
  float crossingScore(const Point& candidate_position);
  Score computeLocationScore(const Point& candidate_position,
                             const Point& candidate_focus);

  // Camera placement logic.
  void selectViewpoints(std::vector<Vector3>* viewpoints);
  bool chooseCameraLocation(Point* location, float time_delta);
  static void setCameraPlacement(
      const Point& location, const Point& focus,
      const ros::Duration& time_from_start,
      view_controller_msgs::CameraPlacement* camera_placement);
  Point interpolatePoint(const Point& start, const Point& end,
                            float speed, float time_delta);
};
}  // namespace autocp

#endif  // AUTOCP_DISPLAY_H
