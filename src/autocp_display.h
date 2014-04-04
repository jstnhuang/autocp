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
  Ogre::Vector3 world_position;

  ClickedControl(std::string marker, Control6Dof control,
                 geometry_msgs::Pose pose, Ogre::Vector3 world_position) {
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

  Viewpoint current_viewpoint_;
  Viewpoint target_viewpoint_;

  //Ogre::Vector3 target_position_;
  //Ogre::Vector3 current_focus_;
  //Ogre::Vector3 target_focus_;
  Visualization* visualization_;

  // Canonical viewpoint locations, expressed as an offset from the current
  // focus point.
  void initializeStandardViewpoints();
  std::vector<Ogre::Vector3> standard_viewpoints_;

  // Sensing.
  void getTransformOrigin(std::string frame, Ogre::Vector3* origin);
  rviz::BoolProperty* show_fps_;

  // Landmarks container.
  Landmarks landmarks_;

  // Weights
  rviz::FloatProperty* crossing_weight_property_;
  float crossing_weight_;

  // Grippers.
  Ogre::Vector3 left_gripper_origin_;
  Ogre::Vector3 right_gripper_origin_;
  rviz::FloatProperty* gripper_weight_;

  // Head focus.
  ros::Subscriber point_head_subscriber_;
  Ogre::Vector3 head_focus_point_;
  void pointHeadCallback(const pr2_controllers_msgs::PointHeadActionGoal& goal);
  rviz::FloatProperty* head_focus_weight_;

  // Head position.
  rviz::FloatProperty* head_weight_;
  void getHeadPosition(Ogre::Vector3* point);

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
  std::vector<Ogre::Vector3> segmented_object_positions_;
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
  void chooseCameraPlacement(float time_delta);
  void computeControlProjection(const ClickedControl& control,
                                const Ogre::Vector3& vector,
                                Ogre::Vector3* projection);

  // Score functions.
  float visibilityScore(const Viewpoint& viewpoint);
  float orthogonalityScore(const Viewpoint& viewpoint,
                           const ClickedControl& control);
  float zoomScore(const Viewpoint& viewpoint);
  float travelingScore(const Viewpoint& current_viewpoint,
                       const Viewpoint& candidate_viewpoint);
  float crossingScore(const Viewpoint& viewpoint,
                      const ClickedControl& control);
  void computeViewpointScore(const Viewpoint& viewpoint, Score* score);

  // Camera placement logic.
  void selectViewpoints(std::vector<Viewpoint>* viewpoints);
  void chooseCameraViewpoint(Viewpoint* target_viewpoint);
  static void setCameraPlacement(
      const Viewpoint& viewpoint,
      const ros::Duration& time_from_start,
      view_controller_msgs::CameraPlacement* camera_placement);

  void interpolateViewpoint(const Viewpoint& start, const Viewpoint& end,
                        float position_speed, float focus_speed,
                        float time_delta, Viewpoint* result);
  void interpolatePoint(const Ogre::Vector3& start, const Ogre::Vector3& end,
                        float speed, float time_delta, Ogre::Vector3* result);
};
}  // namespace autocp

#endif  // AUTOCP_DISPLAY_H
