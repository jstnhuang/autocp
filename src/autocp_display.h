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

#include <OGRE/OgreCamera.h>
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

#include "models/control_6dof.h"
#include "models/clicked_control.h"
#include "models/score.h"
#include "models/viewpoint.h"
#include "autocp_sensing.h"
#include "visibility.h"
#include "visualization.h"

#include <math.h>
#include <map>
#include <string>
#include <vector>
#include <stdio.h>

namespace autocp {

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

  AutoCPSensing* sensing_;
  Visualization* visualization_;
  VisibilityChecker* visibility_checker_;

  // Canonical viewpoint locations, expressed as an offset from the current
  // focus point.
  void initializeStandardViewpoints();
  std::vector<Ogre::Vector3> standard_viewpoints_;

  rviz::BoolProperty* show_fps_;


  // Landmark weights.
  rviz::FloatProperty* gripper_weight_;
  rviz::FloatProperty* head_weight_;
  rviz::FloatProperty* head_focus_weight_;
  rviz::FloatProperty* segmented_object_weight_;

  // Property weights.
  rviz::FloatProperty* stay_in_place_weight_;
  rviz::FloatProperty* be_orthogonal_weight_;
  rviz::FloatProperty* stay_visible_weight_;
  rviz::FloatProperty* zoom_weight_;
  rviz::FloatProperty* crossing_weight_property_;

  // Smoothness controls.
  rviz::FloatProperty* camera_speed_;
  rviz::FloatProperty* score_threshold_;
  rviz::BoolProperty* only_move_on_idle_;
  rviz::IntProperty* occlusion_check_limit_;

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
};

}  // namespace autocp

#endif  // AUTOCP_DISPLAY_H
