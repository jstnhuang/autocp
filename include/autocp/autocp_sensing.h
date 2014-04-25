/*
 * Manages the sensing of the robot's environment as well as user actions.
 */

#ifndef AUTOCP_SENSING_H
#define AUTOCP_SENSING_H

#include <manipulation_msgs/GraspableObjectList.h>
#include <control_msgs/PointHeadActionGoal.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>

#include "autocp/models/clicked_control.h"
#include "autocp/models/viewpoint.h"
#include "autocp/landmarks.h"

namespace autocp {

// Maps built by observing /pr2_marker_control_transparent/feedback
// TODO(jstn): These change between executions, find a way to make this work.
static const std::map<std::string, Control6Dof> POINT_HEAD_CONTROLS = {
  { "_u1", Control6Dof::X },
  { "_u5", Control6Dof::Y },
  { "_u3", Control6Dof::Z },
  { "_u4", Control6Dof::PITCH },
  { "_u0", Control6Dof::ROLL },
  { "_u2", Control6Dof::YAW }
};

static const std::map<std::string, Control6Dof> GRIPPER_CONTROLS = {
  { "_u0", Control6Dof::X },
  { "_u4", Control6Dof::Y },
  { "_u2", Control6Dof::Z },
  { "_u3", Control6Dof::PITCH },
  { "", Control6Dof::ROLL },
  { "_u1", Control6Dof::YAW }
};

enum class MarkerClickState {
  kStart,
  kDown,
  kClick,
  kDrag
};

class AutoCPSensing {
 public:
  AutoCPSensing(const ros::NodeHandle& root_nh,
                tf::TransformListener* tf_listener,
                Ogre::Camera* camera,
                const std::string& fixed_frame);
  ~AutoCPSensing();
  void Initialize();
  void Update();
  Ogre::Vector3* head_position();
  Ogre::Vector3* head_focus();
  Ogre::Vector3* left_gripper_position();
  Ogre::Vector3* right_gripper_position();
  std::vector<Ogre::Vector3>* segmented_object_positions();
  ClickedControl* previous_control();
  Landmarks* landmarks();
  Viewpoint current_viewpoint();
  bool IsMouseClick();
  bool IsMouseDrag();
  bool IsControlActive();

 private:
  ros::NodeHandle root_nh_;
  tf::TransformListener* tf_listener_;
  Ogre::Camera* camera_;
  std::string fixed_frame_;
  Landmarks landmarks_;

  // Values we want to sense.
  Ogre::Vector3 head_position_;
  Ogre::Vector3 head_focus_;
  Ogre::Vector3 left_gripper_position_;
  Ogre::Vector3 right_gripper_position_;
  std::vector<Ogre::Vector3> segmented_object_positions_;
  MarkerClickState marker_click_state_;

  ClickedControl* active_control_;
  ClickedControl* previous_control_;

  // Subscribers.
  ros::Subscriber head_focus_subscriber_;
  ros::Subscriber object_segmentation_subscriber_;
  ros::Subscriber marker_feedback_subscriber_;
  ros::Subscriber marker_feedback_full_subscriber_;

  ros::Time mouse_down_time_;

  void GetTransformOrigin(std::string frame, Ogre::Vector3* result);
  void UpdateHeadPosition();
  void UpdateLeftGripperPosition();
  void UpdateRightGripperPosition();

  // Callbacks.
  void HeadFocusCallback(const control_msgs::PointHeadActionGoal& goal);
  void ObjectSegmentationCallback(
      const manipulation_msgs::GraspableObjectList& list);
  void MarkerFeedbackCallback(
      const visualization_msgs::InteractiveMarkerFeedback& feedback);
  void MarkerFeedbackFullCallback(
      const visualization_msgs::InteractiveMarkerInit& im_init);

  // Helpers.
  void DetectMouseEvent(int event_type, std::string marker_name);
};

}

#endif
