/*
 * Manages the sensing of the robot's environment as well as user actions.
 */

#ifndef AUTOCP_SENSING_H
#define AUTOCP_SENSING_H

#include <manipulation_msgs/GraspableObjectList.h>
#include <control_msgs/PointHeadActionGoal.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <OGRE/OgreVector3.h>

#include "models/clicked_control.h"
#include "landmarks.h"

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

class AutoCPSensing {
 public:
  AutoCPSensing(const ros::NodeHandle& root_nh,
                tf::TransformListener* tf_listener,
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
  bool IsControlActive();

 private:
  ros::NodeHandle root_nh_;
  tf::TransformListener* tf_listener_;
  std::string fixed_frame_;
  Landmarks landmarks_;

  // Values we want to sense.
  Ogre::Vector3 head_position_;
  Ogre::Vector3 head_focus_;
  Ogre::Vector3 left_gripper_position_;
  Ogre::Vector3 right_gripper_position_;
  std::vector<Ogre::Vector3> segmented_object_positions_;

  ClickedControl* active_control_;
  ClickedControl* previous_control_;

  // Subscribers.
  ros::Subscriber head_focus_subscriber_;
  ros::Subscriber object_segmentation_subscriber_;
  ros::Subscriber marker_feedback_subscriber_;
  ros::Subscriber marker_feedback_full_subscriber_;

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

};

}

#endif
