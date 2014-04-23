#include "autocp/autocp_sensing.h"

namespace autocp {
AutoCPSensing::AutoCPSensing(const ros::NodeHandle& root_nh,
                             tf::TransformListener* tf_listener,
                             const std::string& fixed_frame) {
  root_nh_ = root_nh;
  tf_listener_ = tf_listener;
  fixed_frame_ = fixed_frame;

  active_control_ = NULL;
  previous_control_ = NULL;

  head_focus_subscriber_ = root_nh_.subscribe(
      "head_traj_controller/point_head_action/goal", 5,
      &AutoCPSensing::HeadFocusCallback, this);

  object_segmentation_subscriber_ = root_nh_.subscribe(
      "/interactive_object_recognition_result", 5,
      &AutoCPSensing::ObjectSegmentationCallback, this);

  marker_feedback_subscriber_ = root_nh_.subscribe(
      "/pr2_marker_control_transparent/feedback", 5,
      &AutoCPSensing::MarkerFeedbackCallback, this);

  // Just read the first message to get the initial head focus point.
  marker_feedback_full_subscriber_ = root_nh_.subscribe(
      "/pr2_marker_control_transparent/update_full", 5,
      &AutoCPSensing::MarkerFeedbackFullCallback, this);
}

/*
 * Destructor.
 */
AutoCPSensing::~AutoCPSensing() {
}

/*
 * Gets the initial information we want to get on startup.
 */
void AutoCPSensing::Initialize() {
  UpdateHeadPosition();
  landmarks_.UpdateHead(&head_position_);
}

/*
 * This method updates internal values. It must be called in the update loop of
 * the main program for the sensing to work.
 */
void AutoCPSensing::Update() {
  UpdateHeadPosition();
  UpdateLeftGripperPosition();
  UpdateRightGripperPosition();

  if (active_control_ != NULL) {
    if (previous_control_ != NULL) {
      delete previous_control_;
    }
    previous_control_ = active_control_;
    active_control_ = NULL;
  }
}

Ogre::Vector3* AutoCPSensing::head_position() {
  return &head_position_;
}

Ogre::Vector3* AutoCPSensing::head_focus() {
  return &head_focus_;
}

Ogre::Vector3* AutoCPSensing::left_gripper_position() {
  return &left_gripper_position_;
}

Ogre::Vector3* AutoCPSensing::right_gripper_position() {
  return &right_gripper_position_;
}

std::vector<Ogre::Vector3>* AutoCPSensing::segmented_object_positions() {
  return &segmented_object_positions_;
}

/*
 * Gets the current control according to some definition of "current." If we
 * only want to move the camera when it's idle, then the "current" control is
 * the last used control. Otherwise, the current control is whatever control the
 * user is currently manipulating.
 */
ClickedControl* AutoCPSensing::current_control(bool only_move_on_idle) {
  if (only_move_on_idle) {
    return previous_control_;
  } else {
    return active_control_;
  }
}

Landmarks* AutoCPSensing::landmarks() {
  return &landmarks_;
}

/*
 * Returns: Whether the user is manipulating a control right now or not.
 */
bool AutoCPSensing::IsControlActive() {
  return active_control_ != NULL;
}

void AutoCPSensing::GetTransformOrigin(std::string frame,
                                       Ogre::Vector3* result) {
  ros::Duration timeout(5);
  tf_listener_->waitForTransform(fixed_frame_, frame, ros::Time(0), timeout);
  tf::StampedTransform transform;
  tf_listener_->lookupTransform(fixed_frame_, frame, ros::Time(0), transform);
  tf::Vector3 transform_origin = transform.getOrigin();

  *result = Ogre::Vector3(transform_origin.x(), transform_origin.y(),
                          transform_origin.z());
}

void AutoCPSensing::UpdateHeadPosition() {
  GetTransformOrigin("/head_tilt_link", &head_position_);
}

void AutoCPSensing::UpdateLeftGripperPosition() {
  GetTransformOrigin("/l_wrist_roll_link", &left_gripper_position_);
}

void AutoCPSensing::UpdateRightGripperPosition() {
  GetTransformOrigin("/r_wrist_roll_link", &right_gripper_position_);
}

void AutoCPSensing::HeadFocusCallback(
    const control_msgs::PointHeadActionGoal& goal) {
  auto ros_point = goal.goal.target.point;
  head_focus_ = Ogre::Vector3(ros_point.x, ros_point.y, ros_point.z);
  landmarks_.UpdateHeadFocus(&head_focus_);
}

void AutoCPSensing::ObjectSegmentationCallback(
    const manipulation_msgs::GraspableObjectList& list) {
  segmented_object_positions_.clear();
  for (const auto& obj : list.graspable_objects) {
    Ogre::Vector3 obj_location(0, 0, 0);
    for (const auto& point : obj.cluster.points) {
      obj_location.x += point.x;
      obj_location.y += point.y;
      obj_location.z += point.z;
    }
    obj_location /= obj.cluster.points.size();
    segmented_object_positions_.push_back(obj_location);
  }
  landmarks_.UpdateSegmentedObjects(segmented_object_positions_);
}

/*
 * It's necessary to have both the full callback, which gives us whether or not
 * an interactive marker is enabled or not, as well as the regular feedback,
 * which gives us the exact control that's being used.
 */
void AutoCPSensing::MarkerFeedbackCallback(
    const visualization_msgs::InteractiveMarkerFeedback& feedback) {
  if (feedback.event_type
      != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    return;
  }
  std::string marker_name = static_cast<std::string>(feedback.marker_name);
  std::string control_name = static_cast<std::string>(feedback.control_name);

  Control6Dof control;
  auto ros_position = feedback.pose.position;
  auto world_position = Ogre::Vector3(ros_position.x, ros_position.y,
                                      ros_position.z);
  try {
    if (marker_name == "head_point_goal") {
      control = POINT_HEAD_CONTROLS.at(control_name);
    } else if (marker_name == "l_gripper_control") {
      control = GRIPPER_CONTROLS.at(control_name);
      world_position = Ogre::Vector3(left_gripper_position_.x,
                                     left_gripper_position_.y,
                                     left_gripper_position_.z);
    } else if (marker_name == "r_gripper_control") {
      control = GRIPPER_CONTROLS.at(control_name);
      world_position = Ogre::Vector3(right_gripper_position_.x,
                                     right_gripper_position_.y,
                                     right_gripper_position_.z);
    } else if (marker_name == "l_posture_control") {
      control = Control6Dof::ROLL;
    } else if (marker_name == "r_posture_control") {
      control = Control6Dof::ROLL;
    } else {
      ROS_INFO("Unknown marker %s", marker_name.c_str());
      return;
    }

    active_control_ = new ClickedControl {
      marker_name,
      control,
      feedback.pose,
      world_position
    };
  } catch (const std::out_of_range& e) {
    ROS_INFO("Unknown control %s for marker %s", control_name.c_str(),
             marker_name.c_str());
  }
}

/*
 * It's necessary to have both the full callback, which gives us whether or not
 * an interactive marker is enabled or not, as well as the regular feedback,
 * which gives us the exact control that's being used.
 */
void AutoCPSensing::MarkerFeedbackFullCallback(
  const visualization_msgs::InteractiveMarkerInit& im_init) {
  landmarks_.UpdateHeadFocus(NULL);
  landmarks_.UpdateRightGripper(NULL);
  landmarks_.UpdateLeftGripper(NULL);
  for (const auto& marker : im_init.markers) {
    if (marker.name == "head_point_goal") {
      auto ros_position = marker.pose.position;
      Ogre::Vector3 position(ros_position.x, ros_position.y, ros_position.z);
      landmarks_.UpdateHeadFocus(&position);
    } else if (marker.name == "r_gripper_control") {
      auto ros_position = marker.pose.position;
      Ogre::Vector3 position(ros_position.x, ros_position.y, ros_position.z);
      landmarks_.UpdateRightGripper(&position);
    } else if (marker.name == "l_gripper_control") {
      auto ros_position = marker.pose.position;
      Ogre::Vector3 position(ros_position.x, ros_position.y, ros_position.z);
      landmarks_.UpdateLeftGripper(&position);
    }
  }
}

}
