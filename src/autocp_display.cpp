#include "autocp_display.h"
#include "utils.h"

namespace autocp {

/**
 * Constructor. Hooks up the display properties.
 */
AutoCPDisplay::AutoCPDisplay()
    : root_nh_(""), current_viewpoint_(), target_viewpoint_(), sensing_(NULL),
      visualization_(NULL) {
  topic_prop_ =
      new rviz::RosTopicProperty(
          "Command topic",
          "/rviz/camera_placement",
          QString::fromStdString(
              ros::message_traits::datatype<
                  view_controller_msgs::CameraPlacement>()),
          "Topic on which to send out camera placement messages.", this,
          SLOT(updateTopic()));

  // Landmark weights.
  l_gripper_weight_ = new rviz::FloatProperty(
      "Left gripper focus weight", 0.20,
      "How much weight to assign to the left gripper's location.", this,
      SLOT(updateWeights()));
  l_gripper_weight_->setMin(0);
  l_gripper_weight_->setMax(1);

  r_gripper_weight_ = new rviz::FloatProperty(
      "Right gripper focus weight", 0.20,
      "How much weight to assign to the right gripper's location.", this,
      SLOT(updateWeights()));
  r_gripper_weight_->setMin(0);
  r_gripper_weight_->setMax(1);

  head_weight_ = new rviz::FloatProperty(
      "Head weight", 0.20,
      "How much weight to the location of the robot's head.", this,
      SLOT(updateWeights()));
  head_weight_->setMin(0);
  head_weight_->setMax(1);

  head_focus_weight_ = new rviz::FloatProperty(
      "Head focus point weight", 0.20,
      "How much weight to assign to the location the robot is looking.", this,
      SLOT(updateWeights()));
  head_focus_weight_->setMin(0);
  head_focus_weight_->setMax(1);

  segmented_object_weight_ = new rviz::FloatProperty(
      "Segmented object weight", 0.20,
      "How much weight to assign to the locations of segmented objects", this,
      SLOT(updateWeights()));
  segmented_object_weight_->setMin(0);
  segmented_object_weight_->setMax(1);

  // Weights on location.
  stay_in_place_weight_ = new rviz::FloatProperty(
      "Movement moderation weight", 0.1,
      "How much weight to points close to the current location.", this,
      SLOT(updateWeights()));
  stay_in_place_weight_->setMin(0);
  stay_in_place_weight_->setMax(1);

  be_orthogonal_weight_ = new rviz::FloatProperty(
      "Marker orthogonality weight", 0.5,
      "How much weight to assign to points orthogonal to the current marker.",
      this, SLOT(updateWeights()));
  be_orthogonal_weight_->setMin(0);
  be_orthogonal_weight_->setMax(1);

  stay_visible_weight_ = new rviz::FloatProperty(
      "Marker visibility weight", 0.25,
      "How much weight to assign to points where the current marker is "
      "visible.",
      this, SLOT(updateWeights()));
  stay_visible_weight_->setMin(0);
  stay_visible_weight_->setMax(1);

  zoom_weight_ = new rviz::FloatProperty(
      "Zoom weight", 0.1,
      "How much weight to assign to being close to landmarks.", this,
      SLOT(updateWeights()));
  zoom_weight_->setMin(0);
  zoom_weight_->setMax(1);

  crossing_weight_ = new rviz::FloatProperty(
      "Line crossing weight", 0.05,
      "Weight for not flipping the user interface when using a control.", this,
      SLOT(updateWeights()));
  crossing_weight_->setMin(0);
  crossing_weight_->setMax(1);

  // Other properties.
  score_threshold_ = new rviz::FloatProperty(
      "Score improvement threshold", 1.05,
      "Factor by which the score must improve before adjusting the position.",
      this, SLOT(updateWeights()));
  score_threshold_->setMin(0);
  score_threshold_->setMax(30);

  camera_speed_ = new rviz::FloatProperty(
      "Camera speed", 3, "How many m/s the camera can move.",
      this, SLOT(updateCameraOptions()));
  camera_speed_->setMin(0);
  camera_speed_->setMax(10);

  focus_speed_ = new rviz::FloatProperty(
      "Focus speed", 0.5, "How many m/s the camera's focus point can move.",
      this, SLOT(updateCameraOptions()));
  focus_speed_->setMin(0);
  focus_speed_->setMax(10);

  min_zoom_ = new rviz::FloatProperty(
      "Minimum zoom", 0.5, "Minimum distance to zoom into landmarks.",
      this, SLOT(updateWeights()));
  min_zoom_->setMin(0);
  min_zoom_->setMax(10);

  max_zoom_ = new rviz::FloatProperty(
      "Maximum zoom", 5, "Maximum distance to zoom from landmarks.",
      this, SLOT(updateWeights()));
  max_zoom_->setMin(0);
  max_zoom_->setMax(10);

  max_travel_ = new rviz::FloatProperty(
      "Maximum travel distance", 1,
      "Traveling distance per frame for the maximum traveling penalty.",
      this, SLOT(updateWeights()));
  max_travel_->setMin(0);
  max_travel_->setMax(10);

  only_move_on_idle_ = new rviz::BoolProperty(
      "Don't move when using a marker", false,
      "Restricts camera repositioning to when no marker is being used.", this,
      SLOT(updateSmoothnessOption()));

  occlusion_check_limit_ = new rviz::IntProperty(
      "Occlusion check limit", 1000,
      "The number of occlusions to check each frame.", this,
      SLOT(updateWeights()));
  occlusion_check_limit_->setMin(0);
  occlusion_check_limit_->setMax(100000);

  show_fps_ = new rviz::BoolProperty(
      "Show FPS", false, "Whether or not to show the frames per second.", this,
      SLOT(updateCameraOptions()));
}

/**
 * Destructor.
 */
AutoCPDisplay::~AutoCPDisplay() {
}

/**
 * Initialization. Attaches callbacks to subscribers.
 */
void AutoCPDisplay::onInitialize() {
  Display::onInitialize();
  sensing_ = new AutoCPSensing(root_nh_, &tf_listener_,
                               fixed_frame_.toStdString());
  sensing_->Initialize();
  visualization_ = new Visualization(root_nh_, fixed_frame_.toStdString());

  vm_ = static_cast<rviz::VisualizationManager*>(context_);
  sm_ = vm_->getSceneManager();
  camera_ = vm_->getRenderPanel()->getCamera();

  optimization_ = new Optimization(sensing_, sm_, camera_, visualization_);

  current_viewpoint_ = Viewpoint(camera_->getPosition(),
                                 *(sensing_->head_position()));
  target_viewpoint_ = current_viewpoint_;

  updateTopic();
  updateWeights();
}

// Parameter update handlers ---------------------------------------------------
/**
 * Set the topic to publish camera placement commands to.
 */
void AutoCPDisplay::updateTopic() {
  camera_placement_publisher_ = root_nh_
      .advertise<view_controller_msgs::CameraPlacement>(
      topic_prop_->getStdString(), 5);
}

/**
 * No-op for camera options.
 */
void AutoCPDisplay::updateCameraOptions() {
  return;
}

/**
 * Updates the weights.
 */
void AutoCPDisplay::updateWeights() {
  auto landmarks = sensing_->landmarks();
  landmarks->UpdateLeftGripperWeight(l_gripper_weight_->getFloat());
  landmarks->UpdateRightGripperWeight(r_gripper_weight_->getFloat());
  landmarks->UpdateHeadWeight(head_weight_->getFloat());
  landmarks->UpdateHeadFocusWeight(head_focus_weight_->getFloat());
  landmarks->UpdateSegmentedObjectWeight(segmented_object_weight_->getFloat());
  optimization_->set_visibility_weight(stay_visible_weight_->getFloat());
  optimization_->set_view_angle_weight(be_orthogonal_weight_->getFloat());
  optimization_->set_zoom_weight(zoom_weight_->getFloat());
  optimization_->set_travel_weight(stay_in_place_weight_->getFloat());
  optimization_->set_crossing_weight(crossing_weight_->getFloat());
  optimization_->set_score_threshold(score_threshold_->getFloat());
  optimization_->set_max_visibility_checks(occlusion_check_limit_->getInt());
  optimization_->set_min_zoom(min_zoom_->getFloat());
  optimization_->set_max_zoom(max_zoom_->getFloat());
  optimization_->set_max_travel(max_travel_->getFloat());
}

void AutoCPDisplay::updateSmoothnessOption() {
  optimization_->set_only_move_on_idle(only_move_on_idle_->getBool());
}

// Camera placement logic ------------------------------------------------------
/**
 * Main loop that alternates between sensing and placing the camera.
 */
void AutoCPDisplay::update(float wall_dt, float ros_dt) {
  if (sensing_->landmarks()->NumLandmarks() == 0) {
    return;
  }
  current_viewpoint_.position = camera_->getPosition();
  chooseCameraPlacement(wall_dt);

  if (show_fps_->getBool()) {
    ROS_INFO("FPS: %f", 1 / wall_dt);
  }
  sensing_->Update();
}

/**
 * Get the final focus point and location for the camera, then push the camera
 * placement.
 */
void AutoCPDisplay::chooseCameraPlacement(float time_delta) {
  if (!only_move_on_idle_->getBool() || !sensing_->IsControlActive()) {
    optimization_->ChooseViewpoint(current_viewpoint_, &target_viewpoint_);
  }
  Viewpoint next_viewpoint;
  interpolateViewpoint(current_viewpoint_, target_viewpoint_,
                       camera_speed_->getFloat(), focus_speed_->getFloat(),
                       time_delta, &next_viewpoint);
  visualization_->ShowFocus(next_viewpoint.focus);

  view_controller_msgs::CameraPlacement camera_placement;
  setCameraPlacement(next_viewpoint, ros::Duration(time_delta),
                     &camera_placement);
  camera_placement_publisher_.publish(camera_placement);
  current_viewpoint_ = next_viewpoint;
}

// Utilities -------------------------------------------------------------------
/**
 * Convenience method to set the camera placement.
 */
void AutoCPDisplay::setCameraPlacement(
    const Viewpoint& viewpoint,
    const ros::Duration& time_from_start,
    view_controller_msgs::CameraPlacement* camera_placement) {
  camera_placement->target_frame = "<Fixed Frame>";

  camera_placement->time_from_start = time_from_start;

  camera_placement->eye.header.stamp = ros::Time::now();
  camera_placement->eye.header.frame_id = "<Fixed Frame>";
  camera_placement->focus.header.stamp = ros::Time::now();
  camera_placement->focus.header.frame_id = "<Fixed Frame>";
  camera_placement->up.header.stamp = ros::Time::now();
  camera_placement->up.header.frame_id = "<Fixed Frame>";

  camera_placement->eye.point.x = viewpoint.position.x;
  camera_placement->eye.point.y = viewpoint.position.y;
  camera_placement->eye.point.z = viewpoint.position.z;

  camera_placement->focus.point.x = viewpoint.focus.x;
  camera_placement->focus.point.y = viewpoint.focus.y;
  camera_placement->focus.point.z = viewpoint.focus.z;

  camera_placement->up.vector.x = 0.0;
  camera_placement->up.vector.y = 0.0;
  camera_placement->up.vector.z = 1.0;
}

}  // namespace autocp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autocp::AutoCPDisplay, rviz::Display)
