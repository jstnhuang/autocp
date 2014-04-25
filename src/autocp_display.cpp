#include "autocp/autocp_display.h"
#include "autocp/utils.h"

namespace autocp {

/**
 * Constructor. Hooks up the display properties.
 */
AutoCPDisplay::AutoCPDisplay()
    : node_handle_(""),
      tf_listener_(),
      camera_placement_publisher_(),
      camera_(),
      sensing_(NULL),
      visualization_(NULL),
      visibility_checker_(NULL),
      optimization_(NULL),
      state_(State::WAITING) {
  topic_prop_ =
      new rviz::RosTopicProperty(
          "Command topic",
          "/rviz/camera_placement",
          QString::fromStdString(
              ros::message_traits::datatype<
                  view_controller_msgs::CameraPlacement>()),
          "Topic on which to send out camera placement messages.", this,
          SLOT(UpdateTopic()));

  // Landmark weights.
  l_gripper_weight_ = new rviz::FloatProperty(
      "Left gripper focus weight", 0.20,
      "How much weight to assign to the left gripper's location.", this,
      SLOT(UpdateWeights()));
  l_gripper_weight_->setMin(0);
  l_gripper_weight_->setMax(1);

  r_gripper_weight_ = new rviz::FloatProperty(
      "Right gripper focus weight", 0.20,
      "How much weight to assign to the right gripper's location.", this,
      SLOT(UpdateWeights()));
  r_gripper_weight_->setMin(0);
  r_gripper_weight_->setMax(1);

  head_weight_ = new rviz::FloatProperty(
      "Head weight", 0.20,
      "How much weight to the location of the robot's head.", this,
      SLOT(UpdateWeights()));
  head_weight_->setMin(0);
  head_weight_->setMax(1);

  head_focus_weight_ = new rviz::FloatProperty(
      "Head focus point weight", 0.20,
      "How much weight to assign to the location the robot is looking.", this,
      SLOT(UpdateWeights()));
  head_focus_weight_->setMin(0);
  head_focus_weight_->setMax(1);

  segmented_object_weight_ = new rviz::FloatProperty(
      "Segmented object weight", 0.20,
      "How much weight to assign to the locations of segmented objects", this,
      SLOT(UpdateWeights()));
  segmented_object_weight_->setMin(0);
  segmented_object_weight_->setMax(1);

  // Weights on location.
  be_orthogonal_weight_ = new rviz::FloatProperty(
      "Marker orthogonality weight", 0.20,
      "How much weight to assign to points orthogonal to the current marker.",
      this, SLOT(UpdateWeights()));
  be_orthogonal_weight_->setMin(0);
  be_orthogonal_weight_->setMax(1);

  stay_visible_weight_ = new rviz::FloatProperty(
      "Marker visibility weight", 0.20,
      "How much weight to assign to points where the current marker is "
      "visible.",
      this, SLOT(UpdateWeights()));
  stay_visible_weight_->setMin(0);
  stay_visible_weight_->setMax(1);

  centering_weight_ = new rviz::FloatProperty(
      "Centering weight", 0.20,
      "How much weight to assign to having landmarks centered on screen.",
      this, SLOT(UpdateWeights()));
  centering_weight_->setMin(0);
  centering_weight_->setMax(1);

  zoom_weight_ = new rviz::FloatProperty(
      "Zoom weight", 0.20,
      "How much weight to assign to being close to landmarks.", this,
      SLOT(UpdateWeights()));
  zoom_weight_->setMin(0);
  zoom_weight_->setMax(1);

  crossing_weight_ = new rviz::FloatProperty(
      "Line crossing weight", 0.20,
      "How much to penalize \"crossing the line.\"", this,
      SLOT(UpdateWeights()));
  crossing_weight_->setMin(0);
  crossing_weight_->setMax(1);

  // Other properties.
  score_threshold_ = new rviz::FloatProperty(
      "Score improvement threshold", 1.05,
      "Factor by which the score must improve before adjusting the position.",
      this, SLOT(UpdateWeights()));
  score_threshold_->setMin(0);
  score_threshold_->setMax(30);

  min_zoom_ = new rviz::FloatProperty(
      "Minimum zoom", 0.5, "Minimum distance to zoom into landmarks.",
      this, SLOT(UpdateWeights()));
  min_zoom_->setMin(0);
  min_zoom_->setMax(10);

  max_zoom_ = new rviz::FloatProperty(
      "Maximum zoom", 5, "Maximum distance to zoom from landmarks.",
      this, SLOT(UpdateWeights()));
  max_zoom_->setMin(0);
  max_zoom_->setMax(10);

  occlusion_check_limit_ = new rviz::IntProperty(
      "Occlusion check limit", 1000,
      "The number of occlusions to check each frame.", this,
      SLOT(UpdateWeights()));
  occlusion_check_limit_->setMin(0);
  occlusion_check_limit_->setMax(100000);

  show_fps_ = new rviz::BoolProperty(
      "Show FPS", false, "Whether or not to show the frames per second.", this,
      SLOT(UpdateCameraOptions()));
}

/**
 * Destructor.
 */
AutoCPDisplay::~AutoCPDisplay() {
  delete sensing_;
  delete visualization_;
  delete optimization_;
}

/**
 * Initialization. Attaches callbacks to subscribers.
 */
void AutoCPDisplay::onInitialize() {
  Display::onInitialize();
  auto visualization_manager = static_cast<rviz::VisualizationManager*>(
      context_);
  auto scene_manager = visualization_manager->getSceneManager();
  camera_ = visualization_manager->getRenderPanel()->getCamera();
  sensing_ = new AutoCPSensing(node_handle_, &tf_listener_, camera_,
                               fixed_frame_.toStdString());
  sensing_->Initialize();

  visualization_ = new Visualization(node_handle_, fixed_frame_.toStdString());

  visibility_checker_ = new VisibilityChecker(scene_manager, camera_);
  optimization_ = new Optimization(sensing_, visibility_checker_);
  target_viewpoint_ = sensing_->current_viewpoint();

  UpdateTopic();
  UpdateWeights();
}

// Parameter update handlers ---------------------------------------------------
/**
 * Set the topic to publish camera placement commands to.
 */
void AutoCPDisplay::UpdateTopic() {
  camera_placement_publisher_ = node_handle_
      .advertise<view_controller_msgs::CameraPlacement>(
      topic_prop_->getStdString(), 5);
}

/**
 * No-op for camera options.
 */
void AutoCPDisplay::UpdateCameraOptions() {
  return;
}

/**
 * Updates the weights.
 */
void AutoCPDisplay::UpdateWeights() {
  auto landmarks = sensing_->landmarks();
  landmarks->UpdateLeftGripperWeight(l_gripper_weight_->getFloat());
  landmarks->UpdateRightGripperWeight(r_gripper_weight_->getFloat());
  landmarks->UpdateHeadWeight(head_weight_->getFloat());
  landmarks->UpdateHeadFocusWeight(head_focus_weight_->getFloat());
  landmarks->UpdateSegmentedObjectWeight(segmented_object_weight_->getFloat());
  optimization_->set_visibility_weight(stay_visible_weight_->getFloat());
  optimization_->set_centering_weight(centering_weight_->getFloat());
  optimization_->set_view_angle_weight(be_orthogonal_weight_->getFloat());
  optimization_->set_zoom_weight(zoom_weight_->getFloat());
  optimization_->set_crossing_weight(crossing_weight_->getFloat());
  optimization_->set_score_threshold(score_threshold_->getFloat());
  optimization_->set_max_visibility_checks(occlusion_check_limit_->getInt());
  optimization_->set_min_zoom(min_zoom_->getFloat());
  optimization_->set_max_zoom(max_zoom_->getFloat());
}

// Camera placement logic ------------------------------------------------------
/**
 * Main loop that alternates between sensing and placing the camera.
 */
void AutoCPDisplay::update(float wall_dt, float ros_dt) {
  if (sensing_->landmarks()->NumLandmarks() == 0) {
    return;
  }

  if (state_ == State::WAITING) {
    if (sensing_->IsMouseUp()) {
      ChooseCameraPlacement(wall_dt);
      ROS_INFO("Target: %.2f, %.2f, %.2f = %s", target_viewpoint_.position().x,
               target_viewpoint_.position().y, target_viewpoint_.position().z,
               target_viewpoint_.score().toString().c_str());
      state_ = State::MOVING;
    } else {
      current_viewpoint_ = sensing_->current_viewpoint();
    }
  } else {
    if (current_viewpoint_.SamePose(target_viewpoint_)) {
      state_ = State::WAITING;
    } else {
      Viewpoint next_viewpoint;
      interpolateViewpoint(current_viewpoint_, target_viewpoint_,
                           4, wall_dt, &next_viewpoint);
      view_controller_msgs::CameraPlacement message;
      SetCameraPlacement(next_viewpoint, ros::Duration(wall_dt), &message);
      camera_placement_publisher_.publish(message);
      // Have to be careful not to assign current_viewpoint_ to the one returned
      // by sensing_->current_viewpoint(). That viewpoint has a focus point
      // that is always 1 meter out in front.
      current_viewpoint_ = next_viewpoint;
    }
  }

  if (show_fps_->getBool()) {
    ROS_INFO("FPS: %f", 1 / wall_dt);
  }
  sensing_->Update();
}

/**
 * Get the final focus point and location for the camera, then push the camera
 * placement.
 */
void AutoCPDisplay::ChooseCameraPlacement(float time_delta) {
  Score current_score;
  optimization_->ComputeViewpointScore(sensing_->current_viewpoint(),
                                       &current_score);

  std::vector<Viewpoint> target_viewpoints;
  optimization_->ChooseViewpoint(NULL, 1, &target_viewpoints);
  if (target_viewpoints.size() > 0) {
    auto top_viewpoint = target_viewpoints[0];
    auto top_score = top_viewpoint.score().score;
    auto threshold = score_threshold_->getFloat();
    if (top_score > threshold * current_score.score) {
      visualization_->ShowViewpoint(top_viewpoint);
      target_viewpoint_ = top_viewpoint;
    }
  }
}

// Utilities -------------------------------------------------------------------
/**
 * Convenience method to set the camera placement.
 */
void AutoCPDisplay::SetCameraPlacement(
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

  camera_placement->eye.point.x = viewpoint.position().x;
  camera_placement->eye.point.y = viewpoint.position().y;
  camera_placement->eye.point.z = viewpoint.position().z;

  camera_placement->focus.point.x = viewpoint.focus().x;
  camera_placement->focus.point.y = viewpoint.focus().y;
  camera_placement->focus.point.z = viewpoint.focus().z;

  camera_placement->up.vector.x = 0.0;
  camera_placement->up.vector.y = 0.0;
  camera_placement->up.vector.z = 1.0;
}

}  // namespace autocp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autocp::AutoCPDisplay, rviz::Display)
