#include "autocp_display.h"
#include <string>
#include <math.h>
#include <OGRE/OgreCamera.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>

namespace autocp {
/**
 * Constructor. Hooks up the display properties.
 */
AutoCPDisplay::AutoCPDisplay(): root_nh_("") {
  topic_prop_ = new rviz::RosTopicProperty(
    "Command topic",
    "/rviz/camera_placement",
    QString::fromStdString(
      ros::message_traits::datatype<view_controller_msgs::CameraPlacement>()
    ),
    "Topic on which to send out camera placement messages.",
    this,
    SLOT(updateTopic())
  );
  last_position_ = 0;
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
  Display:: onInitialize();
  updateTopic();

  point_head_subscriber_ = root_nh_.subscribe(
    "head_traj_controller/point_head_action/goal",
    5,
    &AutoCPDisplay::pointHeadCallback,
    this
  );

  // TODO(jstn): Just in case it becomes useful.
  vm_ = static_cast<rviz::VisualizationManager*>(context_);
}

/**
 * Main loop that alternates between sensing and placing the camera.
 */
void AutoCPDisplay::update(float wall_dt, float ros_dt) {
  sense();
  chooseCameraPlacement(ros_dt);
}

/**
 * Set the topic to publish camera placement commands to.
 */
void AutoCPDisplay::updateTopic() {
  camera_placement_publisher_ =
    root_nh_.advertise<view_controller_msgs::CameraPlacement>(
      topic_prop_->getStdString(),
      5
    );
}

/**
 * Update the points of interest.
 */
void AutoCPDisplay::sense() {
  getTransformOrigin("/l_wrist_roll_link", &left_gripper_origin_);
  getTransformOrigin("/r_wrist_roll_link", &right_gripper_origin_);
}

/**
 * Get the origin of the given transform.
 */
void AutoCPDisplay::getTransformOrigin(
  std::string frame,
  geometry_msgs::Point* origin
) {
  ros::Duration timeout(5);
  tf_listener_.waitForTransform("/base_link", frame, ros::Time(0), timeout);
  tf::StampedTransform transform;
  tf_listener_.lookupTransform("/base_link", frame, ros::Time(0), transform);
  tf::Vector3 transform_origin = transform.getOrigin();

  origin->x = transform_origin.x();
  origin->y = transform_origin.y();
  origin->z = transform_origin.z();
}

/**
 * Get the target point for the head.
 */
void AutoCPDisplay::pointHeadCallback(
  const pr2_controllers_msgs::PointHeadActionGoal& action_goal
) {
  head_focus_point_ = action_goal.goal.target.point;
}

/**
 * Get the final focus point and location for the camera, then push the camera
 * placement.
 */
void AutoCPDisplay::chooseCameraPlacement(float time_delta) {
  geometry_msgs::Point focus;
  chooseCameraFocus(&focus);

  geometry_msgs::Point location;
  chooseCameraLocation(&location);

  view_controller_msgs::CameraPlacement camera_placement;
  setCameraPlacement(
    location, focus,
    ros::Duration(time_delta),
    &camera_placement
  );

  camera_placement_publisher_.publish(camera_placement);
}

/**
 * Choose a final focus point for the camera. The final focus point is the mean
 * of the current focus points.
 */
void AutoCPDisplay::chooseCameraFocus(geometry_msgs::Point* focus) {
  geometry_msgs::Point* points[] {
    &left_gripper_origin_,
    &right_gripper_origin_,
    &head_focus_point_
  };
  int num_points = 3;

  float mean_x = 0;
  float mean_y = 0;
  float mean_z = 0;
  for (int i = 0; i < num_points; i++) {
    geometry_msgs::Point* point = points[i];
    mean_x += point->x;
    mean_y += point->y;
    mean_z += point->z;
  }
  mean_x /= num_points;
  mean_y /= num_points;
  mean_z /= num_points;

  focus->x = mean_x;
  focus->y = mean_y;
  focus->z = mean_z;
}

/**
 * Choose a location for the camera. The camera should be located at some point
 * on a hemisphere, centered on the robot. We will pick some random points near
 * the current camera position, and pick one which can see all the focus points.
 * Ties are broken based on which is closest to the current camera position.
 */
void AutoCPDisplay::chooseCameraLocation(geometry_msgs::Point* location) {
  Ogre::Vector3 position = vm_->getRenderPanel()->getCamera()->getPosition();
  location->x = round(2*position.x)/2;
  location->y = round(2*position.y)/2;
  location->z = round(2*position.z)/2;
}

/**
 * Convenience method to set the camera placement.
 */
void AutoCPDisplay::setCameraPlacement(
  const geometry_msgs::Point& location,
  const geometry_msgs::Point& focus,
  const ros::Duration& time_from_start,
  view_controller_msgs::CameraPlacement* camera_placement
) {
  camera_placement->target_frame = "<Fixed Frame>";

  camera_placement->time_from_start = time_from_start;

  camera_placement->eye.header.stamp = ros::Time::now();
  camera_placement->eye.header.frame_id = "base_link";
  camera_placement->focus.header.stamp = ros::Time::now();
  camera_placement->focus.header.frame_id = "base_link";
  camera_placement->up.header.stamp = ros::Time::now();
  camera_placement->up.header.frame_id = "base_link";

  camera_placement->eye.point.x = location.x;
  camera_placement->eye.point.y = location.y;
  camera_placement->eye.point.z = location.z;

  camera_placement->focus.point.x = focus.x;
  camera_placement->focus.point.y = focus.y;
  camera_placement->focus.point.z = focus.z;

  camera_placement->up.vector.x = 0.0;
  camera_placement->up.vector.y = 0.0;
  camera_placement->up.vector.z = 1.0;
}
}  // namespace autocp

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autocp::AutoCPDisplay, rviz::Display)
