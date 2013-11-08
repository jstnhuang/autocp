#ifndef AUTOCP_DISPLAY_H
#define AUTOCP_DISPLAY_H

#include <pr2_controllers_msgs/PointHeadAction.h>
#include <ros/ros.h>
#include <rviz/display.h>
#include <view_controller_msgs/CameraPlacement.h>

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;
}

namespace autocp {
class AutoCPDisplay: public rviz::Display {
 Q_OBJECT
 public:
  AutoCPDisplay();
  virtual ~AutoCPDisplay();

  protected:
  virtual void onInitialize();

 private Q_SLOTS:
  void updateTopic();
  void updateRpm();

 private:
  ros::NodeHandle root_nh_;
  void setCameraPlacement(
    float eye_x, float eye_y, float eye_z,
    float focus_x, float focus_y, float focus_z,
    ros::Duration time_from_start,
    view_controller_msgs::CameraPlacement& cp
  );
  void targetPointCallback(
    const pr2_controllers_msgs::PointHeadActionGoal& goal
  );
  ros::Publisher pub_;
  ros::Subscriber sub_;
  rviz::RosTopicProperty* topic_prop_;
};
}

#endif
