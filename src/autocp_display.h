#ifndef AUTOCP_DISPLAY_H
#define AUTOCP_DISPLAY_H

#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/message_filter_display.h>
#include <sensor_msgs/Imu.h>
#include <view_controller_msgs/CameraPlacement.h>

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;
}

namespace autocp {
class AutoCPDisplay: public rviz::MessageFilterDisplay<sensor_msgs::Imu> {
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
  void updateCamera(const ros::TimerEvent&);
  void processMessage(const sensor_msgs::Imu::ConstPtr& msg);
  ros::Publisher pub_;
  rviz::RosTopicProperty* topic_prop_;
  rviz::FloatProperty* rpm_prop_;
  rviz::FloatProperty* fps_prop_;
  ros::Timer timer_;
  float camera_radians_;
};
}

#endif
