#ifndef CLICKED_CONTROL_H
#define CLICKED_CONTROL_H

#include <geometry_msgs/Pose.h>
#include <OGRE/OgreVector3.h>

#include "control_6dof.h"

namespace autocp {

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

}

#endif
