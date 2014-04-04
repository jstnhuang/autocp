#ifndef VIEWPOINT_H
#define VIEWPOINT_H

#include <OGRE/OgreVector3.h>

namespace autocp {
struct Viewpoint {
  Ogre::Vector3 position;
  Ogre::Vector3 focus;
  Viewpoint(const Ogre::Vector3& position, const Ogre::Vector3& focus) {
    this->position = position;
    this->focus = focus;
  }
};
}

#endif
