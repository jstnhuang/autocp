#ifndef VIEWPOINT_H
#define VIEWPOINT_H

#include <OGRE/OgreVector3.h>

namespace autocp {
struct Viewpoint {
  Ogre::Vector3 position;
  Ogre::Vector3 focus;
  Viewpoint() {
    this->position = Ogre::Vector3(0, 0, 0);
    this->focus = Ogre::Vector3(0, 0, 0);
  }
  Viewpoint(const Ogre::Vector3& position, const Ogre::Vector3& focus) {
    this->position = position;
    this->focus = focus;
  }
  bool Equals(const Viewpoint& other) {
    return other.position == position && other.focus == focus;
  }
};
}

#endif
