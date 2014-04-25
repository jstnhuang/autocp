#include <OGRE/OgreVector3.h>

#include "autocp/models/viewpoint.h"

namespace autocp {

Viewpoint::Viewpoint()
    : position_(Ogre::Vector3::ZERO),
      focus_(Ogre::Vector3::ZERO),
      score_() {
}

Viewpoint::Viewpoint(const Ogre::Vector3& position, const Ogre::Vector3& focus)
    : position_(position),
      focus_(focus),
      score_() {
}

Ogre::Vector3 Viewpoint::position() const {
  return position_;
}

Ogre::Vector3 Viewpoint::focus() const {
  return focus_;
}

Score Viewpoint::score() const {
  return score_;
}

void Viewpoint::set_position(const Ogre::Vector3& position) {
  position_ = position;
}

void Viewpoint::set_focus(const Ogre::Vector3& focus) {
  focus_ = focus;
}

void Viewpoint::set_score(const Score& score) {
  score_ = score;
}

bool Viewpoint::SamePose(const Viewpoint& other) {
  return position_.distance(other.position()) < 0.01
      && focus_.distance(other.focus()) < 0.01;
}

}
