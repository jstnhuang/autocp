#ifndef VIEWPOINT_H
#define VIEWPOINT_H

#include <OGRE/OgreVector3.h>
#include <boost/function.hpp>
#include "autocp/models/score.h"

namespace autocp {

class Viewpoint {
 public:
  Viewpoint();
  Viewpoint(const Ogre::Vector3& position, const Ogre::Vector3& focus);
  Ogre::Vector3 position() const;
  Ogre::Vector3 focus() const;
  Score score() const;
  void set_position(const Ogre::Vector3& position);
  void set_focus(const Ogre::Vector3& focus);
  void set_score(const Score& score);
  struct HasHigherScore {
    bool operator()(const Viewpoint& left, const Viewpoint& right) {
      return left.score().score > right.score().score;
    }
  };

 private:
  Ogre::Vector3 position_;
  Ogre::Vector3 focus_;
  Score score_;
};

}

#endif
