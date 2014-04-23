/*
 * Module for doing visibility checks.
 */

#ifndef VISIBILITY_H
#define VISIBILITY_H

#include <OGRE/OgreSceneManager.h>
#include "models/viewpoint.h"

namespace autocp {

const float kOcclusionThreshold = 0.25;

class VisibilityChecker {
 public:
  VisibilityChecker(Ogre::SceneManager* scene_manager,
                    Ogre::Camera* camera);
  ~VisibilityChecker();
  bool IsVisible(const Ogre::Vector3& point, const Viewpoint& viewpoint);
  void GetScreenPosition(const Ogre::Vector3& point, float* screen_x,
                         float* screen_y);

 private:
  Ogre::SceneManager* scene_manager_;
  Ogre::RaySceneQuery* ray_scene_query_;
  Ogre::Camera* camera_;

  bool IsOnScreen(float screen_x, float screen_y);
  bool RaycastAABB(const Ogre::Ray& ray, Ogre::Vector3* hit);
};}

#endif
