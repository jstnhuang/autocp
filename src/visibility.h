/*
 * Module for doing visibility checks.
 */

#ifndef VISIBILITY_H
#define VISIBILITY_H

#include <OGRE/OgreSceneManager.h>
#include "viewpoint.h"

namespace autocp {

const float kOcclusionThreshold = 0.25;

class VisibilityChecker {
 public:
  VisibilityChecker(Ogre::SceneManager* scene_manager,
                    Ogre::Camera* camera);
  ~VisibilityChecker();
  bool IsVisible(const Ogre::Vector3& point, const Viewpoint& viewpoint);

 private:
  Ogre::SceneManager* scene_manager_;
  Ogre::RaySceneQuery* ray_scene_query_;
  Ogre::Camera* camera_;

  void GetScreenPosition(const Ogre::Vector3& point, float* screen_x,
                         float* screen_y);
  bool IsOnScreen(float screen_x, float screen_y);
  bool RaycastAABB(const Ogre::Ray& ray, Ogre::Vector3* hit);
};

#endif