/*
 * Module for doing visibility checks.
 */

#include "visibility.h"

namespace autocp {
/*
 * Constructor.
 *
 * Input:
 *   scene_manager: The OGRE scene manager we're operating within.
 */
VisibilityChecker::VisibilityChecker(Ogre::SceneManager* scene_manager) {
  scene_manager_ = scene_manager;
  ray_scene_query_ = scene_manager_->createRayQuery(
      Ogre::Ray(),
      Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);
}

/*
 * Destructor.
 */
VisibilityChecker::~VisibilityChecker() {
  scene_manager_->destroyQuery(ray_scene_query_);
}

/*
 * Returns true if the given point is visible from the given viewpoint.
 *
 * A point is visible if it will be on screen, and if it is not occluded by
 * other objects.
 *
 * Input:
 *   point: The point whose visibility we want to check.
 *   viewpoint: The viewpoint to check from.
 *
 * Returns: True if the point is visible from the viewpoint.
 */
bool VisibilityChecker::IsVisible(const Ogre::Vector3& point,
                                  const Viewpoint& viewpoint) {
  float screen_x;
  float screen_y;
  auto camera = scene_manager_->createCamera("IsVisible");
  camera->setPosition(viewpoint.position);
  camera->lookAt(viewpoint.focus);
  GetScreenPosition(point, *camera, &screen_x, &screen_y);
  bool result = false;
  if (IsOnScreen(screen_x, screen_y)) {
    Ogre::Ray ray;
    camera->getCameraToViewportRay(screen_x, screen_y, &ray);
    Ogre::Vector3 hit;
    if (RaycastAABB(ray, &hit)) {
      if (point.distance(hit) < OCCLUSION_THRESHOLD) {
        result = true;
      } else { // Hit something, but too far away from the target.
        result = false;
      }
    } else {
      // No hits. The ray should hit the target, but if it doesn't, that usually
      // indicates visibility. This is because if the target is occluded, the
      // ray is likely to have hit the occluding object.
      result = true;
    }
  } else { // Not on screen
    result= false;
  }
  scene_manager_->destroyCamera(camera);
  return result;
}

/*
 * Gets the (x, y) position on the screen of the given 3D point.
 *
 * The coordinates are in the range [0, 1], with the origin in the top left
 * corner. The x-axis is width of the screen, and y-axis is the height.
 *
 * Input:
 *   point: The 3D point whose screen position we want.
 *   camera: The camera we're looking from.
 *
 * Output:
 *   screen_x: The x position of the point on the screen.
 *   screen_y: The y position of the point on the screen.
 */
void VisibilityChecker::GetScreenPosition(const Ogre::Vector3& point,
                                          const Ogre::Camera& camera,
                                          float* screen_x, float* screen_y) {
  // This projection returns x and y in the range of [-1, 1]. The (-1, -1) point
  // is in the bottom left corner.
  Ogre::Vector4 point4(point.x, point.y, point.z, 1);
  Ogre::Vector4 projected = camera.getProjectionMatrix()
      * camera.getViewMatrix() * point4;
  projected = projected / projected.w;

  *screen_x = (projected.x + 1) / 2;
  *screen_y = (1 - projected.y) / 2;
}

/*
 * Returns true if the given (x, y) coordinates are in bounds.
 */
bool VisibilityChecker::IsOnScreen(float screen_x, float screen_y) {
  if (screen_x < 0 || screen_y < 0 || screen_x > 1 || screen_y > 1) {
    return false;
  } else {
    return true;
  }
}

/*
 * Heuristic method for raycasting using axis-aligned bounding boxes.
 *
 * The output of this method is the location the ray hit the median bounding
 * box, when the bounding boxes are sorted in order of the distance to the ray.
 * This is not very accurate, and is designed to determine if the ray hits a
 * point that is "close enough" to some target point. For polygon-level
 * raytracing, see the * OGRE Wiki at goo.gl/YLKQEo.
 *
 * Input:
 *   ray: The ray to check.
 *
 * Output:
 *   hit: The median AABB point the ray hit, if any.
 *
 * Returns: True if the ray hit anything.
 */
bool VisibilityChecker::RaycastAABB(const Ogre::Ray& ray, Ogre::Vector3* hit) {
  if (!ray_scene_query_) {
    return false;
  }
  ray_scene_query_->clearResults();
  ray_scene_query_->setRay(ray);
  Ogre::RaySceneQueryResult& query_results = ray_scene_query_->execute();
  if (query_results.size() <= 0) {
    return false;
  }

  // The query results are ordered by distance along the ray. However, for some
  // reason, some of the results have zero distance. We ignore these when
  // finding the median.
  int first_nonzero = -1;
  for (int i = 0; i < query_results.size(); i++) {
    auto result = query_results[i];
    if (result.distance > 0) {
      first_nonzero = i;
      break;
    }
  }
  if (first_nonzero < 0) {
    return false;
  }
  int median_index = first_nonzero + (query_results.size() - first_nonzero) / 2;
  auto median_result = query_results[median_index];
  *hit = ray.getPoint(median_result.distance);
  return true;
}
}
