/*
 * Visualization module.
 */

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

#include "models/score.h"
#include "models/viewpoint.h"
#include "utils.h"

namespace autocp {
using visualization_msgs::Marker;

static const std::string kFociNamespace = "foci";
static const std::string kCandidateViewpointNamespace = "viewpoints";
static const std::string kCandidateTextNamespace = "scores";
static const std::string kCurrentViewpointNamespace = "current_viewpoint";

class Visualization {
 public:
  Visualization(const ros::NodeHandle& root_node_handle,
                std::string fixed_frame);
  void ShowViewpoints(const std::vector<Viewpoint>& viewpoints,
                 const std::vector<Score>& scores);
  void ShowFocus(const Ogre::Vector3& focus);
  void ShowViewpoint(const Viewpoint& viewpoint);
  void MakeCameraMarker(const Viewpoint& viewpoint, float score,
                        Marker* marker);
 private:
  ros::NodeHandle root_nh_;
  ros::Publisher marker_pub_;
  std::string fixed_frame_;
  int num_foci_;
  int num_candidate_viewpoints_;
  int current_viewpoint_id_;

  void FlushCandidateViewpoints();
  void FlushFoci();
  void FlushViewpoint();
  void FlushMarkers(std::string ns, int max_id);

  void MakeFocusMarker(const Ogre::Vector3& focus, Marker* marker);
  void MakeTextMarker(const Viewpoint& viewpoint, const Score& score,
                      Marker* marker);
  void MakeCurrentViewpointMarker(const Viewpoint& viewpoint, Marker* marker);
};
}

#endif
