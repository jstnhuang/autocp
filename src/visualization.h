/*
 * Visualization module.
 */

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include "score.h"
#include "utils.h"
#include "viewpoint.h"

namespace autocp {
using visualization_msgs::Marker;

static const std::string kFociNamespace = "foci";
static const std::string kCandidateViewpointNamespace = "viewpoints";
static const std::string kCandidateTextNamespace = "scores";

class Visualization {
 public:
  Visualization(const ros::NodeHandle& root_node_handle,
                std::string fixed_frame);
  void ShowViewpoints(const std::vector<Viewpoint>& viewpoints,
                 const std::vector<Score>& scores);
  void ShowFocus(const Ogre::Vector3& focus);
 private:
  ros::NodeHandle root_nh_;
  ros::Publisher marker_pub_;
  std::string fixed_frame_;
  int num_foci_;
  int num_candidate_viewpoints_;

  void FlushCandidateViewpoints();
  void FlushFoci();
  void FlushMarkers(std::string ns, int max_id);

  void MakeFocusMarker(const Ogre::Vector3& focus, Marker* marker);
  void MakeCameraMarker(const Viewpoint& viewpoint, const Score& score,
                        Marker* marker);
  void MakeTextMarker(const Viewpoint& viewpoint, const Score& score,
                      Marker* marker);
};
}

#endif
