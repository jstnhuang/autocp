/*
 * Visualization module.
 */

#include "visualization.h"

namespace autocp {
using visualization_msgs::Marker;

/*
 * Constructor.
 *
 * Input:
 *   root_node_handle: The root node handle of the display.
 */
Visualization::Visualization(const ros::NodeHandle& root_node_handle,
                             std::string fixed_frame) {
  root_nh_ = root_node_handle;
  fixed_frame_ = fixed_frame;
  marker_pub_ = root_nh_.advertise<Marker>("autocp_markers", 1);
  num_foci_ = 0;
  num_candidate_viewpoints_ = 0;
}

void Visualization::ShowViewpoints(const std::vector<Viewpoint>& viewpoints,
                                   const std::vector<Score>& scores) {
  FlushCandidateViewpoints();
  for (int i = 0; i < viewpoints.size(); i++) {
    num_candidate_viewpoints_ = i;
    auto viewpoint = viewpoints[i];
    auto score = scores[i];
    Marker viewpoint_marker;
    Marker text_marker;
    MakeCameraMarker(viewpoint, score, &viewpoint_marker);
    MakeTextMarker(viewpoint, score, &text_marker);
    marker_pub_.publish(viewpoint_marker);
    marker_pub_.publish(text_marker);
  }
}

void Visualization::ShowFocus(const Ogre::Vector3& focus) {
  FlushFoci();
  Marker marker;
  MakeFocusMarker(focus, &marker);
  marker_pub_.publish(marker);
  num_foci_++;
}

void Visualization::FlushCandidateViewpoints() {
  FlushMarkers(kCandidateViewpointNamespace, num_candidate_viewpoints_);
  FlushMarkers(kCandidateTextNamespace, num_candidate_viewpoints_);
  num_candidate_viewpoints_ = 0;

}

void Visualization::FlushFoci() {
  FlushMarkers(kFociNamespace, num_foci_);
  num_foci_ = 0;
}

/**
 * Delete all markers in the given namespace, with ids = 0, 1, ..., max_id.
 */
void Visualization::FlushMarkers(std::string ns, int max_id) {
  for (int i = 0; i < max_id; i++) {
    Marker marker;
    marker.header.frame_id = fixed_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = i;
    marker.action = Marker::DELETE;
    marker_pub_.publish(marker);
  }
}

/**
 * Creates an arrow marker pointing towards the focus point. It is colored green
 * if the score is 1, red if the score is 0, and something in between otherwise.
 */
void Visualization::MakeCameraMarker(const Viewpoint& viewpoint,
                                     const Score& score, Marker* marker) {
  Ogre::Quaternion orientation;
  ViewpointToOrientation(viewpoint, &orientation);
  marker->header.frame_id = fixed_frame_;
  marker->header.stamp = ros::Time::now();
  marker->ns = kCandidateViewpointNamespace;
  marker->id = num_candidate_viewpoints_;
  marker->type = Marker::ARROW;
  marker->action = Marker::ADD;
  marker->pose.position.x = viewpoint.position.x;
  marker->pose.position.y = viewpoint.position.y;
  marker->pose.position.z = viewpoint.position.z;
  marker->pose.orientation.x = orientation.x;
  marker->pose.orientation.y = orientation.y;
  marker->pose.orientation.z = orientation.z;
  marker->pose.orientation.w = orientation.w;
  marker->scale.x = 0.05;
  marker->scale.y = 0.05;
  marker->scale.z = 0.05;
  if (score.score < 0) {
    marker->color.r = 0;
    marker->color.g = 0;
    marker->color.b = 0;
  } else {
    marker->color.r = 1.0 - score.score;
    marker->color.g = score.score;
    marker->color.b = 0.0f;
  }
  marker->color.a = 1.0;
  marker->lifetime = ros::Duration();
}

void Visualization::MakeTextMarker(const Viewpoint& viewpoint,
                                   const Score& score, Marker* marker) {
  marker->header.frame_id = fixed_frame_;
  marker->header.stamp = ros::Time::now();
  marker->ns = kCandidateTextNamespace;
  marker->id = num_candidate_viewpoints_;
  marker->type = Marker::TEXT_VIEW_FACING;
  marker->action = Marker::ADD;
  marker->pose.position.x = viewpoint.position.x;
  marker->pose.position.y = viewpoint.position.y;
  marker->pose.position.z = viewpoint.position.z - 0.1;
  marker->scale.z = 0.05;
  marker->color.r = 0;
  marker->color.g = 0.48;
  marker->color.b = 0.68;
  marker->color.a = 1.0;
  marker->lifetime = ros::Duration();
  marker->text = score.toString();
}

void Visualization::MakeFocusMarker(const Ogre::Vector3& focus,
                                    Marker* marker) {
  marker->header.frame_id = fixed_frame_;
  marker->header.stamp = ros::Time::now();
  marker->ns = kFociNamespace;
  marker->id = num_foci_;
  marker->type = Marker::SPHERE;
  marker->action = Marker::ADD;
  marker->pose.position = toPoint(focus);
  marker->scale.x = 0.1;
  marker->scale.y = 0.1;
  marker->scale.z = 0.1;
  marker->color.r = 0.0;
  marker->color.g = 0.0;
  marker->color.b = 1.0;
  marker->color.a = 1.0;
  marker->lifetime = ros::Duration();
}
}

