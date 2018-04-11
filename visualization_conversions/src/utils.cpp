#include "athena/visualization/utils.h"

visualization_msgs::Marker visualization_utils::createMarkerFromPoint(Eigen::Vector3d point, int id, std::string frame, double size){
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.ns = std::to_string(id);
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = point.x();
  marker.pose.position.y = point.y();
  marker.pose.position.z = point.z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  return marker;
}


visualization_msgs::MarkerArray visualization_utils::createMarkersFromPoints(std::vector<Eigen::Vector3d> points, std::string frame, double size){
  visualization_msgs::MarkerArray marker_array;
  for (int i = 0; i < points.size(); i++){
    marker_array.markers.push_back(createMarkerFromPoint(points.at(i), i, frame, size));
  }
  return marker_array;
}
