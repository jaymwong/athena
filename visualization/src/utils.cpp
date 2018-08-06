#include "athena/visualization/utils.h"

visualization_msgs::Marker athena::visualization::createMarkerFromPoint(Eigen::Vector3d point, int id, std::string frame, double size){
  return createMarkerFromPoint(point, id, frame, size, toColorRGBA(1.0, 0.0, 0.0, 1.0));
}

visualization_msgs::Marker athena::visualization::createMarkerFromPoint(Eigen::Vector3d point, int id, std::string frame, double size, std_msgs::ColorRGBA color){
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.ns = std::to_string(id);
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = point.x();
  marker.pose.position.y = point.y();
  marker.pose.position.z = point.z();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color = color;
  return marker;
}

visualization_msgs::MarkerArray athena::visualization::createMarkersFromPoints(std::vector<Eigen::Vector3d> points, std::string frame, double size, std_msgs::ColorRGBA color){
  visualization_msgs::MarkerArray marker_array;
  for (int i = 0; i < points.size(); i++){
    marker_array.markers.push_back(createMarkerFromPoint(points.at(i), i, frame, size, color));
  }
  return marker_array;
}

visualization_msgs::MarkerArray athena::visualization::createMarkersFromPoints(std::vector<Eigen::Vector3d> points, std::string frame, double size){
  visualization_msgs::MarkerArray marker_array;
  for (int i = 0; i < points.size(); i++){
    marker_array.markers.push_back(createMarkerFromPoint(points.at(i), i, frame, size));
  }
  return marker_array;
}

std_msgs::ColorRGBA athena::visualization::toColorRGBA(double r, double g, double b, double a){
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

visualization_msgs::Marker athena::visualization::createBoundingBoxMarker(int id, Eigen::Vector3f point, Eigen::Quaternionf quat, Eigen::Vector3f size, std::string frame){
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame;
  marker.ns = std::to_string(id);
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = point.x();
  marker.pose.position.y = point.y();
  marker.pose.position.z = point.z();
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.z();
  marker.scale.x = size(0);
  marker.scale.y = size(1);
  marker.scale.z = size(2);
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  return marker;
}
