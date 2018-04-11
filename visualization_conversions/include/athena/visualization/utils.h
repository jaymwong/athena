#ifndef VISUALIZATION_CONVERSIONS_UTILS_H
#define VISUALIZATION_CONVERSIONS_UTILS_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>

namespace visualization_utils{
  visualization_msgs::Marker createMarkerFromPoint(Eigen::Vector3d point, int id, std::string frame, double size);
  visualization_msgs::MarkerArray createMarkersFromPoints(std::vector<Eigen::Vector3d> points, std::string frame, double size);
};

#endif
