#include "athena/pointcloud/planar.h"

athena::pointcloud::PlanarModel::PlanarModel(double a, double b, double c, double d){
  this->a = a;
  this->b = b;
  this->c = c;
  this->d = d;
}
athena::pointcloud::PlanarModel::~PlanarModel(){}


athena::pointcloud::ClosestPointResult athena::pointcloud::PlanarModel::getClosestPointOnPlane(Eigen::Vector3d point){
  double k = (this->d + (this->a*point.x()) + (this->b*point.y()) + (this->c*point.z())) / (pow(this->a,2) + pow(this->b,2) + pow(this->c,2));

  athena::pointcloud::ClosestPointResult result;
  result.closest_pt.x() = point.x() - (k*this->a);
  result.closest_pt.y() = point.y() - (k*this->b);
  result.closest_pt.z() = point.z() - (k*this->c);
  result.distance = (result.closest_pt - point).norm();

  return result;
}
