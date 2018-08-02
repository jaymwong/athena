#include "athena/pointcloud/planar.h"


athena::pointcloud::PlaneEstimationResult athena::pointcloud::estimatePlanarModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double thresh){
  std::cout << "Estimating planar model...\n";
  PlaneEstimationResult result;

  pcl::SACSegmentation<pcl::PointXYZ> *sac_seg = new pcl::SACSegmentation<pcl::PointXYZ>;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  sac_seg->setOptimizeCoefficients (true); // Optional
  sac_seg->setModelType (pcl::SACMODEL_PLANE);
  sac_seg->setMethodType (pcl::SAC_RANSAC);
  sac_seg->setMaxIterations (1000);
  sac_seg->setDistanceThreshold (thresh);

  sac_seg->setInputCloud ((*cloud).makeShared ());
  sac_seg->segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0) {
    ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.");
    result.success = false;
    return result;
  }

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud ((*cloud).makeShared ());
  extract.setIndices(inliers);
  extract.setNegative (false);

  // Get the points associated with the planar surface
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  extract.filter (*plane_cloud);

  result.plane_cloud = plane_cloud;
  result.coefficients = coefficients;
  result.success = true;
  std::cout << "Success.\n";

  coefficients.reset();
  inliers.reset();

  return result;
}


athena::pointcloud::PlanarModel* athena::pointcloud::createPlanarModelFromCoefficients(pcl::ModelCoefficients::Ptr coefficients){
  return new athena::pointcloud::PlanarModel(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
}

athena::pointcloud::PlanarModel::PlanarModel(double a, double b, double c, double d){
  this->a = a;
  this->b = b;
  this->c = c;
  this->d = d;
}
athena::pointcloud::PlanarModel::~PlanarModel(){}


athena::pointcloud::ClosestPointResult athena::pointcloud::PlanarModel::getClosestPointOnPlane(pcl::PointXYZ point){
  Eigen::Vector3d pt(point.x, point.y, point.z);
  return getClosestPointOnPlane(pt);
}
athena::pointcloud::ClosestPointResult athena::pointcloud::PlanarModel::getClosestPointOnPlane(Eigen::Vector3d point){
  double k = (this->d + (this->a*point.x()) + (this->b*point.y()) + (this->c*point.z())) / (pow(this->a,2) + pow(this->b,2) + pow(this->c,2));

  athena::pointcloud::ClosestPointResult result;
  result.pt.x() = point.x() - (k*this->a);
  result.pt.y() = point.y() - (k*this->b);
  result.pt.z() = point.z() - (k*this->c);
  result.distance = (result.pt - point).norm();
  result.signed_distance = getPointRelativeDirection(point)*result.distance;
  pcl::PointXYZ pcl_pt(result.pt.x(), result.pt.y(), result.pt.z());
  result.pcl_pt = pcl_pt;
  return result;
}

int athena::pointcloud::PlanarModel::getPointRelativeDirection(pcl::PointXYZ point){
  Eigen::Vector3d pt(point.x, point.y, point.z);
  return athena::pointcloud::PlanarModel::getPointRelativeDirection(pt);
}

int athena::pointcloud::PlanarModel::getPointRelativeDirection(Eigen::Vector3d point){
  if ( (this->a*point.x() + this->b*point.y() + this->c*point.z() + this->d) > 0){
    return 1;
  }
  return -1;
}

athena::pointcloud::ProcessPointCloudResult athena::pointcloud::PlanarModel::processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double thresh){
  athena::pointcloud::ProcessPointCloudResult result;
  result.inliers = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  result.outliers = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i <  cloud->points.size(); i++){
    auto closest_pt_result = this->getClosestPointOnPlane(cloud->points[i]);
    if (closest_pt_result.distance < thresh ){
      result.inliers->points.push_back(cloud->points[i]);
    }
    else if (!this->getPointRelativeDirection(cloud->points[i])){
      result.inliers->points.push_back(cloud->points[i]);
    }
    else {
      result.outliers->points.push_back(cloud->points[i]);
    }
  }
  return result;
}
