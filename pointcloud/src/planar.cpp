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
  delete sac_seg;

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

athena::pointcloud::ProcessPointCloudResult athena::pointcloud::PlanarModel::processPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double thresh, Eigen::Matrix4d to_world_frame){
  athena::pointcloud::ProcessPointCloudResult result;
  result.inliers = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  result.outliers = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i <  cloud->points.size(); i++){
    auto closest_pt_result = this->getClosestPointOnPlane(cloud->points[i]);
    if (closest_pt_result.distance < thresh ){
      result.inliers->points.push_back(cloud->points[i]);
      continue;
    }

    // Check if the transform is close to identity if not we will use it to further determine sign
    if ((Eigen::Matrix4d::Identity() - to_world_frame).norm() < 0.001){
      result.outliers->points.push_back(cloud->points[i]);
      continue;
    }

    // Finally check the closest point in the world frame to determine sign
    Eigen::Vector3d pt_w = athena::transform::transform_point(to_world_frame, athena::conversions::toEigenVector3d(cloud->points[i]));
    Eigen::Vector3d closest_pt_w = athena::transform::transform_point(to_world_frame, closest_pt_result.pt);
    if (closest_pt_w.z() > pt_w.z()) { result.inliers->points.push_back(cloud->points[i]); }
    else { result.outliers->points.push_back(cloud->points[i]); }

  }
  return result;
}


athena_msgs::PlanarModel athena::conversions::toPlanarModelMsg(athena::pointcloud::PlanarModel *model, std::string frame_id){
  athena_msgs::PlanarModel msg;
  msg.a = model->a;
  msg.b = model->b;
  msg.c = model->c;
  msg.d = model->d;
  msg.header.frame_id = frame_id;
  return msg;
}


athena::pointcloud::PlanarModel* athena::conversions::fromPlanarModelmsg(athena_msgs::PlanarModel msg){
  return new athena::pointcloud::PlanarModel(msg.a, msg.b, msg.c, msg.d);
}
