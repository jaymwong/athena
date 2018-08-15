#include "athena/pointcloud/segmentation.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::removeNegativeWorldPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, Eigen::Matrix4d cam_to_world){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*input_cloud, *out_cloud, cam_to_world);
  out_cloud = athena::pointcloud::doPassThroughFilter(out_cloud, "z", 0.0, kInfinity);
  Eigen::Matrix4d world_to_cam = cam_to_world.inverse();   // Transform the cloud back into camera frame
  pcl::transformPointCloud(*out_cloud, *out_cloud, world_to_cam);
  return out_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::doPassThroughCubeCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4d wTc, pcl::PointXYZ pt, double radius){
  pcl::PointCloud<pcl::PointXYZ>::Ptr result = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*cloud, *result);
  pcl::transformPointCloud(*result, *result, wTc);

  // Now transform the camera_frame point into the world frame; then do the crop
  Eigen::Vector3d world_pt = athena::transform::transform_point(wTc, athena::conversions::toEigenVector3d(pt));
  result = athena::pointcloud::doPassThroughCubeCrop(result, athena::conversions::toPclPointXYZ(world_pt), radius);

  // Transform the cloud back into the camera frame to be visualized
  Eigen::Matrix4d cTw = wTc.inverse();
  pcl::transformPointCloud(*result, *result, cTw);
  return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::doPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string axis, double min, double max){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(axis);
  pass.setFilterLimits(min, max);
  pass.filter(*out_cloud);
  return out_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::doPassThroughCubeCrop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pt, double radius){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  out_cloud = athena::pointcloud::doPassThroughFilter(cloud, "x", pt.x-radius, pt.x+radius);
  out_cloud = athena::pointcloud::doPassThroughFilter(out_cloud, "y", pt.y-radius, pt.y+radius);
  out_cloud = athena::pointcloud::doPassThroughFilter(out_cloud, "z", pt.z-radius, pt.z+radius);
  return out_cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::naivePointCloudProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (pcl::PointXYZ pt: cloud->points){
    pcl::PointXYZ new_pt(pt.x, pt.y, 0.0);
    out_cloud->points.push_back(new_pt);
  }
  return out_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::removeSubCloudFromOriginalCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_, pcl::PointXYZ pt,
                                       double radius_x, double radius_y, double radius_z) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  inliers = athena::pointcloud::findCloudInliers(original_cloud_, pt, radius_x, radius_y, radius_z);
  std::cout << "No.of inliers: " << inliers->indices.size() << "\n";

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(original_cloud_);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*original_cloud_);
  return original_cloud_;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::getCloudPointsWithinStdDev(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double thresh){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (input_cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (thresh);
  sor.filter (*out_cloud);
  return out_cloud;
}
