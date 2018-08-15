#include "athena/pointcloud/registration.h"

RegistrationResult athena::pointcloud::performTemplateBasedRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, RegistrationOptions options){
  FeatureCloud target_cloud, model_cloud;
  target_cloud.setInputCloud (scene);
  model_cloud.setInputCloud (model);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  template_align.setMaximumIterations(options.max_iterations);
  template_align.addTemplateCloud (model_cloud);
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  template_align.findBestAlignment (best_alignment);

  RegistrationResult result;
  result.transform = best_alignment.final_transformation.cast<double>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*model, *result_cloud, result.transform);
  result.cloud = result_cloud;
  return result;
}

RegistrationResult athena::pointcloud::performPointBasedIterativeRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene){
  RegistrationResult result;
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations (1);
  icp.setInputSource (model);
  icp.setInputTarget (scene);
  icp.align (*result_cloud);

  if (icp.hasConverged ()) {
    result.transform = icp.getFinalTransformation ().cast<double>();
    pcl::transformPointCloud(*model, *result_cloud, result.transform);
  }
  else{ PCL_ERROR ("\nICP has not converged.\n"); }
  result.cloud = result_cloud;
  return result;
}

// for every point in source cloud find if the nearest neighbor is within 0.005m and update the count
int athena::pointcloud::findNumCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, double thresh) {
  int count = 0; int K = 1;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (target_cloud);
  for (int i = 0; i < source_cloud->points.size (); i++) {
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree.nearestKSearch (source_cloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
      if (sqrt(pointNKNSquaredDistance[0]) < thresh) { count += 1; }
    }
  }
  return count;
}

CorrespondenceScoreResult athena::pointcloud::computeCorrespondenceScore(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, double thresh) {
  int count_s_to_m = athena::pointcloud::findNumCorrespondences(source_cloud, target_cloud, thresh);
  int count_m_to_s = athena::pointcloud::findNumCorrespondences(target_cloud, source_cloud, thresh);
  CorrespondenceScoreResult result;
  result.s_to_m  = double(count_s_to_m) / (double)(source_cloud->points.size());
  result.m_to_s = double(count_m_to_s) / (double)(target_cloud->points.size());
  return result;
}
