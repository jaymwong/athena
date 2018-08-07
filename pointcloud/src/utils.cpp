#include "athena/pointcloud/utils.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::curatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  // Remove the Nans from the point cloud (according to https://stackoverflow.com/questions/47233474/icp-segmentation-fault-pcl
  // it is best to first transform the cloud to an unorganized cloud before curating)
  ROS_WARN("[athena/curatePointCloud] > should only be used in the camera optical frame!");
  cloud->width    = cloud->width * cloud->height;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  std::cout << "Cloud size: " << cloud->points.size() << "\n";

  // Remove points that are too close to eachother (within a mm)
  pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  pcl::toPCLPointCloud2(*cloud, *cloud_in);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_in);
  sor.setLeafSize (0.001f, 0.001f, 0.001f);
  sor.filter (*cloud_filtered);
  pcl::fromPCLPointCloud2(*cloud_filtered, *cloud);
  std::cout << "Cloud size: " << cloud->points.size() << "\n";

  // Remove points that make no sense relative to the camera (behind it or too close)
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.1, kInfinity);
  pass.filter (*cloud);
  std::cout << "Cloud size: " << cloud->points.size() << "\n";
  return cloud;
}

// Computes the median of a point cloud
Eigen::Vector3d athena::pointcloud::computePointCloudMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  std::vector<double> cloud_x, cloud_y, cloud_z;
  for (int i = 0; i < cloud->points.size(); i++){
    pcl::PointXYZ pt = cloud->points[i];
    cloud_x.push_back(pt.x);
    cloud_y.push_back(pt.y);
    cloud_z.push_back(pt.z);
    // std::cout << "  [athena] > point " << pt.x << " " << pt.y << " " << pt.z << "\n";
  }
  // std::cout << "  [athena] > point cloud size: " << cloud->points.size() << " (" << cloud_x.size() << " " << cloud_y.size() << " " << cloud_z.size() << ")\n";
  return computePointCloudMedian(cloud_x, cloud_y, cloud_z);
}

Eigen::Vector3d athena::pointcloud::computePointCloudMedian(std::vector<double> cluster_pt_x, std::vector<double> cluster_pt_y, std::vector<double> cluster_pt_z){

  sort(cluster_pt_x.begin(), cluster_pt_x.end());
  sort(cluster_pt_y.begin(), cluster_pt_y.end());
  sort(cluster_pt_z.begin(), cluster_pt_z.end());

  Eigen::Vector3d median( cluster_pt_x.at(cluster_pt_x.size()/2),
                          cluster_pt_y.at(cluster_pt_y.size()/2),
                          cluster_pt_z.at(cluster_pt_z.size()/2));
  // std::cout << "  [athena] > median computed: " << median.x() << " " << median.y() << " " << median.z() << "\n";
  return median;
}

Eigen::Vector3d athena::pointcloud::computePointCloudMedian(std::vector<Eigen::Vector3d> vec){
  std::vector<double> x, y, z;
  for (int i = 0; i < vec.size(); i++){
    x.push_back(vec.at(i).x());
    y.push_back(vec.at(i).y());
    z.push_back(vec.at(i).z());
  }
  return computePointCloudMedian(x, y, z);
}

std::vector<double> athena::pointcloud::getMinAndMaxFromVector(std::vector<double> my_vector){
  std::vector<double> min_max;
  int min_idx = std::min_element(my_vector.begin(), my_vector.end()) - my_vector.begin();
  int max_idx = std::max_element(my_vector.begin(), my_vector.end()) - my_vector.begin();

  min_max.push_back(my_vector.at(min_idx));
  min_max.push_back(my_vector.at(max_idx));
  return min_max;
}

// Helper function to directly publish a point cloud under the publisher with desired frame_id
void athena::pointcloud::publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}

void athena::pointcloud::publishPointCloudXYZRGB(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}

PointCloudProperties athena::pointcloud::computePointCloudMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  PointCloudProperties results;
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  results.min_point = minPoint;
  results.max_point = maxPoint;

  // std::cout << "Min Point: " << minPoint.x << " " << minPoint.y << " " << minPoint.z << "\n";
  // std::cout << "Max Point: " << maxPoint.x << " " << maxPoint.y << " " << maxPoint.z << "\n";
  return results;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::removeNegativeWorldPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, Eigen::Matrix4d cam_to_world){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*input_cloud, *out_cloud, cam_to_world);
  out_cloud = athena::pointcloud::doPassThroughFilter(out_cloud, "z", 0.0, kInfinity);
  Eigen::Matrix4d world_to_cam = cam_to_world.inverse();   // Transform the cloud back into camera frame
  pcl::transformPointCloud(*out_cloud, *out_cloud, world_to_cam);
  return out_cloud;
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

pcl::PointIndices::Ptr athena::pointcloud::findCloudInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ pt, double radius_x, double radius_y, double radius_z) {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  for (int i = 0; i < cloud->points.size(); i++) {
    if (fabs(pt.x - cloud->points[i].x) < radius_x && fabs(pt.y - cloud->points[i].y) < radius_y && fabs(pt.z - cloud->points[i].z) < radius_z) {
      inliers->indices.push_back(i);
    }
  }
  return inliers;
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


pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::getMaxEuclideanClusterFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double tolerance){
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extract = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  tree->setInputCloud (input_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (tolerance); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (input_cloud);
  ec.extract (cluster_indices);

  if (cluster_indices.size() == 0 ){ return cloud_extract; }

  std::vector<int> cluster_sizes;
  for (int i = 0; i < cluster_indices.size(); i++){
    cluster_sizes.push_back(cluster_indices.at(i).indices.size());
  }
  auto result = std::max_element(cluster_sizes.begin(), cluster_sizes.end());
  int max_idx = std::distance(cluster_sizes.begin(), result);

  cloud_extract->width = cluster_indices.at(max_idx).indices.size();
  cloud_extract->height = 1;
  cloud_extract->is_dense = true;

  for (std::vector<int>::const_iterator pit = cluster_indices.at(max_idx).indices.begin (); pit != cluster_indices.at(max_idx).indices.end (); ++pit){
    cloud_extract->points.push_back (input_cloud->points[*pit]);
  }
  return cloud_extract;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr athena::pointcloud::createColorizedPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

  colored_cloud->width = cloud->width;
  colored_cloud->height = cloud->height;
  colored_cloud->is_dense = true; //cloud->is_dense;

  std::vector<double> cloud_pts_x, cloud_pts_y, cloud_pts_z;
  for (size_t i_point = 0; i_point < cloud->points.size (); i_point++){
     pcl::PointXYZRGB point;
     point.x = *(cloud->points[i_point].data);
     point.y = *(cloud->points[i_point].data + 1);
     point.z = *(cloud->points[i_point].data + 2);
     point.r = r;
     point.g = g;
     point.b = b;

     cloud_pts_x.push_back(point.x);
     cloud_pts_y.push_back(point.y);
     cloud_pts_z.push_back(point.z);

     colored_cloud->points.push_back(point);
  }
  return colored_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr  athena::pointcloud::convertClusterToPointCloud(int idx, std::vector <pcl::PointIndices> clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extract_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  auto cluster_idxs = clusters.at(idx).indices;
  cloud_extract_->is_dense = true;
  cloud_extract_->width = 0;
  cloud_extract_->height = 1;
  cloud_extract_->points.clear();
  for (int idx = 0; idx < cluster_idxs.size(); idx++){
    pcl::PointXYZ cloud_point = raw_cloud->at(cluster_idxs[idx]);
    cloud_extract_->points.push_back(cloud_point);
    cloud_extract_->width = cloud_extract_->width + 1;
  }
  return cloud_extract_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::doNeighborRadiusSearch(pcl::PointXYZ searchPoint, pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann,
                                                                             pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud, double radius)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  extracted_cloud->width = 0;
  extracted_cloud->height = 1;
  extracted_cloud->is_dense = true;

  if (kd_tree_flann.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
      extracted_cloud->width = pointIdxRadiusSearch.size();
      extracted_cloud->points.clear();
      for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++){
        extracted_cloud->points.push_back(raw_cloud->points[pointIdxRadiusSearch[i]]);
      }
   }
   //std::cout << "Extracted cloud size: " << extracted_cloud->points.size()  << "\n";
   return extracted_cloud;
}
