#include "athena/pointcloud/utils.h"

// Computes the median of a point cloud
Eigen::Vector3d athena::pointcloud::computePointCloudMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  std::vector<double> cloud_x, cloud_y, cloud_z;
  for (int i = 0; i < cloud->points.size(); i++){
    pcl::PointXYZ pt = cloud->points[i];
    cloud_x.push_back(pt.x);
    cloud_y.push_back(pt.y);
    cloud_z.push_back(pt.z);
  }
  return computePointCloudMedian(cloud_x, cloud_y, cloud_z);
}

Eigen::Vector3d athena::pointcloud::computePointCloudMedian(std::vector<double> cluster_pt_x, std::vector<double> cluster_pt_y, std::vector<double> cluster_pt_z){
  Eigen::Vector3d median;
  sort(cluster_pt_x.begin(), cluster_pt_x.end());
  sort(cluster_pt_y.begin(), cluster_pt_y.end());
  sort(cluster_pt_z.begin(), cluster_pt_z.end());

  median(0) = cluster_pt_x.at(cluster_pt_x.size()/2);
  median(1) = cluster_pt_y.at(cluster_pt_y.size()/2);
  median(2) = cluster_pt_z.at(cluster_pt_z.size()/2);
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

// Computes the geometry (e.g. min and max displacements aloong the world coordinate frame) for a point cloud
athena::pointcloud::ObjectGeometries* athena::pointcloud::computePointCloudGeometries(std::string obj_name, pcl::PointCloud<pcl::PointXYZ> cloud){
  double min_x = kInfinity;
  double max_x = -kInfinity;
  double min_y = kInfinity;
  double max_y = -kInfinity;
  double min_z = kInfinity;
  double max_z = -kInfinity;
  for (int i = 0; i < cloud.size(); i++){
    pcl::PointXYZ cloud_point = cloud.at(i);
    if (cloud_point.x > max_x){ max_x = cloud_point.x; }
    if (cloud_point.x < min_x){ min_x = cloud_point.x; }
    if (cloud_point.y > max_y){ max_y = cloud_point.y; }
    if (cloud_point.y < min_y){ min_y = cloud_point.y; }
    if (cloud_point.z > max_z){ max_z = cloud_point.z; }
    if (cloud_point.z < min_z){ min_z = cloud_point.z; }
  }
  athena::pointcloud::ObjectGeometries *obj_geometry = new athena::pointcloud::ObjectGeometries(obj_name, min_x, max_x, min_y, max_y, min_z, max_z);
  return obj_geometry;
}

sensor_msgs::PointCloud2 athena::pointcloud::toSensorMsgPointCloud2(pcl::PointCloud<pcl::PointXYZ> pcl_cloud){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  return cloud_msg;
}

sensor_msgs::PointCloud2 athena::pointcloud::toSensorMsgPointCloud2(pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  return cloud_msg;
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

Eigen::Vector3d athena::pointcloud::computePointCloudBoundingBoxOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  PointCloudProperties props = computePointCloudMinMax(cloud);
  Eigen::Vector3d result;
  result.x() = (props.max_point.x - props.min_point.x)/2.0 +  props.min_point.x;
  result.y() = (props.max_point.y - props.min_point.y)/2.0 +  props.min_point.y;
  result.z() = (props.max_point.z - props.min_point.z)/2.0 +  props.min_point.z;
  return result;
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

pcl::PointXYZ athena::pointcloud::eigenVectorToPclPointXYZ(Eigen::Vector3d vector){
  pcl::PointXYZ result;
  result.x = vector.x();
  result.y = vector.y();
  result.z = vector.z();
  return result;
}

 pcl::PointCloud<pcl::PointXYZ>::Ptr athena::pointcloud::sensorMsgToPclPointCloudXYZ(const sensor_msgs::PointCloud2ConstPtr& input){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2* input_cloud_pcl = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(input_cloud_pcl);
  pcl_conversions::toPCL(*input, *input_cloud_pcl);
  pcl::fromPCLPointCloud2(*input_cloud_pcl, *cloud);
  return cloud;
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


geometry_msgs::Pose athena::pointcloud::pclPointXYZToGeometryMsgPose(pcl::PointXYZ pt){
  geometry_msgs::Pose pose;
  pose.position.x = pt.x;
  pose.position.y = pt.y;
  pose.position.z = pt.z;
  pose.orientation.w = 1.0;
  return pose;
}
