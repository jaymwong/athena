#include "athena/pointcloud/utils.h"

// Computes the median of a point cloud
Eigen::Vector3d pointcloud_utils::computePointCloudMedian(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  std::vector<double> cloud_x, cloud_y, cloud_z;
  for (int i = 0; i < cloud->points.size(); i++){
    pcl::PointXYZ pt = cloud->points[i];
    cloud_x.push_back(pt.x);
    cloud_y.push_back(pt.y);
    cloud_z.push_back(pt.z);
  }
  return computePointCloudMedian(cloud_x, cloud_y, cloud_z);
}

Eigen::Vector3d pointcloud_utils::computePointCloudMedian(std::vector<double> cluster_pt_x, std::vector<double> cluster_pt_y, std::vector<double> cluster_pt_z){
  Eigen::Vector3d median;
  sort(cluster_pt_x.begin(), cluster_pt_x.end());
  sort(cluster_pt_y.begin(), cluster_pt_y.end());
  sort(cluster_pt_z.begin(), cluster_pt_z.end());

  median(0) = cluster_pt_x.at(cluster_pt_x.size()/2);
  median(1) = cluster_pt_y.at(cluster_pt_y.size()/2);
  median(2) = cluster_pt_z.at(cluster_pt_z.size()/2);
  return median;
}

Eigen::Vector3d pointcloud_utils::computePointCloudMedian(std::vector<Eigen::Vector3d> vec){
  std::vector<double> x, y, z;
  for (int i = 0; i < vec.size(); i++){
    x.push_back(vec.at(i).x());
    y.push_back(vec.at(i).y());
    z.push_back(vec.at(i).z());
  }
  return computePointCloudMedian(x, y, z);
}

// Helper function to directly publish a point cloud under the publisher with desired frame_id
void pointcloud_utils::publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}

void pointcloud_utils::publishPointCloudXYZRGB(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}

Eigen::Vector3d pointcloud_utils::computePointCloudBoundingBoxOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  PointCloudProperties props = computePointCloudMinMax(cloud);
  Eigen::Vector3d result;
  result.x() = (props.max_point.x - props.min_point.x)/2.0 +  props.min_point.x;
  result.y() = (props.max_point.y - props.min_point.y)/2.0 +  props.min_point.y;
  result.z() = (props.max_point.z - props.min_point.z)/2.0 +  props.min_point.z;
  return result;
}

PointCloudProperties pointcloud_utils::computePointCloudMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  PointCloudProperties results;
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  results.min_point = minPoint;
  results.max_point = maxPoint;

  // std::cout << "Min Point: " << minPoint.x << " " << minPoint.y << " " << minPoint.z << "\n";
  // std::cout << "Max Point: " << maxPoint.x << " " << maxPoint.y << " " << maxPoint.z << "\n";
  return results;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_utils::getStatisticsPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double thresh){
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (input_cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (thresh);
  sor.filter (*out_cloud);
  return out_cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_utils::getMaxEuclideanClusterFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, double tolerance){
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_utils::createColorizedPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int r, int g, int b){
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


pcl::PointXYZ pointcloud_utils::eigenVectorToPclPointXYZ(Eigen::Vector3d vector){
  pcl::PointXYZ result;
  result.x = vector.x();
  result.y = vector.y();
  result.z = vector.z();
  return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_utils::doNeighborRadiusSearch(pcl::PointXYZ searchPoint, pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann,
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


geometry_msgs::Pose pointcloud_utils::pclPointXYZToGeometryMsgPose(pcl::PointXYZ pt){
  geometry_msgs::Pose pose;
  pose.position.x = pt.x;
  pose.position.y = pt.y;
  pose.position.z = pt.z;
  pose.orientation.w = 1.0;
  return pose;
}
