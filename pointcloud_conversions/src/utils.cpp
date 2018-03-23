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

// Helper function to directly publish a point cloud under the publisher with desired frame_id
void pointcloud_utils::publishPointCloudXYZ(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}
