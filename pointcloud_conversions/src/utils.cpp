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

void pointcloud_utils::publishPointCloudXYZRGB(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, std::string frame_id){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}

PointCloudProperties pointcloud_utils::computePointCloudMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  PointCloudProperties results;
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
  results.min_point = minPoint;
  results.max_point = maxPoint;
  std::cout << "Min Point: " << minPoint.x << " " << minPoint.y << " " << minPoint.z << "\n";
  std::cout << "Max Point: " << maxPoint.x << " " << maxPoint.y << " " << maxPoint.z << "\n";

  return results;
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
