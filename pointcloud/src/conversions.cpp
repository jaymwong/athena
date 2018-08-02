#include "athena/pointcloud/conversions.h"


// From https://stackoverflow.com/questions/41816369/how-to-convert-cvmat-to-pclpointcloud-with-color?noredirect=1&lq=1
// !! Note that the returned image is in BGR format
// Coords is the mapping from image (u, v) to camera optical frame x, y, z
void athena::pointcloud::pointcloudToBGRImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                                  cv::Mat &coords, cv::Mat &image)
{
  coords = cv::Mat(3, cloud->points.size(), CV_64FC1);
  image = cv::Mat(cloud->width, cloud->height, CV_8UC3);
  for(int y=0;y<image.rows;y++) {
    for(int x=0;x<image.cols;x++) {
        coords.at<double>(0,y*image.cols+x) = cloud->points.at(y*image.cols+x).x;
        coords.at<double>(1,y*image.cols+x) = cloud->points.at(y*image.cols+x).y;
        coords.at<double>(2,y*image.cols+x) = cloud->points.at(y*image.cols+x).z;

        cv::Vec3b color = cv::Vec3b(
                cloud->points.at(y*image.cols+x).b,
                cloud->points.at(y*image.cols+x).g,
                cloud->points.at(y*image.cols+x).r);

        image.at<cv::Vec3b>(cv::Point(x,y)) = color;
    }
  }
}

pcl::PointXYZ athena::conversions::toPclPointXYZ(boost::array<float, 3> array){
  pcl::PointXYZ pt;
  pt.x = array[0];
  pt.y = array[1];
  pt.z = array[2];
  return pt;
}

Eigen::Vector3d athena::conversions::toEigenVector3d(pcl::PointXYZ pt){
  Eigen::Vector3d vec;
  vec.x() = pt.x;
  vec.y() = pt.y;
  vec.z() = pt.z;
  return vec;
}

pcl::PointXYZ athena::conversions::toPclPointXYZ(Eigen::Vector3d vec){
  pcl::PointXYZ pt;
  pt.x = vec.x();
  pt.y = vec.y();
  pt.z = vec.z();
  return pt;
}

pcl::PointXYZ athena::conversions::toPclPointXYZ(geometry_msgs::PointStamped msg){
  pcl::PointXYZ pt (msg.point.x, msg.point.y, msg.point.z);
  return pt;
}

sensor_msgs::PointCloud2 athena::conversions::toSensorMsgPointCloud2(pcl::PointCloud<pcl::PointXYZ> pcl_cloud){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  return cloud_msg;
}

sensor_msgs::PointCloud2 athena::conversions::toSensorMsgPointCloud2(pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  return cloud_msg;
}

sensor_msgs::PointCloud2 athena::conversions::toSensorMsgPointCloud2(pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl_conversions::fromPCL(pcl_pc2, cloud_msg);
  return cloud_msg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::conversions::toPclPointCloudXYZ(const sensor_msgs::PointCloud2ConstPtr& input){
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PCLPointCloud2* input_cloud_pcl = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloudPtr(input_cloud_pcl);
 pcl_conversions::toPCL(*input, *input_cloud_pcl);
 pcl::fromPCLPointCloud2(*input_cloud_pcl, *cloud);
 return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr athena::conversions::toPclPointCloudXYZ(sensor_msgs::PointCloud2 input){
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PCLPointCloud2* input_cloud_pcl = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloudPtr(input_cloud_pcl);
 pcl_conversions::toPCL(input, *input_cloud_pcl);
 pcl::fromPCLPointCloud2(*input_cloud_pcl, *cloud);
 return cloud;
}

geometry_msgs::Pose athena::conversions::toGeometryMsgPose(pcl::PointXYZ pt){
  geometry_msgs::Pose pose;
  pose.position.x = pt.x;
  pose.position.y = pt.y;
  pose.position.z = pt.z;
  pose.orientation.w = 1.0;
  return pose;
}
