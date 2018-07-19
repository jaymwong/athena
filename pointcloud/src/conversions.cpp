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

Eigen::Vector3d athena::pointcloud::toEigen(pcl::PointXYZ pt){
  return athena::pointcloud::pclPointToEigenVector3d(pt);
}

Eigen::Vector3d athena::pointcloud::pclPointToEigenVector3d(pcl::PointXYZ pt){
  Eigen::Vector3d vec;
  vec.x() = pt.x;
  vec.y() = pt.y;
  vec.z() = pt.z;
  return vec;
}

Eigen::Vector3d athena::pointcloud::GeometryMsgsPoseStampedToeigenVector3d(geometry_msgs::PoseStamped pt) {
  Eigen::Vector3d point;
  point.x() = pt.pose.position.x;
  point.y() = pt.pose.position.y;
  point.z() = pt.pose.position.z;
  return point;
}

pcl::PointXYZ athena::pointcloud::eigenVector3dToPclPoint(Eigen::Vector3d vec){
  pcl::PointXYZ pt;
  pt.x = vec.x();
  pt.y = vec.y();
  pt.z = vec.z();
  return pt;
}

pcl::PointXYZ athena::pointcloud::toPclPointXYZ(geometry_msgs::PointStamped msg){
  pcl::PointXYZ pt (msg.point.x, msg.point.y, msg.point.z);
  return pt;
}
