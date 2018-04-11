#include "athena/pointcloud/conversions.h"


// From https://stackoverflow.com/questions/41816369/how-to-convert-cvmat-to-pclpointcloud-with-color?noredirect=1&lq=1
// !! Note that the returned image is in BGR format
// Coords is the mapping from image (u, v) to camera optical frame x, y, z
void pointcloud_conversions::pointcloudToBGRImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                                cv::Mat &coords, cv::Mat &image)
{
  coords = cv::Mat(3, cloud->height, cloud->width, CV_64FC1);
  image = cv::Mat(cloud->height, cloud->width, CV_8UC3);

  #pragma omp parallel for
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++){
      coords.at<double>(0, x, y) = cloud->points.at(y*image.cols+x).x;
      coords.at<double>(1, x, y) = cloud->points.at(y*image.cols+x).y;
      coords.at<double>(2, x, y) = cloud->points.at(y*image.cols+x).z;

      cv::Vec3b color = cv::Vec3b(
              cloud->points.at(y*image.cols+x).b,
              cloud->points.at(y*image.cols+x).g,
              cloud->points.at(y*image.cols+x).r);

      image.at<cv::Vec3b>(cv::Point(x,y)) = color;
    }
  }
}
