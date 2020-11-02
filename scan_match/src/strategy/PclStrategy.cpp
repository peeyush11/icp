#include "strategy/PclStrategy.h"
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PclStrategy::PclStrategy() {}

PclStrategy::PclStrategy(const int& maximum_iterations,
                         const float& euclidean_fitness_epsilon,
                         const float& transformation_epsilon,
                         const float& max_correspondence_distance)
    : scan_registration_(PCLScanRegistration(
          maximum_iterations, euclidean_fitness_epsilon, transformation_epsilon,
          max_correspondence_distance)) {}

tf::Transform PclStrategy::computeOdometry(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  PointCloud::Ptr cloud(new PointCloud());
  *cloud = transformScans2Cloud(front_scan, back_scan);
  //*cloud= new_cloud;
  Eigen::Matrix4f odometry = scan_registration_.computeTransformation(cloud);
  return Matrix4ToTF(odometry);
}

Eigen::Matrix4f PclStrategy::computeCalibration(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  PointCloud front_cloud = scan2Cloud(front_scan);
  PointCloud back_cloud = scan2Cloud(back_scan);

  PointCloud back_cloud_transformed;
  pcl_ros::transformPointCloud(back_cloud, back_cloud_transformed,
                               transformation_);
  //*cloud= new_cloud;
  Eigen::Matrix4f calibration = scan_registration_.calibrate(
      front_cloud.makeShared(), back_cloud_transformed.makeShared());
  return calibration * TF2Matrix4(transformation_);
}

PointCloud PclStrategy::scan2Cloud(
    const sensor_msgs::LaserScan::ConstPtr& scan) {
  sensor_msgs::PointCloud2 ros_cloud;
  projector_.projectLaser(*scan, ros_cloud);
  PointCloud pcl_cloud;
  pcl::fromROSMsg(ros_cloud, pcl_cloud);
  return pcl_cloud;
}
PointCloud PclStrategy::transformScans2Cloud(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  PointCloud front_cloud = scan2Cloud(front_scan);
  PointCloud back_cloud = scan2Cloud(back_scan);

  PointCloud back_cloud_transformed;
  pcl_ros::transformPointCloud(back_cloud, back_cloud_transformed,
                               transformation_);

  front_cloud += back_cloud_transformed;
  return front_cloud;
}

void PclStrategy::setTransformation(const Eigen::Matrix4f& transformation) {
  transformation_ = Matrix4ToTF(transformation);
}
