#include "strategy/LeastSquare.h"
typedef Matrix<float, 2, Dynamic> PointCloud;

LeastSquare::LeastSquare() {}

LeastSquare::LeastSquare(const int& maximum_iterations,
                                 const float& euclidean_fitness_epsilon,
                                 const float& transformation_epsilon,
                                 const float& max_correspondence_distance)
    : scan_registration_(new LeastSquareICP(
          maximum_iterations, euclidean_fitness_epsilon, transformation_epsilon,
          max_correspondence_distance)) {}

tf::Transform LeastSquare::computeOdometry(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  PointCloud new_cloud = transformScans2Cloud(front_scan, back_scan);
  if (old_cloud_.cols() < 1) {
    old_cloud_ = new_cloud;
    return Matrix3ToTF(Eigen::Matrix3f::Identity());
  } else {
    scan_registration_->setInputSource(old_cloud_);
    scan_registration_->setInputTarget(new_cloud);
    scan_registration_->align();
    old_cloud_ = new_cloud;
    std::cout<<"odometry: \n"<<scan_registration_->getFinalTransformation()<<std::endl;
    return Matrix3ToTF(scan_registration_->getFinalTransformation());
  }
}

Eigen::Matrix4f LeastSquare::computeCalibration(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {}

PointCloud LeastSquare::scan2Cloud(
    const sensor_msgs::LaserScan::ConstPtr& scan) {
  PointCloud cloud(2, scan->ranges.size());
  for (int increment = 0; increment < cloud.cols(); increment++) {
    cloud(0, increment) =
        scan->ranges[increment] *
        cos(scan->angle_min + increment * scan->angle_increment);
    cloud(1, increment) =
        scan->ranges[increment] *
        sin(scan->angle_min + increment * scan->angle_increment);
  }
  return cloud;
}
PointCloud LeastSquare::transformScans2Cloud(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  PointCloud front_cloud = scan2Cloud(front_scan);

  PointCloud back_cloud = LeastSquareICP::transformPointCloud(
      scan2Cloud(back_scan), transformation_);

  PointCloud cloud(2, front_cloud.cols() + back_cloud.cols());
  cloud << front_cloud, back_cloud;
  return cloud;
}

void LeastSquare::setTransformation(
    const Eigen::Matrix4f& transformation) {
  transformation_ = transformation;
}
