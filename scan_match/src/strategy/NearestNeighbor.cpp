#include "strategy/NearestNeighbor.h"
typedef Matrix<float, 2, Dynamic> PointCloud;

NearestNeighbor::NearestNeighbor() {}

NearestNeighbor::NearestNeighbor(const int& maximum_iterations,
                                 const float& euclidean_fitness_epsilon,
                                 const float& transformation_epsilon,
                                 const float& max_correspondence_distance)
    : scan_registration_(new EigenScanRegistration(
          maximum_iterations, euclidean_fitness_epsilon, transformation_epsilon,
          max_correspondence_distance)) {
  calibration_translation_ << 0, 0;
}

tf::Transform NearestNeighbor::computeOdometry(
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
    std::cout << "odometry: \n"
              << scan_registration_->getFinalTransformation() << std::endl;
    return Matrix3ToTF(scan_registration_->getFinalTransformation());
  }
}

Eigen::Matrix4f NearestNeighbor::computeCalibration(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  scan_registration_->setInputSource(EigenScanRegistration::transformPointCloud(
      scan2Cloud(back_scan), transformation_));
  scan_registration_->setInputTarget(scan2Cloud(front_scan));
  scan_registration_->align();
  Eigen::Matrix3f calibration = scan_registration_->getFinalTransformation();
  scan_registration_->resetTransformation();
  return averageCalibration(calibration);
}

PointCloud NearestNeighbor::scan2Cloud(
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
PointCloud NearestNeighbor::transformScans2Cloud(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  PointCloud front_cloud = scan2Cloud(front_scan);

  PointCloud back_cloud = EigenScanRegistration::transformPointCloud(
      scan2Cloud(back_scan), transformation_);

  PointCloud cloud(2, front_cloud.cols() + back_cloud.cols());
  cloud << front_cloud, back_cloud;
  return cloud;
}

void NearestNeighbor::setTransformation(const Eigen::Matrix4f& transformation) {
  transformation_ = transformation;
  // transformation_.block<2, 2>(0, 0) = transformation.block<2, 2>(0, 0);
  // transformation_.block<2, 1>(0, 2) = transformation.block<2, 1>(0, 3);
}

Eigen::Matrix4f NearestNeighbor::averageCalibration(
    const Eigen::Matrix3f& calibration) {
  calibration_translation_ += calibration.block<2, 1>(0, 2);
  calibration_rotation_ += std::atan2(calibration(0, 1), calibration(0, 0));
  ++counter_;
  Eigen::Matrix4f average_calibration = Eigen::Matrix4f::Identity();
  average_calibration(0, 0) = cos(calibration_rotation_ / counter_);
  average_calibration(1, 1) = cos(calibration_rotation_ / counter_);
  average_calibration(0, 1) = sin(calibration_rotation_ / counter_);
  average_calibration(1, 0) = -sin(calibration_rotation_ / counter_);
  average_calibration(0, 3) = calibration_translation_(0, 0) / counter_;
  average_calibration(1, 3) = calibration_translation_(1, 0) / counter_;
  return average_calibration * transformation_;
}