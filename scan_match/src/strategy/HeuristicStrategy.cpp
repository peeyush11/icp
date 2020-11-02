#include "strategy/HeuristicStrategy.h"
typedef Matrix<float, 2, Dynamic> PointCloud;

HeuristicStrategy::HeuristicStrategy() {}

HeuristicStrategy::HeuristicStrategy(const int& maximum_iterations,
                                     const float& euclidean_fitness_epsilon,
                                     const float& transformation_epsilon,
                                     const float& max_correspondence_distance,
                                     const float& max_correspondence_angle)
    : max_correspondence_angle_{max_correspondence_angle} {
  scan_registration_ = new HeuristicRegistration(
      maximum_iterations, euclidean_fitness_epsilon, transformation_epsilon,
      max_correspondence_distance);
}

PointCloud HeuristicStrategy::transformScans2Cloud(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  PointCloud front_cloud = scan2Cloud(front_scan);

  PointCloud back_cloud = EigenScanRegistration::transformPointCloud(
      scan2Cloud(back_scan), transformation_);
  int back_cloud_length = end_ind_ - start_ind_ + 1;
  back_cloud = back_cloud.block(0, start_ind_, 2, back_cloud_length);

  PointCloud cloud(2, front_cloud.cols() + back_cloud_length);
  cloud << front_cloud, back_cloud;
  return cloud;
}
tf::Transform HeuristicStrategy::computeOdometry(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  if (old_cloud_.cols() < 1) {
    extractIndices(front_scan, back_scan);
    scan_registration_->setNumberNeighbors(number_neighbors_);
    old_cloud_ = transformScans2Cloud(front_scan, back_scan);
    return Matrix3ToTF(Eigen::Matrix3f::Identity());
  } else {
    PointCloud new_cloud = transformScans2Cloud(front_scan, back_scan);
    scan_registration_->setInputSource(old_cloud_);
    scan_registration_->setInputTarget(makeCircular(new_cloud));
    scan_registration_->align();
    old_cloud_ = new_cloud;
    std::cout << "odometry: \n"
              << scan_registration_->getFinalTransformation() << std::endl;
    return Matrix3ToTF(scan_registration_->getFinalTransformation());
  }
}

void HeuristicStrategy::extractIndices(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  PointCloud back_cloud = EigenScanRegistration::transformPointCloud(
      scan2Cloud(back_scan), transformation_);
  bool have_start_ind = false;
  for (int col = 0; col < back_cloud.cols(); ++col) {
    float angle =
        atan2(back_cloud(1, col), back_cloud(0, col));  // tan = sin /cos
    if (angle > front_scan->angle_max && !have_start_ind) {
      start_ind_ = col;
      have_start_ind = true;
    }
    if (angle < front_scan->angle_min) {
      end_ind_ = col;
    }
  }
  number_neighbors_ =
      (int)(max_correspondence_angle_ / front_scan->angle_increment);
  ROS_INFO_STREAM("number_neighbors_:" << number_neighbors_);
}

PointCloud HeuristicStrategy::makeCircular(const PointCloud& cloud) {
  PointCloud circular_cloud(2, cloud.cols() + 2 * number_neighbors_);
  circular_cloud << cloud.block(0, (cloud.cols()) - number_neighbors_, 2,
                                number_neighbors_),
      cloud, cloud.block(0, 0, 2, number_neighbors_);
  return circular_cloud;
}
