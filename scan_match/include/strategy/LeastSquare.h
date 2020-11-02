#pragma once
#include "LeastSquareICP.hpp"
#include "RosUtils.h"
#include "strategy/ScanMatchingStrategy.h"

class LeastSquare : public ScanMatchingStrategy {
  typedef Matrix<float, 2, Dynamic> PointCloud;

 public:
  LeastSquare();
  LeastSquare(const int& maximum_iterations,
                  const float& euclidean_fitness_epsilon,
                  const float& transformation_epsilon,
                  const float& max_correspondence_distance);

 virtual tf::Transform computeOdometry(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan) override;

  Eigen::Matrix4f computeCalibration(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan) override;


  void setTransformation(const Eigen::Matrix4f& transformation) override;

 protected:
  static PointCloud scan2Cloud(const sensor_msgs::LaserScan::ConstPtr& scan);
  virtual PointCloud transformScans2Cloud(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan);

  PointCloud old_cloud_;
  Eigen::Matrix4f transformation_;
  LeastSquareICP* scan_registration_;
};
