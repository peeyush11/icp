#pragma once
#include <math.h>
#include "EigenScanRegistration.hpp"
#include "RosUtils.h"
#include "strategy/ScanMatchingStrategy.h"

class NearestNeighbor : public ScanMatchingStrategy {
  typedef Matrix<float, 2, Dynamic> PointCloud;

 public:
  NearestNeighbor();
  NearestNeighbor(const int& maximum_iterations,
                  const float& euclidean_fitness_epsilon,
                  const float& transformation_epsilon,
                  const float& max_correspondence_distance);
  /*
   Computes the odometry (position & orientation) from 2 scans with
   respetct to the previous one, and returns the absolute odometry
  */
  virtual tf::Transform computeOdometry(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan) override;
  /*
  Computes the transformation between front and back scan, and returns the
  average transformation
  */
  Eigen::Matrix4f computeCalibration(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan) override;
  /*
  case Odometry: Sets the transformation (calibration) between the front and
  back scanner
  case Calibration: Sets the initial guess for the calibration
  */
  void setTransformation(const Eigen::Matrix4f& transformation) override;

 protected:
  // transforms the laserscan data in the eigen format
  static PointCloud scan2Cloud(const sensor_msgs::LaserScan::ConstPtr& scan);
  // Transforms the back scan into the same of the front scan and compines both
  // pointclouds
  virtual PointCloud transformScans2Cloud(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan);
  Eigen::Matrix4f averageCalibration(const Eigen::Matrix3f& calibration);

  PointCloud old_cloud_;
  Eigen::Matrix4f transformation_;  // Transformation between back and front
  EigenScanRegistration* scan_registration_;
  float calibration_rotation_ = 0;
  Eigen::Matrix<float, 2, 1> calibration_translation_;
  int counter_ = 0;
};
