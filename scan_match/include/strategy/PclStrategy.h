#pragma once

#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/transforms.h>
#include "PCLScanRegistration.hpp"
#include "RosUtils.h"
#include "strategy/ScanMatchingStrategy.h"

class PclStrategy : public ScanMatchingStrategy {
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

 public:
  PclStrategy();
  PclStrategy(const int& maximum_iterations,
              const float& euclidean_fitness_epsilon,
              const float& transformation_epsilon,
              const float& max_correspondence_distance);

  /*
   Computes the odometry (position & orientation) from 2 scans with
   respetct to the previous one, and returns the absolute odometry
  */
  tf::Transform computeOdometry(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan) override;
  /*
  Computes the transformation between front and back scan, and returns the
  average transformation
  */
  Eigen::Matrix4f computeCalibration(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan) override;

  // Transforms the scan message into the pcl format
  PointCloud scan2Cloud(const sensor_msgs::LaserScan::ConstPtr& scan);
  /*
  case Odometry: Sets the transformation (calibration) between the front and
  back scanner
  case Calibration: Sets the initial guess for the calibration
  */
  void setTransformation(const Eigen::Matrix4f& transformation) override;

 protected:
  // Transforms the back scan into the same of the front scan and compines both
  // pointclouds
  PointCloud transformScans2Cloud(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan);


  PCLScanRegistration scan_registration_;   //handles the registration
  PointCloud::Ptr old_cloud_;
  tf::Transform transformation_;    // calibration between back and front
  laser_geometry::LaserProjection projector_; //projects the scan into an pointcloud
};
