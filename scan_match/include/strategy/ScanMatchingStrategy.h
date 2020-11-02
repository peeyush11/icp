#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

class ScanMatchingStrategy {
 public:
  //ScanMatchingStrategy(ros::NodeHandle& nodeHandle);
  ScanMatchingStrategy(){};

  /*
   Computes the odometry (position & orientation) from 2 scans with
   respetct to the previous one, and returns the absolute odometry
  */
  virtual tf::Transform computeOdometry(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan)=0;

  /*
  Computes the transformation between front and back scan, and returns the
  average transformation
  */
  virtual Eigen::Matrix4f computeCalibration(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan)=0;

  /*
  case Odometry: Sets the transformation (calibration) between the front and
  back scanner
  case Calibration: Sets the initial guess for the calibration
  */
  virtual void setTransformation(const Eigen::Matrix4f& transformation)=0;

 //protected:
 // Eigen::Matrix4f transformation_;
};