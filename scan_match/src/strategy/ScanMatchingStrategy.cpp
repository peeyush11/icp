#include "strategy/ScanMatchingStrategy.h"
ScanMatchingStrategy::ScanMatchingStrategy() {}
//ScanMatchingStrategy::ScanMatchingStrategy(ros::NodeHandle& nodeHandle) {}

void ScanMatchingStrategy::setTransformation(
    const Eigen::Matrix4f& transformation) {
  transformation_ = transformation;
}

tf::Transform ScanMatchingStrategy::computeOdometry(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan){
    ROS_WARN("Using Abstract Method");

}

Eigen::Matrix4f ScanMatchingStrategy::computeCalibration(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan){}
