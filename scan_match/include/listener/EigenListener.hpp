#pragma once

// ROS
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_srvs/Trigger.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <time.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "EigenScanRegistration.hpp"

using namespace Eigen;
typedef Matrix<float, 2, Dynamic> PointCloud;
typedef Matrix<float, 2, 1> Point;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                                                        sensor_msgs::LaserScan>
    sync_pol_t;
typedef message_filters::Synchronizer<sync_pol_t> sync_t;

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class EigenListener {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  EigenListener(ros::NodeHandle& nodeHandle);
  EigenListener();

  /*!
   * Destructor.
   */
  virtual ~EigenListener();
  // setting the calibration
  void setScannerTransformation(const Matrix3f transformation);
  
  void Callback(const sensor_msgs::LaserScan::ConstPtr& first_scan,
                const sensor_msgs::LaserScan::ConstPtr& second_scan);
  void heuristicCallback(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan);
  static PointCloud scan2Cloud(const sensor_msgs::LaserScan::ConstPtr& scan);
  static Matrix3f tfToEigen(const tf::Transform& transform);
  static sensor_msgs::PointCloud PointCloud2Ros(const PointCloud& cloud);
  static nav_msgs::Odometry Eigen2Pose(const Eigen::Matrix3f& odometry);
  void publishOdom2Tf(const nav_msgs::Odometry& odom,
                      const std::string& source_frame);
  EigenScanRegistration::ScanProperties extractProperties(
      const sensor_msgs::LaserScan::ConstPtr& scan);
  void publishStaticTf(const Eigen::Matrix3f& transformation,
                                    const std::string& frame_id,
                                    const std::string& child_frame_id);

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */

  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);
  void waitForTransformation(const std::string& source_frame,
                             const std::string& target_frame);
  PointCloud transformScans2Cloud(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan);

  //! ROS node handle.
  ros::NodeHandle* nodeHandle_ = nullptr;

  std::unique_ptr<sync_t> sync_;
  message_filters::Subscriber<sensor_msgs::LaserScan> front_scan_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> back_scan_sub_;
  // ros::ServiceServer serviceServer_;

  std::string in_scan_topic_first_ = "/scan_front";
  std::string in_scan_topic_second_ = "/scan_rear";
  std::string out_cloud_topic_ = "scan_registrated";
  std::string out_pose_topic_ = "odometry";

  ros::Publisher publisher_front_;
  ros::Publisher publisher_back_;
  ros::Publisher publisher_pose_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  //! Algorithm computation object.
  EigenScanRegistration scan_registration_;
  PointCloud old_cloud_;
  PointCloud new_cloud_;
  Matrix3f back_to_front_transformation_;
  std::string back_frame_="back";
  std::string front_frame_="front";

  bool have_transformation_ = false;
  bool use_both_scans_ = false;

  bool calibrate_ = true;
  bool use_heuristic_=false;
  float max_correspondence_angle_=0;

  // ICP
  int maximum_iterations_ = 10;
  float euclidean_fitness_epsilon_ = 1e-4;
  float transformation_epsilon_ = 1e-8;
  float max_correspondence_distance_ = 0.1;

};
