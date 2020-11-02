#pragma once
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen_conversions/eigen_msg.h>

#include "RosUtils.h"
#include "strategy/HeuristicStrategy.h"
#include "strategy/NearestNeighbor.h"
#include "strategy/PclStrategy.h"
#include "strategy/LeastSquare.h"
#include "strategy/ScanMatchingStrategy.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                                                        sensor_msgs::LaserScan>
    sync_pol_t;
typedef message_filters::Synchronizer<sync_pol_t> sync_t;

/*
Class that handles the ros related stuff:
-receiving the scanner messages (for calibration and odometry)
-reads the parameter
-publish the tf and the messages
*/
class ScanHandler {
 public:
  ScanHandler(ros::NodeHandle& node_handle);
  void startListening();

 private:
  /*
  ScanOdometryCallback:
  receives 2 scans
  computes the odometry between these and the previous ones
  publish the odometry and the scans
    */
  void scanOdometryCallback(const sensor_msgs::LaserScan::ConstPtr& front_scan,
                            const sensor_msgs::LaserScan::ConstPtr& back_scan);

  /*
  calibrationCallback:
  receives 2 scans 
  computes the transformation from the back frame to the front frame
  saves the calibration
  publish the transformation and the scans
  */
  void calibrationCallback(const sensor_msgs::LaserScan::ConstPtr& front_scan,
                           const sensor_msgs::LaserScan::ConstPtr& back_scan);

  bool initStrategy();
  void waitForTransformation(const std::string& source_frame,
                             const std::string& target_frame);

  void publishTf(const tf::Transform& transformation,
                 const std::string& frame_id, const std::string& child_frame_id,
                 const std_msgs::Header& header);

  void publishStaticTf(const Eigen::Matrix4f& transformation,
                       const std::string& frame_id,
                       const std::string& child_frame_id);

  bool readParameters();

  void saveCalibration(const Eigen::Matrix4f& calibration,
                       const std::string& calibration_file);

  ScanMatchingStrategy* strategy_;
  std::string method_;
  ros::NodeHandle* node_handle_ = nullptr;

  std::unique_ptr<sync_t> sync_;
  message_filters::Subscriber<sensor_msgs::LaserScan> front_scan_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> back_scan_sub_;
  // ros::ServiceServer serviceServer_;

  std::string in_scan_topic_front_ = "/scan_front";
  std::string in_scan_topic_back_ = "/scan_rear";
  std::string back_frame_ = "/back";
  std::string front_frame_ = "/front";
  std::string calibration_file_;

  ros::Publisher publisher_front_;
  ros::Publisher publisher_back_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  bool have_transformation_ = false;
  bool calibrate_ = false;
  bool use_both_scans_ = true;
  Eigen::Matrix4f back_to_front_transformation_;

  // ICP Parameter
  int maximum_iterations_ = 10;
  float euclidean_fitness_epsilon_ = 1e-4;
  float transformation_epsilon_ = 1e-8;
  float max_correspondence_distance_ = 0.1;

  int number_transformations_=0;
  double accumulated_time_=0;
};