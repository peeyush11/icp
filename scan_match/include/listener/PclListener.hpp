#pragma once

// ROS
#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <time.h>

#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "PCLScanRegistration.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                                                        sensor_msgs::LaserScan>
    sync_pol_t;
typedef message_filters::Synchronizer<sync_pol_t> sync_t;

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class PclListener {
    typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef pcl::PointCloud<pcl::PointXYZ> pcl_PointCloud;

 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  PclListener(ros::NodeHandle& nodeHandle);
  PclListener(ros::NodeHandle& nodeHandle, bool calibrate);

  /*!
   * Destructor.
   */
  virtual ~PclListener();

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
  void calibrationCallback(const sensor_msgs::LaserScan::ConstPtr& first_scan,
                           const sensor_msgs::LaserScan::ConstPtr& second_scan);
  void odometryCallback(const sensor_msgs::LaserScan::ConstPtr& first_scan,
                        const sensor_msgs::LaserScan::ConstPtr& second_scan);

  bool serviceCallback(std_srvs::Trigger::Request& request,
                       std_srvs::Trigger::Response& response);
  void waitForTransformation(const std::string& source_frame,
                             const std::string& target_frame);
  void saveCalibration(const std::string& calibration_file);
  static Eigen::Matrix4f TF2Matrix4(const tf::Transform& transformation);
  static tf::Transform Matrix4ToTF(const Eigen::Matrix4f& matrix);
  void publishStaticTf(const tf::Transform& transformation,
                       const std::string& frame_id,
                       const std::string& child_frame_id);
  void publishTf(const tf::Transform& transformation,
                 const std::string& frame_id, const std::string& child_frame_id,
                 const std_msgs::Header& header);
    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    std::unique_ptr<sync_t> sync_;
    message_filters::Subscriber<sensor_msgs::LaserScan> first_scan_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> second_scan_sub_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
    ros::ServiceServer serviceServer_;

    laser_geometry::LaserProjection projector_;

    std::string in_scan_topic_first_;
    std::string in_scan_topic_second_;
    std::string out_cloud_topic_;

    tf::Transform back_to_front_transformation_;
    bool have_transformation_ = false;
    ros::Publisher publisher_cloud_;
    ros::Publisher publisher_front_;
    ros::Publisher publisher_back_;

    //! Algorithm computation object.
    PCLScanRegistration pcl_scan_registration_;
    bool calibrate_ = false;
    bool use_both_scans_ = true;
    std::string calibration_file_;
    std::string back_frame_ = "back";
    std::string front_frame_ = "front";

    int maximum_iterations_ = 10;
    float euclidean_fitness_epsilon_ = 1e-4;
    float transformation_epsilon_ = 1e-8;
    float max_correspondence_distance_ = 0.1;
  };
