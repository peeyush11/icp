#include "listener/EigenListener.hpp"

EigenListener::EigenListener(ros::NodeHandle& nodeHandle)
    : nodeHandle_(&nodeHandle) {
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  scan_registration_ = EigenScanRegistration(
      maximum_iterations_, euclidean_fitness_epsilon_, transformation_epsilon_,
      max_correspondence_distance_);

  publisher_back_ =
      nodeHandle.advertise<sensor_msgs::LaserScan>("/back_scan", 10);
  publisher_front_ =
      nodeHandle.advertise<sensor_msgs::LaserScan>("/front_scan", 10);
  publisher_pose_ =
      nodeHandle.advertise<nav_msgs::Odometry>(out_pose_topic_, 10);
  front_scan_sub_.subscribe(nodeHandle, in_scan_topic_first_, 1);
  back_scan_sub_.subscribe(nodeHandle, in_scan_topic_second_, 1);

  // sync policy
  sync_ = std::unique_ptr<sync_t>(
      new sync_t(sync_pol_t(10), front_scan_sub_, back_scan_sub_));
  if (use_heuristic_) {
    scan_registration_.setMaxCorrespondenceAngle(max_correspondence_angle_);
    sync_->registerCallback(
        boost::bind(&EigenListener::Callback, this, _1, _2));
  } else {
    sync_->registerCallback(
        boost::bind(&EigenListener::Callback, this, _1, _2));
  }
  ROS_INFO("Successfully launched node.");
}
EigenListener::EigenListener() {}
EigenListener::~EigenListener() {}

void EigenListener::Callback(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  clock_t tStart = clock();
  std::cout << "\n\n ---------------msgs received---------" << std::endl;
  if (!have_transformation_) {
    waitForTransformation(back_scan->header.frame_id,
                          front_scan->header.frame_id);

  } else {
    PointCloud cloud;
    if (use_both_scans_) {
      cloud = transformScans2Cloud(front_scan, back_scan);
    } else {
      cloud = scan2Cloud(front_scan);
    }
    if (old_cloud_.cols() < 1) {
      old_cloud_ = cloud;
      new_cloud_ = cloud;
    } else {
      old_cloud_ = new_cloud_;
      new_cloud_ = cloud;
      scan_registration_.setInputSource(old_cloud_);
      scan_registration_.setInputTarget(new_cloud_);
      scan_registration_.align();

      std::cout << "absolute transformation: \n"
                << scan_registration_.getFinalTransformation() << std::endl;
      nav_msgs::Odometry odometry =
          Eigen2Pose(scan_registration_.getFinalTransformation());
      odometry.header = front_scan->header;
      odometry.header.frame_id = "/world";
      publisher_pose_.publish(odometry);
      publishOdom2Tf(odometry, front_frame_);
      sensor_msgs::LaserScan front_republished = *front_scan;
      front_republished.header.frame_id = front_frame_;

      sensor_msgs::LaserScan back_republished = *back_scan;
      back_republished.header.frame_id = back_frame_;

      publisher_front_.publish(front_republished);
      publisher_back_.publish(back_republished);
      /*publisher_cloud_old.publish(scan_old_);
      sensor_msgs::PointCloud ros_cloud =
          PointCloud2Ros(scan_registration_.getTransformedCloud());
      ros_cloud.header = front_republished.header;
      publisher_cloud_transformed.publish(ros_cloud);
      scan_old_ = front_republished;*/
    }
  }
  printf("ICP Time taken: %.3fs\n",
         (double)(clock() - tStart) / CLOCKS_PER_SEC);
}
PointCloud EigenListener::scan2Cloud(
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
bool EigenListener::readParameters() {
  if (!nodeHandle_->getParam("in_scan_topic_first", in_scan_topic_first_) ||
      !nodeHandle_->getParam("use_heuristic", use_heuristic_) ||
      !nodeHandle_->getParam("use_both_scans", use_both_scans_) ||
      !nodeHandle_->getParam("out_cloud_topic", out_cloud_topic_) ||
      !nodeHandle_->getParam("out_pose_topic", out_pose_topic_) ||
      !nodeHandle_->getParam("in_scan_topic_second", in_scan_topic_second_) ||
      !nodeHandle_->getParam("maximum_iterations", maximum_iterations_) ||
      !nodeHandle_->getParam("euclidean_fitness_epsilon",
                             euclidean_fitness_epsilon_) ||
      !nodeHandle_->getParam("transformation_epsilon",
                             transformation_epsilon_) ||
      !nodeHandle_->getParam("max_correspondence_distance",
                             max_correspondence_distance_)) {
    return false;
  }
  if (use_heuristic_) {
    if (!nodeHandle_->getParam("max_correspondence_angle",
                               max_correspondence_angle_)) {
      return false;
    }
  }
  max_correspondence_angle_ = max_correspondence_angle_ / 180 * M_PI;

  std::vector<float> calibration_vector;
  if (nodeHandle_->getParam("calibration_matrix", calibration_vector)) {
    back_to_front_transformation_ << calibration_vector[0],
        calibration_vector[1], calibration_vector[3], calibration_vector[4],
        calibration_vector[5], calibration_vector[7], 0, 0, 1;
    have_transformation_ = true;
    ROS_INFO_STREAM("Calibration from file: \n"
                    << back_to_front_transformation_);
    publishStaticTf(back_to_front_transformation_, front_frame_, back_frame_);
  }
  return true;
}

Matrix3f EigenListener::tfToEigen(const tf::Transform& transform) {
  Eigen::Affine3d affine_matrix;
  tf::transformTFToEigen(transform, affine_matrix);
  Matrix3f transform_eigen = Matrix3f::Identity();
  transform_eigen.block<2, 2>(0, 0) =
      affine_matrix.matrix().block<2, 2>(0, 0).cast<float>();
  transform_eigen.block<2, 1>(0, 2) =
      affine_matrix.matrix().block<2, 1>(0, 3).cast<float>();
  return transform_eigen;
}
void EigenListener::setScannerTransformation(const Matrix3f transformation) {
  back_to_front_transformation_ = transformation;
  have_transformation_ = true;
  publishStaticTf(back_to_front_transformation_, front_frame_, back_frame_);
}

void EigenListener::waitForTransformation(const std::string& back_frame,
                                          const std::string& front_frame) {
  tf::StampedTransform tf_transform;
  tf_listener_.waitForTransform(back_frame, front_frame, ros::Time(0),
                                ros::Duration(1e6));
  tf_listener_.lookupTransform(back_frame, front_frame, ros::Time(0),
                               tf_transform);
  back_to_front_transformation_ = tfToEigen(tf_transform);
  have_transformation_ = true;

  publishStaticTf(back_to_front_transformation_, front_frame_, back_frame_);
}

PointCloud EigenListener::transformScans2Cloud(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  PointCloud front_cloud = scan2Cloud(front_scan);

  PointCloud back_cloud = EigenScanRegistration::transformPointCloud(
      scan2Cloud(back_scan), back_to_front_transformation_);

  PointCloud cloud(
      2, front_cloud.cols() + back_cloud.cols());  // we have to merge here both
  cloud << front_cloud, back_cloud;
  return cloud;
}

sensor_msgs::PointCloud EigenListener::PointCloud2Ros(const PointCloud& cloud) {
  sensor_msgs::PointCloud ros_cloud;
  ros_cloud.points.resize(cloud.cols());
  for (int col = 0; col < cloud.cols(); ++col) {
    ros_cloud.points[col].x = cloud(0, col);
    ros_cloud.points[col].y = cloud(1, col);
    ros_cloud.points[col].z = 0;
  }
  return ros_cloud;
}

nav_msgs::Odometry EigenListener::Eigen2Pose(const Eigen::Matrix3f& odometry) {
  nav_msgs::Odometry pose;
  Eigen::Affine3f transformation;
  transformation.matrix() = Eigen::Matrix4f::Identity();
  transformation.matrix().block<2, 2>(0, 0) = odometry.block<2, 2>(0, 0);
  transformation.matrix().block<2, 1>(0, 3) = odometry.block<2, 1>(0, 2);
  tf::poseEigenToMsg(transformation.cast<double>(), pose.pose.pose);
  return pose;
}

void EigenListener::publishOdom2Tf(const nav_msgs::Odometry& odometry,
                                   const std::string& source_frame) {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(odometry.pose.pose.position.x,
                                  odometry.pose.pose.position.y,
                                  odometry.pose.pose.position.z));

  transform.setRotation(tf::Quaternion(
      odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y,
      odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w));

  tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform, odometry.header.stamp, source_frame,
                           odometry.header.frame_id));
  // std::cout << "frame: " << source_frame
  //          << " in frame: " << odometry.header.frame_id << std::endl;
}

void EigenListener::publishStaticTf(const Eigen::Matrix3f& transformation,
                                    const std::string& frame_id,
                                    const std::string& child_frame_id) {
  Eigen::Affine3f affine_transformation;
  affine_transformation.matrix() = Eigen::Matrix4f::Identity();
  affine_transformation.matrix().block<2, 2>(0, 0) =
      transformation.block<2, 2>(0, 0);
  affine_transformation.matrix().block<2, 1>(0, 3) =
      transformation.block<2, 1>(0, 2);

  geometry_msgs::TransformStamped ros_transformation;
  tf::transformEigenToMsg(affine_transformation.cast<double>(),
                          ros_transformation.transform);
  ros_transformation.header.frame_id = frame_id;
  ros_transformation.child_frame_id = child_frame_id;

  static_broadcaster_.sendTransform(ros_transformation);
  ROS_INFO_STREAM("static transformation: frame_id: "
                  << frame_id << " child_frame_id: " << child_frame_id);
}