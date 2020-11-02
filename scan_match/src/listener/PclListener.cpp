#include "listener/PclListener.hpp"
typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef pcl::PointCloud<pcl::PointXYZ> pcl_PointCloud;

PclListener::PclListener(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle) {
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  pcl_scan_registration_ = PCLScanRegistration(
      maximum_iterations_, euclidean_fitness_epsilon_, transformation_epsilon_,
      max_correspondence_distance_);

  first_scan_sub_.subscribe(nodeHandle, in_scan_topic_first_, 1);
  second_scan_sub_.subscribe(nodeHandle, in_scan_topic_second_, 1);

  // sync policy
  sync_ = std::unique_ptr<sync_t>(
      new sync_t(sync_pol_t(10), first_scan_sub_, second_scan_sub_));
  if (calibrate_) {
    sync_->registerCallback(
        boost::bind(&PclListener::calibrationCallback, this, _1, _2));
  } else {
    sync_->registerCallback(
        boost::bind(&PclListener::odometryCallback, this, _1, _2));
  }
  publisher_cloud_ =
      nodeHandle.advertise<sensor_msgs::PointCloud2>(out_cloud_topic_, 2);
  publisher_back_ =
      nodeHandle.advertise<sensor_msgs::LaserScan>("/back_scan", 10);
  publisher_front_ =
      nodeHandle.advertise<sensor_msgs::LaserScan>("/front_scan", 10);

  // serviceServer_ = nodeHandle_.advertiseService(
  //    "finish_calibration", &PclListener::serviceCallback, this);

  ROS_INFO("Successfully launched node.");
}
PclListener::PclListener(ros::NodeHandle& nodeHandle, bool calibrate)
    : nodeHandle_(nodeHandle), calibrate_(calibrate) {
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  pcl_scan_registration_ = PCLScanRegistration(
      maximum_iterations_, euclidean_fitness_epsilon_, transformation_epsilon_,
      max_correspondence_distance_);

  first_scan_sub_.subscribe(nodeHandle, in_scan_topic_first_, 1);
  second_scan_sub_.subscribe(nodeHandle, in_scan_topic_second_, 1);

  // sync policy
  sync_ = std::unique_ptr<sync_t>(
      new sync_t(sync_pol_t(10), first_scan_sub_, second_scan_sub_));
  if (calibrate_) {
    sync_->registerCallback(
        boost::bind(&PclListener::calibrationCallback, this, _1, _2));
  } else {
    sync_->registerCallback(
        boost::bind(&PclListener::odometryCallback, this, _1, _2));
  }
  publisher_cloud_ =
      nodeHandle.advertise<sensor_msgs::PointCloud2>(out_cloud_topic_, 2);
  publisher_back_ =
      nodeHandle.advertise<sensor_msgs::LaserScan>("/back_scan", 10);
  publisher_front_ =
      nodeHandle.advertise<sensor_msgs::LaserScan>("/front_scan", 10);

  // serviceServer_ = nodeHandle_.advertiseService(
  //    "finish_calibration", &PclListener::serviceCallback, this);

  ROS_INFO("Successfully launched node.");
}

PclListener::~PclListener() {}

bool PclListener::readParameters() {
  if (!nodeHandle_.getParam("in_scan_topic_first", in_scan_topic_first_) ||
      !nodeHandle_.getParam("out_cloud_topic", out_cloud_topic_) ||
      !nodeHandle_.getParam("in_scan_topic_second", in_scan_topic_second_) ||
      !nodeHandle_.getParam("maximum_iterations", maximum_iterations_) ||
      !nodeHandle_.getParam("use_both_scans", use_both_scans_) ||
      !nodeHandle_.getParam("euclidean_fitness_epsilon",
                            euclidean_fitness_epsilon_) ||
      !nodeHandle_.getParam("transformation_epsilon",
                            transformation_epsilon_) ||
      !nodeHandle_.getParam("max_correspondence_distance",
                            max_correspondence_distance_)) {
    return false;
  }
  if (!nodeHandle_.getParam("calibration_file", calibration_file_) &&
      calibrate_) {
    return false;
  }
  std::vector<float> calibration_vector;
  Eigen::Matrix4f calibration_matrix;
  if (nodeHandle_.getParam("calibration_matrix", calibration_vector)) {
    calibration_matrix << calibration_vector[0], calibration_vector[1],
        calibration_vector[2], calibration_vector[3], calibration_vector[4],
        calibration_vector[5], calibration_vector[6], calibration_vector[7],
        calibration_vector[8], calibration_vector[9], calibration_vector[10],
        calibration_vector[11], calibration_vector[12], calibration_vector[13],
        calibration_vector[14], calibration_vector[15];
    back_to_front_transformation_ = Matrix4ToTF(calibration_matrix);
    have_transformation_ = true;
    ROS_INFO_STREAM("Calibration from file: \n" << calibration_matrix);
    publishStaticTf(back_to_front_transformation_, front_frame_, back_frame_);
  }
  return true;
}

void PclListener::odometryCallback(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  clock_t tStart = clock();

  if (!have_transformation_) {
    waitForTransformation(back_scan->header.frame_id,
                          front_scan->header.frame_id);
  } else {
    sensor_msgs::PointCloud2 front_cloud;
    projector_.projectLaser(*front_scan, front_cloud);
    pcl_PointCloud::Ptr front_pcl_cloud(new pcl_PointCloud());
    pcl::fromROSMsg(front_cloud, *front_pcl_cloud);

    sensor_msgs::PointCloud2 back_cloud;
    projector_.projectLaser(*back_scan, back_cloud);
    pcl_PointCloud::Ptr back_pcl_cloud(new pcl_PointCloud());
    pcl::fromROSMsg(back_cloud, *back_pcl_cloud);

    pcl_PointCloud::Ptr back_pcl_cloud_transformed(new pcl_PointCloud());
    pcl_ros::transformPointCloud(*back_pcl_cloud, *back_pcl_cloud_transformed,
                                 back_to_front_transformation_);

    *front_pcl_cloud += *back_pcl_cloud_transformed;
    Eigen::Matrix4f odometry =
        pcl_scan_registration_.computeTransformation(front_pcl_cloud);
    std::cout << "odometry: \n" << odometry << std::endl;

    sensor_msgs::LaserScan front_republished = *front_scan;
    front_republished.header.frame_id = front_frame_;

    sensor_msgs::LaserScan back_republished = *back_scan;
    back_republished.header.frame_id = back_frame_;

    publisher_front_.publish(front_republished);
    publisher_back_.publish(back_republished);
    publishTf(Matrix4ToTF(odometry), front_frame_, "/world",
              front_scan->header);
  }
  printf("ICP Time taken: %.2fs\n",
         (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

void PclListener::calibrationCallback(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  clock_t tStart = clock();

  if (!have_transformation_) {
    waitForTransformation(back_scan->header.frame_id,
                          front_scan->header.frame_id);
  } else {
    sensor_msgs::PointCloud2 front_cloud;
    projector_.projectLaser(*front_scan, front_cloud);
    pcl_PointCloud::Ptr front_pcl_cloud(new pcl_PointCloud());
    pcl::fromROSMsg(front_cloud, *front_pcl_cloud);

    sensor_msgs::PointCloud2 back_cloud;
    projector_.projectLaser(*back_scan, back_cloud);
    pcl_PointCloud::Ptr back_pcl_cloud(new pcl_PointCloud());
    pcl::fromROSMsg(back_cloud, *back_pcl_cloud);

    pcl_PointCloud::Ptr back_pcl_cloud_transformed(new pcl_PointCloud());
    pcl_ros::transformPointCloud(*back_pcl_cloud, *back_pcl_cloud_transformed,
                                 back_to_front_transformation_);

    pcl_scan_registration_.calibrate(front_pcl_cloud,
                                     back_pcl_cloud_transformed);

    *front_pcl_cloud += *(pcl_scan_registration_.getTransformedCloud());

    sensor_msgs::PointCloud2 publish_msg;
    pcl::toROSMsg(*front_pcl_cloud, publish_msg);
    publish_msg.header = front_scan->header;
    publisher_cloud_.publish(publish_msg);
    saveCalibration(calibration_file_);
  }
  printf("ICP Time taken: %.2fs\n",
         (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

bool PclListener::serviceCallback(std_srvs::Trigger::Request& request,
                                  std_srvs::Trigger::Response& response) {
  response.success = true;
  response.message = "stopping calibration, starting odometry";
  pcl_scan_registration_.final_calibration();
  pcl_scan_registration_.resetTransformation();
  calibrate_ = false;

  return true;
}

void PclListener::waitForTransformation(const std::string& source_frame,
                                        const std::string& target_frame) {
  tf::StampedTransform tf_transform;
  tf_listener_.waitForTransform(source_frame, target_frame, ros::Time(0),
                                ros::Duration(1e6));
  tf_listener_.lookupTransform(source_frame, target_frame, ros::Time(0),
                               tf_transform);
  back_to_front_transformation_ = tf_transform;
  have_transformation_ = true;
}

void PclListener::saveCalibration(const std::string& calibration_file) {
  std::ofstream file(calibration_file);
  Eigen::IOFormat eigen_format(Eigen::FullPrecision, Eigen::DontAlignCols, ",",
                               ",", "", "", "[", "]");
  Eigen::Matrix4f final_calibration =
      pcl_scan_registration_.final_calibration() *
      TF2Matrix4(back_to_front_transformation_);
  file << "calibration_matrix: " << final_calibration.format(eigen_format);
}

Eigen::Matrix4f PclListener::TF2Matrix4(const tf::Transform& transformation) {
  Eigen::Matrix4f matrix;
  matrix << transformation.getBasis()[0][0], transformation.getBasis()[0][1],
      transformation.getBasis()[0][2], transformation.getOrigin().x(),
      transformation.getBasis()[1][0], transformation.getBasis()[1][1],
      transformation.getBasis()[1][2], transformation.getOrigin().y(),
      transformation.getBasis()[2][0], transformation.getBasis()[2][1],
      transformation.getBasis()[2][2], transformation.getOrigin().z(), 0, 0, 0,
      1;
  return matrix;
}
tf::Transform PclListener::Matrix4ToTF(const Eigen::Matrix4f& matrix) {
  tf::Matrix3x3 rotation_matrix;
  rotation_matrix.setValue(
      static_cast<double>(matrix(0, 0)), static_cast<double>(matrix(0, 1)),
      static_cast<double>(matrix(0, 2)), static_cast<double>(matrix(1, 0)),
      static_cast<double>(matrix(1, 1)), static_cast<double>(matrix(1, 2)),
      static_cast<double>(matrix(2, 0)), static_cast<double>(matrix(2, 1)),
      static_cast<double>(matrix(2, 2)));
  tf::Quaternion rotation_tf;
  rotation_matrix.getRotation(rotation_tf);

  tf::Vector3 translation_tf(matrix(0, 3), matrix(1, 3), matrix(2, 3));
  tf::Transform transformation(rotation_tf, translation_tf);
  return transformation;
}

void PclListener::publishStaticTf(const tf::Transform& transformation,
                                  const std::string& frame_id,
                                  const std::string& child_frame_id) {
  Eigen::Affine3f affine_transformation;
  affine_transformation.matrix() = TF2Matrix4(transformation);

  geometry_msgs::TransformStamped ros_transformation;
  tf::transformEigenToMsg(affine_transformation.cast<double>(),
                          ros_transformation.transform);
  ros_transformation.header.frame_id = frame_id;
  ros_transformation.child_frame_id = child_frame_id;

  static_broadcaster_.sendTransform(ros_transformation);
  ROS_INFO_STREAM("static transformation: frame_id: "
                  << frame_id << " child_frame_id: " << child_frame_id);
}
void PclListener::publishTf(const tf::Transform& transformation,
                            const std::string& frame_id,
                            const std::string& child_frame_id,
                            const std_msgs::Header& header) {
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      transformation, header.stamp, frame_id, child_frame_id));
}