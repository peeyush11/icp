#include "listener/ScanHandler.h"

ScanHandler::ScanHandler(ros::NodeHandle& node_handle)
    : node_handle_(&node_handle) {
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  ROS_INFO_STREAM("launched node");
}
void ScanHandler::startListening() {
  publisher_front_ =
      node_handle_->advertise<sensor_msgs::LaserScan>("/front_scan", 10);
  publisher_back_ =
      node_handle_->advertise<sensor_msgs::LaserScan>("/back_scan", 10);
  if (use_both_scans_) {
    front_scan_sub_.subscribe(*node_handle_, in_scan_topic_front_, 1);
    back_scan_sub_.subscribe(*node_handle_, in_scan_topic_back_, 1);

    sync_ = std::unique_ptr<sync_t>(
        new sync_t(sync_pol_t(10), front_scan_sub_, back_scan_sub_));
    if (calibrate_) {
      sync_->registerCallback(
          boost::bind(&ScanHandler::calibrationCallback, this, _1, _2));
    } else {
      sync_->registerCallback(
          boost::bind(&ScanHandler::scanOdometryCallback, this, _1, _2));
    }
  } else {
    ROS_INFO_STREAM("1 scan callback not implemented yet");
  }
  ROS_INFO_STREAM("start listening");
}

bool ScanHandler::readParameters() {
  if (!node_handle_->getParam("method", method_) ||
      !node_handle_->getParam("calibrate", calibrate_) ||
      !node_handle_->getParam("calibration_file", calibration_file_) ||
      !node_handle_->getParam("in_scan_topic_front", in_scan_topic_front_) ||
      !node_handle_->getParam("maximum_iterations", maximum_iterations_) ||
      !node_handle_->getParam("euclidean_fitness_epsilon",
                              euclidean_fitness_epsilon_) ||
      !node_handle_->getParam("transformation_epsilon",
                              transformation_epsilon_) ||
      !node_handle_->getParam("max_correspondence_distance",
                              max_correspondence_distance_)) {
    return false;
  }
  // use both scans?
  if (!node_handle_->getParam("in_scan_topic_back", in_scan_topic_back_)) {
    use_both_scans_ = false;
    ROS_INFO_STREAM("using just 1 scan");
  }
  // Initialize Strategy
  ROS_INFO_STREAM("ScanMatchingStrategy: " << method_);
  if (!initStrategy()) {
    ROS_ERROR_STREAM("could not Initialize strategy!");
    return false;
  }
  // Load CalibrationMatrix
  std::vector<float> calibration_vector;
  if (node_handle_->getParam("calibration_matrix", calibration_vector)) {
    back_to_front_transformation_ << calibration_vector[0],
        calibration_vector[1], calibration_vector[2], calibration_vector[3],
        calibration_vector[4], calibration_vector[5], calibration_vector[6],
        calibration_vector[7], calibration_vector[8], calibration_vector[9],
        calibration_vector[10], calibration_vector[11], calibration_vector[12],
        calibration_vector[13], calibration_vector[14], calibration_vector[15];
    have_transformation_ = true;
    ROS_INFO_STREAM("Calibration from file: \n"
                    << back_to_front_transformation_);

    strategy_->setTransformation(back_to_front_transformation_);
    if (!calibrate_) {
      publishStaticTf(back_to_front_transformation_, front_frame_, back_frame_);
    }
  }
  return true;
}
bool ScanHandler::initStrategy() {
  if (method_ == "nearest") {
    std::cout << "take nearest" << std::endl;
    strategy_ = new NearestNeighbor(
        maximum_iterations_, euclidean_fitness_epsilon_,
        transformation_epsilon_, max_correspondence_distance_);
  }
  if (method_ == "pcl") {
    strategy_ =
        new PclStrategy(maximum_iterations_, euclidean_fitness_epsilon_,
                        transformation_epsilon_, max_correspondence_distance_);
  }
  if (method_ == "heuristic") {
    // heuristic parameter
    float max_correspondence_angle = 0;
    if (!node_handle_->getParam("max_correspondence_angle",
                                max_correspondence_angle)) {
      return false;
    }
    max_correspondence_angle = max_correspondence_angle / 180 * M_PI;
    strategy_ = new HeuristicStrategy(
        maximum_iterations_, euclidean_fitness_epsilon_,
        transformation_epsilon_, max_correspondence_distance_,
        max_correspondence_angle);
    //...
  }
  if (method_ == "plane") {
    strategy_ =
        new LeastSquare(maximum_iterations_, euclidean_fitness_epsilon_,
                        transformation_epsilon_, max_correspondence_distance_);
  }
  return true;
}

void ScanHandler::scanOdometryCallback(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  clock_t tStart = clock();
  std::cout << "\n\n ---------------msgs received---------" << std::endl;
  if (!have_transformation_) {
    waitForTransformation(back_scan->header.frame_id,
                          front_scan->header.frame_id);
    publishStaticTf(back_to_front_transformation_, front_frame_, back_frame_);

  } else {
    tf::Transform odometry = strategy_->computeOdometry(front_scan, back_scan);

    publishTf(odometry, front_frame_, "world", front_scan->header);

    sensor_msgs::LaserScan front_republished = *front_scan;
    front_republished.header.frame_id = front_frame_;

    sensor_msgs::LaserScan back_republished = *back_scan;
    back_republished.header.frame_id = back_frame_;

    publisher_front_.publish(front_republished);
    publisher_back_.publish(back_republished);
  }
  printf("ICP Time taken: %.3fs\n",
         (double)(clock() - tStart) / CLOCKS_PER_SEC);
  ++number_transformations_;
  accumulated_time_ += (double)(clock() - tStart) / CLOCKS_PER_SEC;

  ROS_INFO_STREAM("average ICP time: " << accumulated_time_ /
                                              number_transformations_);
}

void ScanHandler::calibrationCallback(
    const sensor_msgs::LaserScan::ConstPtr& front_scan,
    const sensor_msgs::LaserScan::ConstPtr& back_scan) {
  clock_t tStart = clock();
  std::cout << "\n\n ---------------msgs received---------" << std::endl;
  if (!have_transformation_) {
    waitForTransformation(back_scan->header.frame_id,
                          front_scan->header.frame_id);

  } else {
    Eigen::Matrix4f calibration =
        strategy_->computeCalibration(front_scan, back_scan);
    saveCalibration(calibration, calibration_file_);
    publishTf(Matrix4ToTF(calibration), front_frame_, back_frame_,
              front_scan->header);

    sensor_msgs::LaserScan front_republished = *front_scan;
    front_republished.header.frame_id = front_frame_;

    sensor_msgs::LaserScan back_republished = *back_scan;
    back_republished.header.frame_id = back_frame_;

    publisher_front_.publish(front_republished);
    publisher_back_.publish(back_republished);
  }
  printf("ICP Time taken: %.3fs\n",
         (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

void ScanHandler::publishStaticTf(const Eigen::Matrix4f& transformation,
                                  const std::string& frame_id,
                                  const std::string& child_frame_id) {
  Eigen::Affine3f affine_transformation;
  affine_transformation.matrix() = transformation;

  geometry_msgs::TransformStamped ros_transformation;
  tf::transformEigenToMsg(affine_transformation.cast<double>(),
                          ros_transformation.transform);
  ros_transformation.header.frame_id = frame_id;
  ros_transformation.child_frame_id = child_frame_id;

  static_broadcaster_.sendTransform(ros_transformation);
}
void ScanHandler::publishTf(const tf::Transform& transformation,
                            const std::string& frame_id,
                            const std::string& child_frame_id,
                            const std_msgs::Header& header) {
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      transformation, header.stamp, frame_id, child_frame_id));
}

void ScanHandler::waitForTransformation(const std::string& back_frame,
                                        const std::string& front_frame) {
  tf::StampedTransform tf_transform;
  tf_listener_.waitForTransform(back_frame, front_frame, ros::Time(0),
                                ros::Duration(1e6));
  tf_listener_.lookupTransform(back_frame, front_frame, ros::Time(0),
                               tf_transform);
  back_to_front_transformation_ = TF2Matrix4(tf_transform);
  have_transformation_ = true;

  strategy_->setTransformation(back_to_front_transformation_);
}

void ScanHandler::saveCalibration(const Eigen::Matrix4f& calibration,
                                  const std::string& calibration_file) {
  std::ofstream file(calibration_file);
  Eigen::IOFormat eigen_format(Eigen::FullPrecision, Eigen::DontAlignCols, ",",
                               ",", "", "\n", "[", "]");
  file << "calibration_matrix: " << calibration.format(eigen_format);
}