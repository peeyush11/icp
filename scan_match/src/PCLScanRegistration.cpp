#include "PCLScanRegistration.hpp"

typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef pcl::PointCloud<pcl::PointXYZ> pcl_PointCloud;

PCLScanRegistration::PCLScanRegistration() {
  first_cloud_ = pcl_PointCloud::Ptr(new pcl_PointCloud);
  second_cloud_ = pcl_PointCloud::Ptr(new pcl_PointCloud);
  cloud_transformed_ = pcl_PointCloud::Ptr(new pcl_PointCloud);

  accumulated_transformation_ = Eigen::Matrix4f::Identity();
}

PCLScanRegistration::PCLScanRegistration(
    const int& maximum_iterations, const float& euclidean_fitness_epsilon,
    const float& transformation_epsilon,
    const float& max_correspondence_distance)
    : maximum_iterations_{maximum_iterations},
      euclidean_fitness_epsilon_{euclidean_fitness_epsilon},
      transformation_epsilon_{transformation_epsilon},
      max_correspondence_distance_{max_correspondence_distance} {
  first_cloud_ = pcl_PointCloud::Ptr(new pcl_PointCloud);
  second_cloud_ = pcl_PointCloud::Ptr(new pcl_PointCloud);
  cloud_transformed_ = pcl_PointCloud::Ptr(new pcl_PointCloud);

  accumulated_transformation_ = Eigen::Matrix4f::Identity();
}

PCLScanRegistration::~PCLScanRegistration() {}

Matrix4f PCLScanRegistration::computeTransformation(
    pcl_PointCloud::Ptr& new_cloud) {
  if (first_cloud_->empty()) {
    // std::cout << "no error" << std::endl;
    second_cloud_ = new_cloud;
    first_cloud_ = new_cloud;
    // std::cout << "no error" << std::endl;
    return accumulated_transformation_;

  } else {
    first_cloud_ = second_cloud_;
    second_cloud_ = new_cloud;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // Set the input source and target
    icp.setInputSource(first_cloud_);
    icp.setInputTarget(second_cloud_);
    // Set the max correspondence distance to 5cm (e.g., correspondences with
    // higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(maximum_iterations_);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(transformation_epsilon_);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
    // Perform the alignment
    pcl_PointCloud cloud_transformed;
    icp.align(cloud_transformed);
    std::cout << "has converged:" << icp.hasConverged()
              << " score: " << icp.getFitnessScore() << std::endl;
    accumulated_transformation_ =
        icp.getFinalTransformation() * accumulated_transformation_;
    std::cout << "relative transformation:" << std::endl
              << icp.getFinalTransformation() << std::endl;
    std::cout << "absolute transformation:" << std::endl
              << accumulated_transformation_ << std::endl;
    return accumulated_transformation_;
  }
}

Matrix4f PCLScanRegistration::calibrate(const pcl_PointCloud::Ptr& first_cloud,
                                        const pcl_PointCloud::Ptr& second_cloud) {
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // Set the input source and target
  icp.setInputSource(second_cloud);
  icp.setInputTarget(first_cloud);
  // Set the max correspondence distance to 5cm (e.g., correspondences with
  // higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(maximum_iterations_);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(transformation_epsilon_);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
  // Perform the alignment
  icp.align(*cloud_transformed_);
  std::cout << "has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
  accumulated_transformation_ += icp.getFinalTransformation();
  std::cout << "transformation of this period:" << std::endl
            << icp.getFinalTransformation() << std::endl;
  transformation_counter_ += 1;
  return final_calibration();
}

Matrix4f PCLScanRegistration::final_calibration() {
  Eigen::Matrix3f rotation_matrix_sum =
      accumulated_transformation_.block<3, 3>(0, 0);
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(
      rotation_matrix_sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f rotation_matrix_average =
      svd.matrixU() * svd.matrixV().transpose();

  Eigen::Vector3f translation_averae =
      accumulated_transformation_.block<3, 1>(0, 3);
  translation_averae /= transformation_counter_;
  Matrix4f average_transformation;
  average_transformation << rotation_matrix_average(0, 0),
      rotation_matrix_average(0, 1), rotation_matrix_average(0, 2),
      translation_averae(0), rotation_matrix_average(1, 0),
      rotation_matrix_average(1, 1), rotation_matrix_average(1, 2),
      translation_averae(1), rotation_matrix_average(2, 0),
      rotation_matrix_average(2, 1), rotation_matrix_average(2, 2),
      translation_averae(2), 0, 0, 0, 1;
  std::cout << "average transformation:\n"
            << average_transformation << std::endl;
  return average_transformation;
}

void PCLScanRegistration::resetTransformation() {
  accumulated_transformation_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  std::cout << "accumulated_transformation:\n"
            << accumulated_transformation_ << std::endl;
}
void PCLScanRegistration::initTransformation(
    const Matrix4f& initial_transformation) {
  accumulated_transformation_ = initial_transformation;
}
pcl_PointCloud::Ptr PCLScanRegistration::getTransformedCloud() {
  return cloud_transformed_;
}
