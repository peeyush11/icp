#include "EigenScanRegistration.hpp"

typedef Matrix<float, 2, Dynamic> PointCloud;
typedef Matrix<float, 2, 1> Point;

using std::cout;
using std::endl;

EigenScanRegistration::EigenScanRegistration() {
  final_transformation_ = Eigen::Matrix3f::Identity();
}
EigenScanRegistration::~EigenScanRegistration() {}
EigenScanRegistration::EigenScanRegistration(
    const int& maximum_iterations, const float& euclidean_fitness_epsilon,
    const float& transformation_epsilon,
    const float& max_correspondence_distance)
    : maximum_iterations_{maximum_iterations},
      euclidean_fitness_epsilon_{euclidean_fitness_epsilon},
      transformation_epsilon_{transformation_epsilon},
      max_correspondence_distance_{max_correspondence_distance} {
  final_transformation_ = Eigen::Matrix3f::Identity();
};

void EigenScanRegistration::align() {
  Eigen::Matrix2f cross_covariance_matrix;
  Eigen::Matrix3f transformation_matrix;
  float error = 1e9;
  float error_change = 1e9;
  int iterations = 0;
  while (error > euclidean_fitness_epsilon_ &&
         error_change > transformation_epsilon_) {
    cross_covariance_matrix = findCorrespondencies();

    transformation_matrix = computeTransformation(cross_covariance_matrix);

    final_transformation_ = transformation_matrix * final_transformation_;
    source_ = transformPointCloud(source_, transformation_matrix);
    float new_error =
        computeError() / correspondencies_.cols();  // normalization
    error_change = std::abs(error - new_error);
    error = new_error;
    ++iterations;
    if (iterations > maximum_iterations_) {
      cout << "break because of too many iterations" << endl;
      break;
    }
  }
}

Eigen::Matrix2f EigenScanRegistration::findCorrespondencies() {
  // need to fill source_mean_ and target_mean_ and correspondencies_
  int K = 1;
  int cols = source_.cols();
  PointCloud corresponding_points_source(2, cols);
  PointCloud corresponding_points_target(2, cols);
  correspondencies_.resize(2, cols);
  int number_correspondencies = 0;

  Eigen::Matrix<float, 1, Dynamic> squared_distances(1, target_.cols());
  Eigen::Matrix<float, 1, Dynamic>::Index min_index;
  // Loop over all source Points
  for (int col = 0; col < cols; ++col) {
    squared_distances =
        (target_.colwise() - source_.col(col)).colwise().squaredNorm();

    if (squared_distances.minCoeff(&min_index) <
        max_correspondence_distance_ * max_correspondence_distance_) {
      corresponding_points_source.col(number_correspondencies) =
          source_.col(col);
      corresponding_points_target.col(number_correspondencies) =
          target_.col(min_index);
      correspondencies_(0, number_correspondencies) = min_index;
      correspondencies_(1, number_correspondencies) = col;

      number_correspondencies++;
    }
  }
  corresponding_points_source.conservativeResize(2, number_correspondencies);
  corresponding_points_target.conservativeResize(2, number_correspondencies);
  correspondencies_.conservativeResize(2, number_correspondencies);

  source_mean_ = corresponding_points_source.rowwise().mean();
  corresponding_points_source.colwise() -= source_mean_;

  target_mean_ = corresponding_points_target.rowwise().mean();
  corresponding_points_target.colwise() -= target_mean_;

  Eigen::Matrix2f cross_covariance_matrix =
      corresponding_points_target * corresponding_points_source.transpose();

  return cross_covariance_matrix;
}


Eigen::Matrix3f EigenScanRegistration::computeTransformation(
    const Eigen::Matrix2f& cross_covariance_matrix) {
  Eigen::Matrix3f transformation_matrix = Eigen::Matrix3f::Identity();

  Eigen::JacobiSVD<Eigen::Matrix2f> svd(
      cross_covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2f rotation_matrix = svd.matrixU() * svd.matrixV().transpose();
  Eigen::Vector2f translation = target_mean_ - (rotation_matrix * source_mean_);
  transformation_matrix.block<2, 2>(0, 0) = rotation_matrix;
  transformation_matrix.block<2, 1>(0, 2) = translation;

  return transformation_matrix;
}
PointCloud EigenScanRegistration::transformPointCloud(
    const PointCloud& cloud, const Eigen::Matrix3f& transformation) {
  PointCloud transformed_cloud = transformation.block<2, 2>(0, 0) * cloud;
  transformed_cloud.colwise() += transformation.block<2, 1>(0, 2);
  return transformed_cloud;
}
PointCloud EigenScanRegistration::transformPointCloud(
    const PointCloud& cloud, const Eigen::Matrix4f& transformation) {
  PointCloud transformed_cloud = transformation.block<2, 2>(0, 0) * cloud;
  transformed_cloud.colwise() += transformation.block<2, 1>(0, 3);
  return transformed_cloud;
}
float EigenScanRegistration::computeError() {
  float error = 0;
  for (int col = 0; col < correspondencies_.cols(); ++col) {
    error += (target_.col(correspondencies_(0, col)) -
              source_.col(correspondencies_(1, col)))
                 .squaredNorm();
  }
  return error;
}
void EigenScanRegistration::setInputSource(PointCloud source) {
  source_ = source;
}
void EigenScanRegistration::setInputTarget(const PointCloud& target) {
  target_ = target;
}
void EigenScanRegistration::setMaxCorrespondenceDistance(
    const float& max_correspondence_distance) {
  max_correspondence_distance_ = max_correspondence_distance;
}
void EigenScanRegistration::setMaximumIterations(
    const int& maximum_iterations) {
  maximum_iterations_ = maximum_iterations;
}
void EigenScanRegistration::setTransformationEpsilon(
    const float& transformation_epsilon) {
  transformation_epsilon_ = transformation_epsilon;
}
void EigenScanRegistration::setEuclideanFitnessEpsilon(
    const float& euclidean_fitness_epsilon) {
  euclidean_fitness_epsilon_ = euclidean_fitness_epsilon_;
}
void EigenScanRegistration::setMaxCorrespondenceAngle(
    const float& max_correspondence_angle) {
  max_correspondence_angle_ = max_correspondence_angle;
}
Eigen::Matrix3f EigenScanRegistration::getFinalTransformation() {
  return final_transformation_;
}

bool EigenScanRegistration::comparePointClouds(
    const Matrix<float, Dynamic, Dynamic>& first_cloud,
    const Matrix<float, Dynamic, Dynamic>& second_cloud,
    const float& threshold) {
  return !((first_cloud - second_cloud).array().abs() > threshold).any();
}

PointCloud EigenScanRegistration::getTransformedCloud() { return source_; }

void EigenScanRegistration::setNumberNeighbors(const int& number_neighbours) {
  std::cerr << "This Function is only for Heurist registration!" << std::endl;
}
void EigenScanRegistration::resetTransformation() {
  final_transformation_ = Eigen::Matrix3f::Identity();
}
