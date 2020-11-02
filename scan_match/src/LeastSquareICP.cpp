#include "LeastSquareICP.hpp"

using std::cout;
using std::endl;

LeastSquareICP::LeastSquareICP() {
  final_transformation_ = Eigen::Matrix3f::Identity();
}

LeastSquareICP::~LeastSquareICP() {}
LeastSquareICP::LeastSquareICP(const int& maximum_iterations,
                               const float& euclidean_fitness_epsilon,
                               const float& transformation_epsilon,
                               const float& max_correspondence_distance)
    : maximum_iterations_{maximum_iterations},
      euclidean_fitness_epsilon_{euclidean_fitness_epsilon},
      transformation_epsilon_{transformation_epsilon},
      max_correspondence_distance_{max_correspondence_distance} {
  final_transformation_ = Eigen::Matrix3f::Identity();
};

void LeastSquareICP::align() {
  Eigen::Matrix3f transformation_matrix;

  Eigen::MatrixXf x(3, 1);

  int iterations = 0;
  float error = 1e9;
  float error_change = 1e9;

  NormalEstimation::Normal N(target_.cols());
  N.GetNormal(target_);

  // N.PrintNormal();

  while (error > euclidean_fitness_epsilon_ &&
         error_change > transformation_epsilon_) {
    PointCloud corresponding_points_source, corresponding_points_target,
        target_normal;

    Correspondencies(N, corresponding_points_source,
                     corresponding_points_target, target_normal);

    x = LSicp(corresponding_points_source, corresponding_points_target,
              target_normal);

    transformation_matrix << cos(x(2, 0)), sin(x(2, 0)), x(0, 0), -sin(x(2, 0)),
        cos(x(2, 0)), x(1, 0), 0, 0, 1;

    final_transformation_ = transformation_matrix * final_transformation_;

    source_ = transformPointCloud(source_, transformation_matrix);

    float new_error = computeError(target_normal) /
                      corresponding_points_source.cols();  // normalization
    error_change = std::abs(error - new_error);
    error = new_error;

    ++iterations;
    if (iterations > maximum_iterations_) {
      cout << "break because of too many iterations" << endl;
      break;
    }
  }
}

void LeastSquareICP::Correspondencies(NormalEstimation::Normal N,
                                      PointCloud& corresponding_points_source,
                                      PointCloud& corresponding_points_target,
                                      PointCloud& target_normal) {
  int K = 1;
  corresponding_points_source = PointCloud(2, source_.cols());
  corresponding_points_target = PointCloud(2, source_.cols());
  target_normal = PointCloud(2, source_.cols());

  int cols = N.length;  // size of unsampled normal

  Eigen::ArrayXXf index(1, cols);
  index.setZero();

  correspondencies_.resize(2, source_.cols());
  int number_correspondencies = 0;

  Eigen::Matrix<float, 2, 1> p;
  Eigen::Matrix<float, 2, 1> n;

  Eigen::Matrix<float, 1, Dynamic> squared_distances(1, target_.cols());
  Eigen::Matrix<float, 1, Dynamic>::Index min_index;

  for (int i = 0; i < source_.cols(); ++i) {
    squared_distances =
        (target_.colwise() - source_.col(i)).colwise().squaredNorm();

    if (squared_distances.minCoeff(&min_index) <
        max_correspondence_distance_ * max_correspondence_distance_) {
      corresponding_points_source.col(number_correspondencies) = source_.col(i);
      corresponding_points_target.col(number_correspondencies) =
          target_.col(min_index);
      target_normal(0, number_correspondencies) = N.normal[min_index].nx;
      target_normal(1, number_correspondencies) = N.normal[min_index].ny;

      correspondencies_(0, number_correspondencies) = i;
      correspondencies_(1, number_correspondencies) = min_index;

      number_correspondencies++;
    }
  }

  corresponding_points_source.conservativeResize(2, number_correspondencies);
  corresponding_points_target.conservativeResize(2, number_correspondencies);
  correspondencies_.conservativeResize(2, number_correspondencies);
  target_normal.conservativeResize(2, number_correspondencies);
}

float LeastSquareICP::dist2plane(const Eigen::Matrix<float, 2, 1>& p,
                                 const Eigen::Matrix<float, 2, 1>& n,
                                 const Eigen::Matrix<float, 2, 1>& source) {
  float dist = std::abs((source - p).transpose() * n);
  return dist;
}

Eigen::Matrix<float, 3, 1> LeastSquareICP::LSicp(PointCloud& source,
                                                 PointCloud& target,
                                                 PointCloud& normal) {
  int col = source.cols();

  Eigen::MatrixXf x(3, 1);
  Eigen::MatrixXf dx(3, 1);

  x << 0, 0, 0;  // Initial parameters

  Eigen::MatrixXf e(1, col);
  Eigen::MatrixXf e0(1, col);
  Eigen::MatrixXf e_est(1, col);
  Eigen::MatrixXf en(1, col);
  Eigen::MatrixXf de(1, col);
  Eigen::ArrayXXf check(1, col);

  e.setZero();
  int iterations = 0;

  do {
    Eigen::MatrixXf R(2, 2);
    R << cos(x(2, 0)), sin(x(2, 0)), -sin(x(2, 0)), cos(x(2, 0));

    Eigen::MatrixXf Rp(2, 2);
    Rp << -sin(x(2, 0)), cos(x(2, 0)), -cos(x(2, 0)), -sin(x(2, 0));

    Eigen::MatrixXf A(col, 3);
    Eigen::MatrixXf At(3, col);

    for (int i = 0; i < col; ++i) {
      e0(0, i) = ((R * source.col(i) + x.block<2, 1>(0, 0)) - target.col(i))
                     .transpose() *
                 normal.col(i);

      A(i, 0) = normal(0, i);
      A(i, 1) = normal(1, i);
      A(i, 2) = (Rp * source.col(i)).transpose() * normal.col(i);
    }

    de = e - e0;
    At = A.transpose();

    dx = ((At * A).inverse()) * At * de.transpose();

    x = x + dx;

    e_est = e0 + (A * dx).transpose();
    e = e_est;

    ++iterations;
    if (Eigen::isnan(dx.array()).any() || Eigen::isnan(x.array()).any() ||
        Eigen::isnan(e.array()).any() || Eigen::isnan(A.array()).any()) {
      x << 0, 0, 0;
      break;
    }
  } while ((dx.array().abs() > 1e-7).any() && iterations < 40);

  return x;
}

PointCloud LeastSquareICP::transformPointCloud(
    const PointCloud& cloud, const Eigen::Matrix3f& transformation) {
  PointCloud transformed_cloud = transformation.block<2, 2>(0, 0) * cloud;
  transformed_cloud.colwise() += transformation.block<2, 1>(0, 2);
  return transformed_cloud;
}
PointCloud LeastSquareICP::transformPointCloud(
    const PointCloud& cloud, const Eigen::Matrix4f& transformation) {
  PointCloud transformed_cloud = transformation.block<2, 2>(0, 0) * cloud;
  transformed_cloud.colwise() += transformation.block<2, 1>(0, 3);
  return transformed_cloud;
}
float LeastSquareICP::computeError(PointCloud& normal) {
  float error = 0;
  for (int col = 0; col < correspondencies_.cols(); ++col) {
    error += std::abs((source_.col(correspondencies_(0, col)) -
                       target_.col(correspondencies_(1, col)))
                          .transpose() *
                      normal.col(col));
  }

  return error;
}
void LeastSquareICP::setInputSource(PointCloud source) { source_ = source; }
void LeastSquareICP::setInputTarget(const PointCloud& target) {
  target_ = target;
}
void LeastSquareICP::setMaxCorrespondenceDistance(
    const float& max_correspondence_distance) {
  max_correspondence_distance_ = max_correspondence_distance;
}
void LeastSquareICP::setMaximumIterations(const int& maximum_iterations) {
  maximum_iterations_ = maximum_iterations;
}
void LeastSquareICP::setTransformationEpsilon(
    const float& transformation_epsilon) {
  transformation_epsilon_ = transformation_epsilon;
}
void LeastSquareICP::setEuclideanFitnessEpsilon(
    const float& euclidean_fitness_epsilon) {
  euclidean_fitness_epsilon_ = euclidean_fitness_epsilon_;
}
bool LeastSquareICP::hasConverged() { return converged_; }
float LeastSquareICP::getFitnessScore() { return fitness_score_; }
Eigen::Matrix3f LeastSquareICP::getFinalTransformation() {
  return final_transformation_;
}

bool LeastSquareICP::comparePointClouds(
    const Matrix<float, Dynamic, Dynamic>& first_cloud,
    const Matrix<float, Dynamic, Dynamic>& second_cloud,
    const float& threshold) {
  return !((first_cloud - second_cloud).array().abs() > threshold).any();
}
Matrix<int, 2, Dynamic> LeastSquareICP::getCorrespondencies() {
  return correspondencies_;
}
