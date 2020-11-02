#include "HeuristicRegistration.h"

typedef Matrix<float, 2, Dynamic> PointCloud;
typedef Matrix<float, 2, 1> Point;

using std::cout;
using std::endl;

HeuristicRegistration::HeuristicRegistration() {
  final_transformation_ = Eigen::Matrix3f::Identity();
}
HeuristicRegistration::~HeuristicRegistration() {}
HeuristicRegistration::HeuristicRegistration(
    const int& maximum_iterations, const float& euclidean_fitness_epsilon,
    const float& transformation_epsilon,
    const float& max_correspondence_distance)
    : EigenScanRegistration::EigenScanRegistration(
          maximum_iterations, euclidean_fitness_epsilon, transformation_epsilon,
          max_correspondence_distance) {
  final_transformation_ = Eigen::Matrix3f::Identity();
}

Eigen::Matrix2f HeuristicRegistration::findCorrespondencies() {
  // need to fill source_mean_ and target_mean_ and correspondencies_
  int cols = source_.cols();
  PointCloud corresponding_points_source(2, cols);
  PointCloud corresponding_points_target(2, cols);
  correspondencies_.resize(2, cols);
  int number_correspondencies = 0;

  int vector_length = 2 * number_neighbors_ + 1;
  Eigen::Matrix<float, 1, Dynamic> squared_distances(1, vector_length);
  Eigen::Matrix<float, 1, Dynamic>::Index min_index;
  // Loop over all source Points
  for (int col = 0; col < cols; ++col) {
    //cout<<"start_ind: "<<col<<" vector_length: "<<vector_length<<" source_cols:"<<cols<<" target_cols:"<<target_.cols()<<" number_neighbors_:"<<number_neighbors_<<endl;
    int start_ind = col;
    squared_distances =
        (target_.block(0, start_ind, 2, vector_length).colwise() -
         source_.col(col))
            .colwise()
            .squaredNorm();
    if (squared_distances.minCoeff(&min_index) <
        max_correspondence_distance_ * max_correspondence_distance_) {
      //cout<<"min_ind:"<<min_index<<endl;
      int index = min_index + col;
      corresponding_points_source.col(number_correspondencies) =
          source_.col(col);
      corresponding_points_target.col(number_correspondencies) =
          target_.col(index);
      correspondencies_(0, number_correspondencies) = index;
      correspondencies_(1, number_correspondencies) = col;

      number_correspondencies++;
    }
  }
  corresponding_points_source.conservativeResize(2, number_correspondencies);
  corresponding_points_target.conservativeResize(2, number_correspondencies);
  correspondencies_.conservativeResize(2, number_correspondencies);
  //cout<<"correspondencies_:"<<correspondencies_<<endl;
  source_mean_ = corresponding_points_source.rowwise().mean();
  corresponding_points_source.colwise() -= source_mean_;

  target_mean_ = corresponding_points_target.rowwise().mean();
  corresponding_points_target.colwise() -= target_mean_;

  Eigen::Matrix2f cross_covariance_matrix =
      corresponding_points_target * corresponding_points_source.transpose();

  return cross_covariance_matrix;
}

void HeuristicRegistration::setNumberNeighbors(const int& number_neighbors) {
  number_neighbors_ = number_neighbors;
}
