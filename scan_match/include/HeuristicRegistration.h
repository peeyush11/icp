#pragma once
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "EigenScanRegistration.hpp"

class HeuristicRegistration : public EigenScanRegistration {
  typedef Matrix<float, 2, Dynamic> PointCloud;
  typedef Matrix<float, 2, 1> Point;

 public:
  HeuristicRegistration();
  HeuristicRegistration(const int& maximum_iterations,
                        const float& euclidean_fitness_epsilon,
                        const float& transformation_epsilon,
                        const float& max_correspondence_distance);
  void setNumberNeighbors(const int& number_neighbors) override;
  ~HeuristicRegistration();

 protected:
  Eigen::Matrix2f findCorrespondencies() override;
  int number_neighbors_ = 100;
};
