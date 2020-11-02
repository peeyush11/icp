#pragma once
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <cmath>
using namespace Eigen;

class EigenScanRegistration {
  typedef Matrix<float, 2, Dynamic> PointCloud;
  typedef Matrix<float, 2, 1> Point;

 public:
  struct ScanProperties {
    float angle_min;
    float angle_max;
    float angle_increment;
  };
  //constructors
  EigenScanRegistration();
  EigenScanRegistration(const int& maximum_iterations,
                        const float& euclidean_fitness_epsilon,
                        const float& transformation_epsilon,
                        const float& max_correspondence_distance);
  ~EigenScanRegistration();

  // aligns the source cloud to the target cloud 
  void align();

  // setter Functions
  void setInputSource(PointCloud source);
  void setInputTarget(const PointCloud& target);
  void setMaxCorrespondenceDistance(const float& max_correspondence_distance);
  void setMaximumIterations(const int& maximum_iterations);
  void setTransformationEpsilon(const float& transformation_epsilon);
  void setEuclideanFitnessEpsilon(const float& euclidean_fitness_epsilon);
  void setMaxCorrespondenceAngle(const float& max_correspondence_angle);
  void resetTransformation();

  //returns if the last icp converged
  Eigen::Matrix3f getFinalTransformation();
  PointCloud getTransformedCloud();

  //Transforming the laserscan (Rotation + translation)
  static PointCloud transformPointCloud(const PointCloud& cloud,
                                        const Eigen::Matrix3f& transformation);
  static PointCloud transformPointCloud(const PointCloud& cloud,
                                        const Eigen::Matrix4f& transformation);

  //Elementwise comparision if 2 matrices are the same
  static bool comparePointClouds(
      const Matrix<float, Dynamic, Dynamic>& first_cloud,
      const Matrix<float, Dynamic, Dynamic>& second_cloud,
      const float& threshold);

  //just for the heuristic approach
  virtual void setNumberNeighbors(const int& number_neighbours);

 protected:
  PointCloud source_;  // moving
  Point source_mean_;
  PointCloud target_;  // stable
  Point target_mean_;
  PointCloud source_transformed_;             // after alignement
  Matrix<int, 2, Dynamic> correspondencies_;  // first row: target indices,
                                              // second row: source indices
  // pcl::KdTreeFLANN<Point> kdtree_;
  int maximum_iterations_ = 10;
  float euclidean_fitness_epsilon_ = 1e-4; // mean error threshold
  float transformation_epsilon_ = 1e-8; // mean error change threshold
  // wont consider correspondencies further this range
  float max_correspondence_distance_ = 0.1; 
  // wont consider correspondencies further this angle 
  float max_correspondence_angle_ = 0;
  float rotation_angle_ = 0;
  Matrix3f final_transformation_;

  // searching for correspondencies and computing the cross covariance matrix
  virtual Eigen::Matrix2f findCorrespondencies();
  
  Eigen::Matrix3f computeTransformation(
      const Eigen::Matrix2f& cross_covariance_matrix);
  float computeError();

};
