#pragma once
#include <NormalEstimation.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
using namespace Eigen;
typedef Matrix<float, 2, Dynamic> PointCloud;
typedef Matrix<float, 2, 1> Point;

#define PI 3.14159265;

class LeastSquareICP {
 public:
  LeastSquareICP();
  LeastSquareICP(const int& maximum_iterations,
                 const float& euclidean_fitness_epsilon,
                 const float& transformation_epsilon,
                 const float& max_correspondence_distance);
  ~LeastSquareICP();

  void align();
  void setInputSource(PointCloud source);
  void setInputTarget(const PointCloud& target);
  void setMaxCorrespondenceDistance(const float& max_correspondence_distance);
  void setMaximumIterations(const int& maximum_iterations);
  void setTransformationEpsilon(const float& transformation_epsilon);
  void setEuclideanFitnessEpsilon(const float& euclidean_fitness_epsilon);
  bool hasConverged();
  float getFitnessScore();
  Eigen::Matrix3f getFinalTransformation();

  static PointCloud transformPointCloud(const PointCloud& cloud,
                                        const Eigen::Matrix3f& transformation);
  static bool comparePointClouds(
      const Matrix<float, Dynamic, Dynamic>& first_cloud,
      const Matrix<float, Dynamic, Dynamic>& second_cloud,
      const float& threshold);
  static Eigen::Matrix<float, 3, 1> LSicp(PointCloud& source,
                                          PointCloud& target,
                                          PointCloud& normal);
  static PointCloud transformPointCloud(const PointCloud& cloud,
                                        const Eigen::Matrix4f& transformation);
  static float dist2plane(const Eigen::Matrix<float, 2, 1>& p,
                          const Eigen::Matrix<float, 2, 1>& n,
                          const Eigen::Matrix<float, 2, 1>& source);
  void Correspondencies(NormalEstimation::Normal N,
                        PointCloud& corresponding_points_source,
                        PointCloud& corresponding_points_target,
                        PointCloud& target_normal);
  Matrix<int, 2, Dynamic> getCorrespondencies();

 private:
  PointCloud source_;  // moving
  Point source_mean_;
  PointCloud target_;  // stable
  Point target_mean_;
  PointCloud source_transformed_;             //
  Matrix<int, 2, Dynamic> correspondencies_;  // first row: source indices,
                                              // second row: target indices
  // pcl::KdTreeFLANN<Point> kdtree_;
  int maximum_iterations_ = 10;
  float euclidean_fitness_epsilon_ = 1e-4;
  float transformation_epsilon_ = 1e-8;
  float max_correspondence_distance_ = 0.1;
  bool converged_ = false;
  float fitness_score_ = -1;
  Matrix3f final_transformation_;

  // Eigen::Matrix2f Correspondencies(NormalEstimation::Normal &N);
  // void Correspondencies(NormalEstimation::Normal &N);
  Eigen::Matrix2f findCorrespondencies();
  Eigen::Matrix3f computeTransformation(
      const Eigen::Matrix2f& cross_covariance_matrix);
  float computeError(PointCloud& normal);
};
