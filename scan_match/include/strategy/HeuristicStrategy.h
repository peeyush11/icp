#pragma once
#include "HeuristicRegistration.h"
#include "RosUtils.h"
#include "strategy/NearestNeighbor.h"
/*
inherit from NearestNeighbor:
  - same approach
  - uses the heuristic that the pointcloud is sorted by angle
  - restrict the searching to the neighbors
*/
class HeuristicStrategy : public NearestNeighbor {
 public:
  typedef Matrix<float, 2, Dynamic> PointCloud;

  HeuristicStrategy();
  HeuristicStrategy(const int& maximum_iterations,
                    const float& euclidean_fitness_epsilon,
                    const float& transformation_epsilon,
                    const float& max_correspondence_distance,
                    const float& max_correspondence_angle);
  /*
   Computes the odometry (position & orientation) from 2 scans with
   respetct to the previous one, and returns the absolute odometry
  */
  tf::Transform computeOdometry(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan) override;

 protected:
  PointCloud transformScans2Cloud(
      const sensor_msgs::LaserScan::ConstPtr& front_scan,
      const sensor_msgs::LaserScan::ConstPtr& back_scan);

  void extractIndices(const sensor_msgs::LaserScan::ConstPtr& front_scan,
                      const sensor_msgs::LaserScan::ConstPtr& back_scan);

  PointCloud makeCircular(const PointCloud& cloud);
  int start_ind_;  // defines the part of the scan that will be used
  int end_ind_;    // defines the part of the scan that will be used
  // number of how many neighbors will be taken into account
  int number_neighbors_ = 100;
  // can be converted into number_neighbors with the angle increment
  float max_correspondence_angle_;
};
