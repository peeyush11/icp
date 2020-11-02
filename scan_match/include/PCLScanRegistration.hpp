#pragma once
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>

/*!
 * Class containing the algorithmic part of the package.
 */
class PCLScanRegistration {
 public:
  typedef Eigen::Matrix<float, 4, 4> Matrix4f;
  typedef pcl::PointCloud<pcl::PointXYZ> pcl_PointCloud;

  /*!
   * Constructor.
   */
  PCLScanRegistration();
  PCLScanRegistration(const int& maximum_iterations,
                      const float& euclidean_fitness_epsilon,
                      const float& transformation_epsilon,
                      const float& max_correspondence_distance);

  /*!
   * Destructor.
   */
  virtual ~PCLScanRegistration();

  /*!
   * Add new measurement data.
   * @param data the new data.
   */
  Matrix4f computeTransformation(pcl_PointCloud::Ptr& new_cloud);
  Matrix4f calibrate(const pcl_PointCloud::Ptr& first_cloud,
                     const pcl_PointCloud::Ptr& second_cloud);
  Matrix4f final_calibration();
  void initTransformation(const Matrix4f& initial_transformation);
  void resetTransformation();
  pcl_PointCloud::Ptr getTransformedCloud();

 private:
  pcl_PointCloud::Ptr first_cloud_;
  pcl_PointCloud::Ptr second_cloud_;
  pcl_PointCloud::Ptr cloud_transformed_;

  Matrix4f accumulated_transformation_;
  int transformation_counter_ = 0;

  // ICP Parameter
  float max_correspondence_distance_ = 0.1;
  int maximum_iterations_ = 10;
  float transformation_epsilon_ = 1e-8;
  float euclidean_fitness_epsilon_ = 1e-2;
};
