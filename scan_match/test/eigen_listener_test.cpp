#include "EigenScanRegistration.hpp"
#include "listener/EigenListener.hpp"

#include <gtest/gtest.h>
TEST(EigenScanRegistration, comparePointClouds_true) {
  PointCloud first_cloud(2, 3);
  first_cloud << 1, 2, 3, 1, 2, 3;
  PointCloud second_cloud(2, 3);
  second_cloud << 1.1, 2.1, 2.9, 1.1, 1.9, 2.9;
  EXPECT_EQ(true, EigenScanRegistration::comparePointClouds(first_cloud,
                                                            second_cloud, 2e-1))
      << "first_cloud:\n"
      << first_cloud << "\nsecond_cloud:\n"
      << second_cloud << "\ndiff\n"
      << (first_cloud - second_cloud);
}

TEST(EigenScanRegistration, comparePointClouds_false) {
  PointCloud first_cloud(2, 3);
  first_cloud << 1, 2, 3, 1, 2, 3;
  PointCloud second_cloud(2, 3);
  second_cloud << 1.3, 2.1, 2.9, 1.1, 1.9, 2.9;
  EXPECT_EQ(false, EigenScanRegistration::comparePointClouds(
                       first_cloud, second_cloud, 2e-1))
      << "first_cloud:\n"
      << first_cloud << "\nsecond_cloud:\n"
      << second_cloud << "\ndiff\n"
      << (first_cloud - second_cloud);
}
TEST(EigenListener, tfToEigen) {
  float angle = 0.7;
  tf::Matrix3x3 tf_rotation(cos(angle), -sin(angle), 0, sin(angle), cos(angle),
                            0, 0, 0, 1);
  tf::Vector3 tf_translation(1, 2, 0);
  tf::Transform transform(tf_rotation, tf_translation);

  Matrix3f eigen_transformation;
  eigen_transformation << cos(angle), -sin(angle), 1, sin(angle), cos(angle), 2,
      0, 0, 1;

  Matrix3f eigen_transformation_test = EigenListener::tfToEigen(transform);
  EXPECT_EQ(true, EigenScanRegistration::comparePointClouds(
                      eigen_transformation, eigen_transformation_test, 1e-5))
      << "eigen_transformation:\n"
      << eigen_transformation << "\neigen_transformation_test:\n"
      << eigen_transformation_test << "\ndiff\n"
      << (eigen_transformation - eigen_transformation_test)
      << "\ntf_eigentranslation:\n"
      << tf_translation[0] << " , " << tf_translation[1];
}

TEST(EigenListener, scan2cloud) {
  sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan());
  scan->angle_min = 0;
  scan->angle_increment = 0.1;
  std::vector<float> ranges({0, 1, 2, 3, 4, 5, 6, 7});
  scan->ranges = ranges;
  // sensor_msgs::LaserScan::Ptr scan_ptr=&scan;
  PointCloud test_cloud = EigenListener::scan2Cloud(scan);
  PointCloud true_cloud(2, 8);
  true_cloud << 0, 0.9950, 1.9601, 2.8660, 3.6842, 4.3879, 4.9520, 5.3539, 0,
      0.0998, 0.3973, 0.8866, 1.5577, 2.3971, 3.3879, 4.5095;

  EXPECT_EQ(true, EigenScanRegistration::comparePointClouds(test_cloud,
                                                            true_cloud, 2e-4))
      << "true_cloud:\n"
      << true_cloud << "\ntest_cloud:\n"
      << test_cloud;
}
TEST(EigenScanRegistration, transformPointCloud) {
  PointCloud cloud(2, 3);
  cloud << 0, 1, 2, 0, 1, 2;
  Matrix3f transformation;
  transformation << 1, 0, 1, 0, 1, 2, 0, 0, 1;
  PointCloud real_cloud(2, 3);
  real_cloud << 1, 2, 3, 2, 3, 4;
  PointCloud cloud_transformed =
      EigenScanRegistration::transformPointCloud(cloud, transformation);
  EXPECT_EQ(true, EigenScanRegistration::comparePointClouds(
                      real_cloud, cloud_transformed, 1e-6))
      << "real cloud\n"
      << real_cloud << "\ncloud_transformed\n"
      << cloud_transformed << "\ncloud\n"
      << cloud;
}
TEST(EigenScanRegistration, ICP) {
  using std::cout;
  using std::endl;
  cout << "start init" << endl;
  PointCloud target = PointCloud(2, 5);
  target << 0, 5, 10, 25, 20, 0, 7, 15, 16, 20;
  // target = PointCloud(2, 2);
  // target << 1,1,2,3;
  Matrix3f real_transformation;
  float angle = 0.3;
  real_transformation << cos(angle), -sin(angle), 0.5, sin(angle), cos(angle),
      0.3, 0, 0, 1;
  // real_transformation << 0.9848, -0.1736, 0.05, 0.1736, 0.9848, 0.08, 0, 0,
  // 1;
  // real_transformation << 1, 0, 0.05, 0, 1, 0.01, 0, 0, 1;
  PointCloud source =
      EigenScanRegistration::transformPointCloud(target, real_transformation);

  EigenScanRegistration ICP(10, 1e-4, 1e-8, 1e5);

  ICP.setInputSource(source);
  ICP.setInputTarget(target);

  ICP.align();

  PointCloud source_test =
      EigenScanRegistration::transformPointCloud(target, real_transformation);

  EXPECT_EQ(true, EigenScanRegistration::comparePointClouds(
                      real_transformation.inverse(),
                      ICP.getFinalTransformation(), 1e-4))
      << "real_transformation_inverse: \n"
      << real_transformation.inverse() << "\ntransformation from icp:\n"
      << ICP.getFinalTransformation();
  EXPECT_EQ(true, EigenScanRegistration::comparePointClouds(source, source_test,
                                                            1e-8))
      << "source_after_icp: \n<<" << source << "\n source before icp: \n"
      << source_test << endl;
  EXPECT_EQ(false, &source == &source_test)
      << "source and test have the same address";
  EXPECT_EQ(true, EigenScanRegistration::comparePointClouds(
                      target, ICP.getTransformedCloud(), 1e-5))
      << "target:" << endl
      << target << endl
      << "source after icp: " << endl
      << ICP.getTransformedCloud() << endl;
}

TEST(EigenScanRegistration, ICP_path) {
  using std::cout;
  using std::endl;
  cout << "start init" << endl;

  PointCloud target = PointCloud(2, 5);
  PointCloud source = PointCloud(2, 5);
  std::vector<Matrix3f> real_transformations(3);
  float angle = 0.2;
  real_transformations[0] << cos(angle), -sin(angle), 0.5, sin(angle),
      cos(angle), 0.3, 0, 0, 1;
  real_transformations[1] << cos(angle / 2), -sin(angle / 2), 0.2,
      sin(angle / 2), cos(angle / 2), 0.3, 0, 0, 1;
  real_transformations[2] << cos(angle + 0.1), -sin(angle + 0.1), 0.5,
      sin(angle + 0.1), cos(angle + 0.1), 0, 0, 0, 1;
  Matrix3f real_odometry = Matrix3f::Identity();

  EigenScanRegistration ICP(10, 1e-4, 1e-8, 1e5);

  source << 0, 5, 10, 25, 20, 0, 7, 15, 16, 20;
  for (const Matrix3f& transformation : real_transformations) {
    PointCloud target =
        EigenScanRegistration::transformPointCloud(source, transformation);
    real_odometry = transformation * real_odometry;

    ICP.setInputSource(source);
    ICP.setInputTarget(target);
    ICP.align();

    EXPECT_EQ(true, EigenScanRegistration::comparePointClouds(
                        real_odometry, ICP.getFinalTransformation(), 1e-5))
        << "real_transformation: \n"
        << real_odometry << "\ntransformation from icp:\n"
        << ICP.getFinalTransformation();
    EXPECT_EQ(true, EigenScanRegistration::comparePointClouds(
                        target, ICP.getTransformedCloud(), 1e-5))
        << "target:" << endl
        << target << endl
        << "source after icp: " << endl
        << ICP.getTransformedCloud() << endl;
  }
  source = target;
  EXPECT_EQ(false, &source == &target)
      << "source and target have the same address";
}
/*int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
*/