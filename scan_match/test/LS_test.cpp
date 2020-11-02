#include "LeastSquareICP.hpp"
//#include "listener/EigenListener.hpp"

#include <gtest/gtest.h>

TEST(LeastSquareICP, ICP) {
  using std::cout;
  using std::endl;
  cout << "start init" << endl;
  PointCloud target = PointCloud(2, 5);
  target << 0, 1, 2, 3, 3, 0, 0, 0, 0, 1;
  Eigen::Matrix3f real_transformation;
  real_transformation << 1, 0, 0, 0, 1, 0.05, 0, 0, 1;

  PointCloud normal_target = PointCloud(2, 5);
  normal_target << 0, 0, 0, 0, -1, -1, -1, -1, -1, 0;

  Eigen::Matrix3f transformation = real_transformation.inverse();
  PointCloud source =
      LeastSquareICP::transformPointCloud(target, transformation);
  cout << "target : " << target << std::endl;
  int col = source.cols();
  Eigen::Matrix<float, 3, 1> parameter =
      LeastSquareICP::LSicp(source, target, normal_target);
  Eigen::Matrix<float, 3, 1> parameter_true;
  parameter_true << 0, 0.05, 0;
  cout << "real_transformation: \n"
       << parameter_true << "\ntransformation from icp:\n"
       << parameter;
  EXPECT_EQ(true,
            LeastSquareICP::comparePointClouds(parameter, parameter_true, 1e-4))
      << "real_transformation: \n"
      << parameter_true << "\ntransformation from icp:\n"
      << parameter;
}

TEST(LeastSquareICP, ICP_two) {
  using std::cout;
  using std::endl;
  cout << "start init" << endl;
  PointCloud target = PointCloud(2, 5);
  target << 0, 1, 2, 3, 4, 0, 0, 0, 0, 0;
  Eigen::Matrix3f real_transformation;
  real_transformation << 1, 0, 0.05, 0, 1, 0.05, 0, 0, 1;

  PointCloud normal_target = PointCloud(2, 5);
  normal_target << 0, 0, 0, 0, -1, -1, -1, -1, -1, 0;
  Eigen::Matrix3f transformation = real_transformation.inverse();
  PointCloud source =
      LeastSquareICP::transformPointCloud(target, transformation);
  cout << "target : " << target << std::endl;
  int col = source.cols();
  Eigen::Matrix<float, 3, 1> parameter =
      LeastSquareICP::LSicp(source, target, normal_target);
  Eigen::Matrix<float, 3, 1> parameter_true;
  parameter_true << 0.05, 0.05, 0;
  cout << "real_transformation: \n"
       << parameter_true << "\ntransformation from icp:\n"
       << parameter;
  EXPECT_EQ(true,
            LeastSquareICP::comparePointClouds(parameter, parameter_true, 1e-4))
      << "real_transformation: \n"
      << parameter_true << "\ntransformation from icp:\n"
      << parameter;
}
TEST(LeastSquareICP, ICP_three) {
  using std::cout;
  using std::endl;
  cout << "start init" << endl;
  PointCloud target = PointCloud(2, 6);
  target << 1, 2, 3, 4, 4, 4, 4, 4, 4, 4, 5, 6;
  Eigen::Matrix3f real_transformation;
  real_transformation << 1, 0, 4, 0, 1, 4, 0, 0, 1;

  PointCloud normal_target = PointCloud(2, 6);
  normal_target << 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0;
  Eigen::Matrix3f transformation = real_transformation.inverse();
  PointCloud source =
      LeastSquareICP::transformPointCloud(target, transformation);
  cout << "target : " << target << std::endl;
  int col = source.cols();
  Eigen::Matrix<float, 3, 1> parameter =
      LeastSquareICP::LSicp(source, target, normal_target);
  Eigen::Matrix<float, 3, 1> parameter_true;
  parameter_true << 4, 4, 0;
  cout << "real_transformation: \n"
       << parameter_true << "\ntransformation from icp:\n"
       << parameter;
  EXPECT_EQ(true,
            LeastSquareICP::comparePointClouds(parameter, parameter_true, 1e-4))
      << "real_transformation: \n"
      << parameter_true << "\ntransformation from icp:\n"
      << parameter;
}
TEST(LeastSquareICP, ICP_four) {
  using std::cout;
  using std::endl;
  cout << "start init" << endl;
  PointCloud target = PointCloud(2, 6);
  target << 1, 2, 3, 4, 4, 4, 4, 4, 4, 4, 5, 6;
  Eigen::Matrix3f real_transformation;
  real_transformation << 0.86781918, 0.49688014, 4, -0.49688014, 0.86781918, 4,
      0, 0, 1;

  PointCloud normal_target = PointCloud(2, 6);
  normal_target << 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0;
  Eigen::Matrix3f transformation = real_transformation.inverse();
  PointCloud source =
      LeastSquareICP::transformPointCloud(target, transformation);
  cout << "target : " << target << std::endl;
  int col = source.cols();
  Eigen::Matrix<float, 3, 1> parameter =
      LeastSquareICP::LSicp(source, target, normal_target);
  Eigen::Matrix<float, 3, 1> parameter_true;
  parameter_true << 4, 4, 0.52;
  cout << "real_transformation: \n"
       << parameter_true << "\ntransformation from icp:\n"
       << parameter;
  EXPECT_EQ(true,
            LeastSquareICP::comparePointClouds(parameter, parameter_true, 1e-4))
      << "real_transformation: \n"
      << parameter_true << "\ntransformation from icp:\n"
      << parameter;
}
TEST(LeastSquareICP, ICP_five) {
  using std::cout;
  using std::endl;
  cout << "start init" << endl;
  PointCloud target = PointCloud(2, 6);
  target << 1, 2, 3, 4, 4, 4, 4, 4, 4, 4, 5, 6;
  Eigen::Matrix3f real_transformation;
  real_transformation << 0.86781918, -0.49688014, 4, 0.49688014, 0.86781918, 4,
      0, 0, 1;

  PointCloud normal_target = PointCloud(2, 6);
  normal_target << 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, 0, 0;
  Eigen::Matrix3f transformation = real_transformation.inverse();
  PointCloud source =
      LeastSquareICP::transformPointCloud(target, transformation);
  cout << "target : " << target << std::endl;

  Eigen::Matrix<float, 3, 1> parameter =
      LeastSquareICP::LSicp(source, target, normal_target);
  Eigen::Matrix<float, 3, 1> parameter_true;
  parameter_true << 4, 4, -0.52;
  cout << "real_transformation: \n"
       << parameter_true << "\ntransformation from icp:\n"
       << parameter;
  EXPECT_EQ(true,
            LeastSquareICP::comparePointClouds(parameter, parameter_true, 1e-4))
      << "real_transformation: \n"
      << parameter_true << "\ntransformation from icp:\n"
      << parameter;
}
TEST(LeastSquareICP, align) {
  using std::cout;
  using std::endl;
  cout << "start init" << endl;
  PointCloud source = PointCloud(2, 6);
  source << 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 4, 4;
  Eigen::Matrix3f real_transformation;
  float angle = 5.0f/180.0f*M_PI;
  std::cout<<"angle: "<<angle<<std::endl;
  real_transformation << cos(angle), sin(angle), 0.1, -sin(angle), cos(angle),
      0.2, 0, 0, 1;

  PointCloud normal_target = PointCloud(2, 6);
  PointCloud target =
      LeastSquareICP::transformPointCloud(source, real_transformation);
  cout << "source : " << source << std::endl;
  cout << "target : " << target << std::endl;

  LeastSquareICP LSICP = LeastSquareICP(1e3, 1e-9, 1e-8, 1e4);
  LSICP.setInputSource(source);
  LSICP.setInputTarget(target);
  LSICP.align();
  Eigen::Matrix3f actual_transformation = LSICP.getFinalTransformation();

  EXPECT_EQ(true, LeastSquareICP::comparePointClouds(actual_transformation,
                                                     real_transformation, 1e-5))
      << "real_transformation: \n"
      << real_transformation << "\ntransformation from icp:\n"
      << actual_transformation;
}
TEST(LeastSquareICP, align_translation) {
  using std::cout;
  using std::endl;
  cout << "start init" << endl;
  PointCloud source = PointCloud(2, 6);
  source << 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 4, 4;
  Eigen::Matrix3f real_transformation;
  float angle = 0.0f/180.0f*M_PI;
  std::cout<<"angle: "<<angle<<std::endl;
  real_transformation << cos(angle), sin(angle), 0.1, -sin(angle), cos(angle),
      0.2, 0, 0, 1;

  PointCloud normal_target = PointCloud(2, 6);
  PointCloud target =
      LeastSquareICP::transformPointCloud(source, real_transformation);
  cout << "source : " << source << std::endl;
  cout << "target : " << target << std::endl;

  LeastSquareICP LSICP = LeastSquareICP(1e3, 1e-9, 1e-8, 1e4);
  LSICP.setInputSource(source);
  LSICP.setInputTarget(target);
  LSICP.align();
  Eigen::Matrix3f actual_transformation = LSICP.getFinalTransformation();

  EXPECT_EQ(true, LeastSquareICP::comparePointClouds(actual_transformation,
                                                     real_transformation, 1e-5))
      << "real_transformation: \n"
      << real_transformation << "\ntransformation from icp:\n"
      << actual_transformation;
}
TEST(LeastSquareICP, align_rotation) {
  using std::cout;
  using std::endl;
  cout << "start init" << endl;
  PointCloud source = PointCloud(2, 6);
  source << 1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 4, 4;
  Eigen::Matrix3f real_transformation;
  float angle = 5.0f/180.0f*M_PI;
  std::cout<<"angle: "<<angle<<std::endl;
  real_transformation << cos(angle), sin(angle), 0.0, -sin(angle), cos(angle),
      0.0, 0, 0, 1;

  PointCloud normal_target = PointCloud(2, 6);
  PointCloud target =
      LeastSquareICP::transformPointCloud(source, real_transformation);
  cout << "source : " << source << std::endl;
  cout << "target : " << target << std::endl;

  LeastSquareICP LSICP = LeastSquareICP(1e3, 1e-9, 1e-8, 1e4);
  LSICP.setInputSource(source);
  LSICP.setInputTarget(target);
  LSICP.align();
  Eigen::Matrix3f actual_transformation = LSICP.getFinalTransformation();

  EXPECT_EQ(true, LeastSquareICP::comparePointClouds(actual_transformation,
                                                     real_transformation, 1e-5))
      << "real_transformation: \n"
      << real_transformation << "\ntransformation from icp:\n"
      << actual_transformation;
}

TEST(LeastSquareICP, normal) {
  Eigen::Matrix<float, 2, 1> p, n, source;
  p << 1, 1;
  n << 0, -1;
  source << 0, 0;

  float dist = LeastSquareICP::dist2plane(p, n, source);

  EXPECT_NEAR(dist, 1, 1e-5) << "expected: " << 1 << " actual: " << dist
                             << std::endl;
}

TEST(NormalEstimation, est_normal_diagonal) {
  PointCloud source = PointCloud(2, 6);
  source << 1, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 7;
  PointCloud real_normals = PointCloud(2, 6);
  real_normals << 1 / sqrt(2), 1 / sqrt(2), 1 / sqrt(2), 1 / sqrt(2),
      1 / sqrt(2), 1 / sqrt(2), -1 / sqrt(2), -1 / sqrt(2), -1 / sqrt(2),
      -1 / sqrt(2), -1 / sqrt(2), -1 / sqrt(2);

  NormalEstimation::Normal N(source.cols());
  N.GetNormal(source);
  for (int i = 0; i < N.length; ++i) {
    Eigen::Matrix<float, 2, 1> actual_normal;
    actual_normal << N.normal[i].nx, N.normal[i].ny;

    EXPECT_EQ(true, LeastSquareICP::comparePointClouds(
                        actual_normal, real_normals.col(i), 1e-5) &&
                        !std::isnan(actual_normal.norm()))
        << "i: " << i << " real_normal: \n"
        << real_normals.col(i) << "\nactual_normal:\n"
        << actual_normal << std::endl;
  }
}

TEST(NormalEstimation, est_normal_horizontal) {
  PointCloud source = PointCloud(2, 6);
  source << 1, 2, 3, 4, 5, 6, 2, 2, 2, 2, 2, 2;
  PointCloud real_normals = PointCloud(2, 6);
  real_normals << 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1;

  NormalEstimation::Normal N(source.cols());
  N.GetNormal(source);
  for (int i = 0; i < N.length; ++i) {
    Eigen::Matrix<float, 2, 1> actual_normal;
    actual_normal << N.normal[i].nx, N.normal[i].ny;

    EXPECT_EQ(true, LeastSquareICP::comparePointClouds(
                        actual_normal, real_normals.col(i), 1e-5) &&
                        !std::isnan(actual_normal.norm()))
        << "i: " << i << " real_normal: \n"
        << real_normals.col(i) << "\nactual_normal:\n"
        << actual_normal << std::endl;
  }
}
TEST(NormalEstimation, est_normal_vertical) {
  PointCloud source = PointCloud(2, 6);
  source << 2, 2, 2, 2, 2, 2, 1, 2, 3, 4, 5, 6;
  PointCloud real_normals = PointCloud(2, 6);
  real_normals << -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0;

  NormalEstimation::Normal N(source.cols());
  N.GetNormal(source);
  for (int i = 0; i < N.length; ++i) {
    Eigen::Matrix<float, 2, 1> actual_normal;
    actual_normal << N.normal[i].nx, N.normal[i].ny;
    EXPECT_EQ(true, LeastSquareICP::comparePointClouds(
                        actual_normal, real_normals.col(i), 1e-5) &&
                        !std::isnan(actual_normal.norm()))
        << "i: " << i << " real_normal: \n"
        << real_normals.col(i) << "\nactual_normal:\n"
        << actual_normal << std::endl;
  }
}
/*
TEST(LeastSquareICP, Correspondencies) {
  PointCloud target(2, 1);
  target << 1, 1;
  PointCloud normal(2, 1);
  normal << 0, -1;
  PointCloud source(2, 2);
  source << 2, 3, 2, 1;
  LeastSquareICP LSICP = LeastSquareICP(1e3, 1e-9, 1e-8, 1e4);
  LSICP.setInputTarget(target);
  LSICP.setInputSource(source);

  NormalEstimation::Normal N(1);
  N.normal[0].x = 1;
  N.normal[0].y = 1;
  N.normal[0].nx = 0;
  N.normal[0].ny = -1;

  PointCloud corresponding_points_source, corresponding_points_target,
      target_normal;

  LSICP.Correspondencies(N, corresponding_points_source,
                         corresponding_points_target, target_normal);
  Matrix<int, 2, Dynamic> correspondencies = LSICP.getCorrespondencies();
  EXPECT_EQ(1, correspondencies.cols())
      << "cols() should be 1, is: " << correspondencies.cols() << std::endl;
  EXPECT_EQ(0, correspondencies(1, 0))
      << "target indice should be 0, is: " << correspondencies(1, 0)
      << std::endl
      << "correspondencies:\n"
      << correspondencies << std::endl
      << "corresponding_points_target:\n"
      << corresponding_points_target << std::endl
      << "corresponding_points_source:\n"
      << corresponding_points_source << std::endl;
  EXPECT_EQ(1, correspondencies(0, 0))
      << "source indice should be 1, is: " << correspondencies(0, 0)
      << std::endl
      << "corresponding_points_target:\n"
      << corresponding_points_target << std::endl
      << "corresponding_points_source:\n"
      << corresponding_points_source << std::endl;
}
*/