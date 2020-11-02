#include "HeuristicRegistration.h"
#include "strategy/HeuristicStrategy.h"

#include <gtest/gtest.h>
using std::cout;
using std::endl;
typedef Matrix<float, 2, Dynamic> PointCloud;

bool comparePointClouds(const Matrix<float, Dynamic, Dynamic>& first_cloud,
                        const Matrix<float, Dynamic, Dynamic>& second_cloud,
                        const float& threshold) {
  return !((first_cloud - second_cloud).array().abs() > threshold).any();
}

/*class HeuristicRegistration_test : public HeuristicRegistration {
 public:
  HeuristicRegistration_test();
  Eigen::Matrix2f findCorrespondencies() override {
    return HeuristicRegistration::findCorrespondencies();
  }
};*/
class HeuristicStrategy_test : public HeuristicStrategy {
 public:
  HeuristicStrategy_test() { HeuristicStrategy(); }
  PointCloud makeCircular(const PointCloud& cloud) {
    return HeuristicStrategy::makeCircular(cloud);
  }
  void setNumberNeighbors(int nn) { number_neighbors_ = nn; }
};
TEST(HeuristicStrategy, makeCircular) {
  HeuristicStrategy_test heuristic_strategy;
  heuristic_strategy.setNumberNeighbors(2);
  PointCloud cloud(2, 4);
  cloud << 0, 1, 2, 3, 
          0, 1, 2, 3;
  PointCloud cloud_target(2, 8);
  cloud_target << 2, 3, 0, 1, 2, 3, 0, 1,
                   2, 3, 0, 1, 2, 3, 0, 1;

  PointCloud cloud_test = heuristic_strategy.makeCircular(cloud);
  //cout << cloud_target << endl << endl << cloud_test << endl;
  EXPECT_EQ(true, comparePointClouds(cloud_target, cloud_test, 1e-5)) << cloud_target << endl << endl << cloud_test << endl;
}
TEST(HeuristicRegistration, ICP) {
  cout << "start init" << endl;
  PointCloud target = PointCloud(2, 6);
  /*target << 0, 1, 2, 3, 4, 5,
          0, 1, 2, 3, 4 ,5;
  */
  target << 0, 5, 10, 25, 20,30, 
            0, 7, 15, 16, 20,30;

  // target = PointCloud(2, 2);
  // target << 1,1,2,3;
  Eigen::Matrix3f real_transformation;
  float angle = 0.3;
  real_transformation << cos(angle), -sin(angle), 0.1, sin(angle), cos(angle),
      0.1, 0, 0, 1;
  // real_transformation << 0.9848, -0.1736, 0.05, 0.1736, 0.9848, 0.08, 0, 0,
  // 1;
  // real_transformation << 1, 0, 0.05, 0, 1, 0.01, 0, 0, 1;
  PointCloud source =
      EigenScanRegistration::transformPointCloud(target, real_transformation);

  HeuristicRegistration ICP(100, 1e-8, 1e-12, 1e5);
  ICP.setNumberNeighbors(2);

  HeuristicStrategy_test heuristic_strategy;
  heuristic_strategy.setNumberNeighbors(2);

  ICP.setInputSource(source);
  ICP.setInputTarget(heuristic_strategy.makeCircular(target));

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
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
