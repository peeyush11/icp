#include "RosUtils.h"

Eigen::Matrix4f TF2Matrix4(const tf::Transform& transformation) {
  Eigen::Matrix4f matrix;
  matrix << transformation.getBasis()[0][0], transformation.getBasis()[0][1],
      transformation.getBasis()[0][2], transformation.getOrigin().x(),
      transformation.getBasis()[1][0], transformation.getBasis()[1][1],
      transformation.getBasis()[1][2], transformation.getOrigin().y(),
      transformation.getBasis()[2][0], transformation.getBasis()[2][1],
      transformation.getBasis()[2][2], transformation.getOrigin().z(), 0, 0, 0,
      1;
  return matrix;
}
tf::Transform Matrix4ToTF(const Eigen::Matrix4f& matrix) {
  tf::Matrix3x3 rotation_matrix;
  rotation_matrix.setValue(
      static_cast<double>(matrix(0, 0)), static_cast<double>(matrix(0, 1)),
      static_cast<double>(matrix(0, 2)), static_cast<double>(matrix(1, 0)),
      static_cast<double>(matrix(1, 1)), static_cast<double>(matrix(1, 2)),
      static_cast<double>(matrix(2, 0)), static_cast<double>(matrix(2, 1)),
      static_cast<double>(matrix(2, 2)));
  tf::Quaternion rotation_tf;
  rotation_matrix.getRotation(rotation_tf);

  tf::Vector3 translation_tf(matrix(0, 3), matrix(1, 3), matrix(2, 3));
  tf::Transform transformation(rotation_tf, translation_tf);
  return transformation;
}

tf::Transform Matrix3ToTF(const Eigen::Matrix3f& matrix) {
  Eigen::Matrix4f matrix4 = Eigen::Matrix4f::Identity();
  matrix4.block<2, 2>(0, 0) = matrix.block<2, 2>(0, 0);
  matrix4.block<2, 1>(0, 3) = matrix.block<2, 1>(0, 2);

  return Matrix4ToTF(matrix4);
}