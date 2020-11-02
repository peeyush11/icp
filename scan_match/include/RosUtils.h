#pragma once
#include <math.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

Eigen::Matrix4f TF2Matrix4(const tf::Transform& transformation);
tf::Transform Matrix4ToTF(const Eigen::Matrix4f& matrix);
tf::Transform Matrix3ToTF(const Eigen::Matrix3f& matrix);
