#include "NormalEstimation.h"
#include <iostream>

namespace NormalEstimation {

Normal::Normal(int size) {
  sample_normal.reserve(size / 3);
  normal = std::vector<data>(size);
  ;
  length = size;
  count = 0;
}

Normal::Normal() {}

Normal::~Normal() {}

void Normal::SampleNormal() {
  Eigen::Vector2f V1;
  Eigen::Vector2f V2;
  int j = 1;
  int i = 0;

  while (i < length) {
    V1 << normal[i].nx, normal[i].ny;
    sample_normal.emplace_back(i);
    count++;

    while (j < i + 20 && j < length) {
      V2 << normal[j].nx, normal[j].ny;
      float d = V1.dot(V2);

      if (d < 0.9960)  // 4.4 degrees - 0.9970
      {
        break;
      } else {
        j++;
      }

      i = j;
    }
  }
}
void Normal::GetNormal(const Eigen::Matrix<float, 2, Dynamic> &SourceCloud) {
  int s = SourceCloud.cols();
  length = 0;
  Eigen::Vector2f scanner;
  scanner << 0, 0;

  for (int i = 0; i < SourceCloud.cols(); ++i) {
    Eigen::MatrixXf temp(4, 1);
    Eigen::MatrixXf points = NearestPoints(SourceCloud, i);

    int p = points.cols();
    Eigen::MatrixXf L_est(p, 1);
    Eigen::MatrixXf x_est(2, 1);
    if (points.cols() < 2) {
      continue;
    } else {
      FitLine(points, L_est, x_est);

      temp(0, 0) = points(0, 0);
      temp(1, 0) = points(1, 0);

      Eigen::Vector2f normal_vector(-x_est(0, 0), 1);
      normal_vector = normal_vector / normal_vector.norm();  // unit vector

      if (std::isnan(x_est(0, 0)) ||
          Eigen::isnan(normal_vector.array()).any() ||
          (normal_vector.array() < 0.7).all() ||
          (normal_vector.array() > 1).any() || std::abs(x_est(0, 0)) > 1e2) {
        normal_vector << 1, 0;
      }

      Eigen::Vector2f scanner_vector = temp.block<2, 1>(0, 0) - scanner;
      if (normal_vector.dot(scanner_vector) > 0) {
        normal_vector = normal_vector * -1;
      }

      temp(2, 0) = normal_vector(0, 0);
      temp(3, 0) = normal_vector(1, 0);

      normal[length].x = temp(0, 0);
      normal[length].y = temp(1, 0);
      normal[length].nx = temp(2, 0);
      normal[length].ny = temp(3, 0);
      length++;
    }
  }
  // SampleNormal();
}

void Normal::FitLine(const Eigen::Matrix<float, 2, Dynamic> &points,
                     Eigen::MatrixXf &L_est, Eigen::MatrixXf &x_est) {
  int cols = points.cols();

  Eigen::MatrixXf A(cols, 2);

  L_est = GetInitialObservation(points, cols);

  A = Jacobian(points, cols);
  x_est = (A.transpose() * A).inverse() * A.transpose() * L_est;

  L_est = A * x_est;
}

Eigen::Matrix<float, 2, Dynamic> Normal::NearestPoints(
    const Eigen::Matrix<float, 2, Dynamic> &Cloud, const int &i) {
  int cols = Cloud.cols();
  int search_window = 10;
  int k = 0;
  int count = 1;
  int s = search_window * 2;

  float left_x;
  float left_y;
  float right_x;
  float right_y;
  float point_x;
  float point_y;

  Eigen::Matrix<float, 2, Dynamic> points;
  points.setZero(2, s);

  points(0, 0) = Cloud(0, i);
  points(1, 0) = Cloud(1, i);
  k++;

  point_x = (Cloud(0, i));
  point_y = (Cloud(1, i));

  while (count <= search_window) {
    if (i - count >= 0) {
      left_x = (Cloud(0, i - count));
      left_y = (Cloud(1, i - count));

      if ((left_x <= (point_x + 2)) && (left_x >= (point_x - 2)) && k < s) {
        if ((left_y <= (point_y + 2)) && (left_y >= (point_y - 2))) {
          points(0, k) = Cloud(0, i - count);
          points(1, k) = Cloud(1, i - count);
          k = k + 1;
        }
      }
    }

    if (i + count < cols) {
      right_x = (Cloud(0, i + count));
      right_y = (Cloud(1, i + count));

      if ((right_x <= (point_x + 2)) && (right_x >= (point_x - 2)) && k < s) {
        if ((right_y <= (point_y + 2)) && (right_y >= (point_y - 2))) {
          points(0, k) = Cloud(0, i + count);
          points(1, k) = Cloud(1, i + count);
          k = k + 1;
        }
      }
    }

    count++;
  }

  Map<MatrixXf> points1(points.data(), 2, k);
  return points1;
}

Eigen::Matrix<float, 2, 1> Normal::GetInitialParams(
    const Eigen::Matrix<float, 2, 2> &points) {
  Eigen::MatrixXf x(2, 1);

  x(0, 0) = (points(1, 0) - points(1, 1)) / (points(0, 0) - points(0, 1));

  x(1, 0) = points(1, 0) - points(0, 0) * x(0, 0);

  return x;
}

// Eigen::Matrix<float, Dynamic, 1> Normal::CalcFunc(const Eigen::Matrix<float,
// 2, Dynamic> points, const Eigen::Matrix<float, 2,1> x){

// 	Eigen::Matrix<float, Dynamic, 1> F_i;

// 	for (int i = 0; i < points.cols(); ++i)
// 	{
// 		F_i(i,0) = x(0,0) * points(0,i) + x(1,0);
// 	}

// 	return F_i;
// }

Eigen::Matrix<float, Dynamic, 2> Normal::Jacobian(
    const Eigen::Matrix<float, 2, Dynamic> &points, int s) {
  Eigen::MatrixXf A(s, 2);

  for (int i = 0; i < points.cols(); ++i) {
    A(i, 0) = points(0, i);
    A(i, 1) = 1;
  }

  return A;
}

Eigen::Matrix<float, Dynamic, 1> Normal::GetInitialObservation(
    const Eigen::Matrix<float, 2, Dynamic> &points, const int s) {
  Eigen::MatrixXf L(s, 1);

  for (int i = 0; i < s; ++i) {
    L(i, 0) = points(1, i);
  }

  return L;
}

void Normal::PrintNormal() {
  for (int i = 0; i < length; ++i) {
    std::cout << " " << normal[i].x << " " << normal[i].y << " " << normal[i].nx
              << " " << normal[i].ny << std::endl;
  }
}
}  // namespace NormalEstimation