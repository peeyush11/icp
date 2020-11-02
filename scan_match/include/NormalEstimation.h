#pragma once

#include "iostream"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <tuple>
#include <vector>
#include <math.h>


using namespace Eigen;

namespace NormalEstimation{
   
   struct data
   {
      float x, y, nx, ny;

      data(){}

      data(float x, float y, float nx, float ny)
         : x(x), y(y), nx(nx), ny(ny){}

   };


   class Normal{

        public:

         std::vector<data> normal;
         std::vector<int> sample_normal;
         int length,count;
         
         Normal();
         Normal(int size);
         virtual ~Normal();

         void GetNormal(const Eigen::Matrix<float, 2, Dynamic> &SourceCloud);

         void FitLine(const Eigen::Matrix<float,2,Dynamic> &points,Eigen::MatrixXf &L_est, Eigen::MatrixXf &x_est);

         Eigen::Matrix<float, 2, Dynamic> NearestPoints(const Eigen::Matrix<float, 2, Dynamic> &Cloud,const int &i);

         Eigen::Matrix<float, 2 , 1> GetInitialParams(const Eigen::Matrix<float, 2, 2> &points);

         Eigen::Matrix<float, Dynamic, 1> CalcFunc(const Eigen::Matrix<float, 2, Dynamic> &points, const Eigen::Matrix<float,2,1> x);

         Eigen::Matrix<float, Dynamic, 2> Jacobian(const Eigen::Matrix<float, 2, Dynamic> &points, int s);

         Eigen::Matrix<float,Dynamic,1> GetInitialObservation(const Eigen::Matrix<float,2,Dynamic> &points,  const int s);

         void SampleNormal();

         void PrintNormal();

   };


} // namespace NormalEstimation