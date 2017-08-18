#include "kalman_filter.h"
#include <iostream>
#include <iomanip>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;


}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  UpdateBoth(z - H_ * x_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd h = VectorXd(3);
  float r = sqrt(x_(0) * x_(0) + x_(1) * x_(1));


  // if r is to small - skip measurement, avoiding errors

  if ( r > 0.01 )
  {
    h << r, atan2(x_(1) , x_(0)), (x_(0)*x_(2) + x_(1)*x_(3)) / r;
    VectorXd y = z - h;
    //convert to -PI/2 +PI/2
    while (y(1) > M_PI / 2) {
      y(1) -= M_PI;
    }
    while (y(1) <= -M_PI / 2)
    {
      y(1) += M_PI;
    }
    UpdateBoth(y);
  }
}

void KalmanFilter::UpdateBoth(const VectorXd &y)
{

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;

  MatrixXd K = P_ * Ht * S.inverse();
  VectorXd d = K * y;


  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

