#include "kalman_filter.h"
#include <iostream>

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
    x_=F_*x_;
    P_=F_*P_*F_.transpose() + Q_;
    
}

void KalmanFilter::Update(const VectorXd &z) {
    cout << "Update " << endl;
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    UpdateBoth(y);
    
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    cout << "Update EKF" << endl;

    VectorXd h = VectorXd(3);
    float r = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    
    // if py is to small - skip measurement, avoiding errors with atan2
    if (fabs(x_(1))>0.5)
    {
        cout << "r=" << r << endl;
        cout << "x(1)=" << x_(1) << endl;
        h << r, atan2(x_(1) , x_(0)), (x_(0)*x_(2) + x_(1)*x_(3)) /r;
        UpdateBoth(z-h);
    }
}

void KalmanFilter::UpdateBoth(const VectorXd &y)
{
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    
    cout << "S:" << endl << S << endl << endl;
    cout << "S:" << endl << S.inverse() << endl << endl;
    
    
    MatrixXd K = P_ * Ht * S.inverse();
    cout << "y=" << y << endl;
    
    //new estimate
    x_ = x_ + (K * y);
    
    cout << "Delta:" <<(K * y) << endl;
    
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

