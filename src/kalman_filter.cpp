#include "kalman_filter.h"
#include <iostream> 

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

//**need to init!!
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
  /**
  TODO:
    * predict the state
  */
  // std::cout << "before predict" << std::endl;
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  // std::cout << "after predict" << std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // std::cout << "z lidar: " << z << std::endl;
  VectorXd y = VectorXd(2);
  y = z - H_ * x_;
  // std::cout << "y is" << y << std::endl;

  MatrixXd S = MatrixXd(2, 2);
  S = H_ * P_ * H_.transpose() + R_;
  // std::cout << "S is" << S << std::endl;

  MatrixXd K = MatrixXd(2, 2);
  K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;

  long x_size = x_.size();
  // std::cout << "xsize" << x_size << std::endl;
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
  // std::cout << "pred complete" << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // std::cout << "radar z: " << z << std::endl;
  // std::cout << "new jacob" << std::endl;
  // if (i < 5) {
  //   std::cout << "xstarts: " << x_ << std::endl;
  //   i += 1;
  // }
  float sqrt_sq = std::sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  if (sqrt_sq < 1e-6) 
    sqrt_sq = 1e-6;
  VectorXd h(3);

  h << sqrt_sq, atan2(x_(1), x_(0)), (x_(0)*x_(2) + x_(1)*x_(3))/sqrt_sq;
  VectorXd y = z - h;
  while (y(1) > 3.1415) {
    y(1) -= 6.2830;
  }
  while (y(1) < -3.1415) {
    y(1) += 6.2830;
  }
  // if (y(1) > 3.14 || y(1) < -3.14) {
  //   std::cout << "wrong one found " << y(1) << std::endl;
  // }
  // // std::cout << "h" << h << std::endl;
  // // std::cout << "z" << z << std::endl;
  // // std::cout << "y" << y << std::endl;
  // // std::cout << "before norm" << y(1) << std::endl;
  // // // y(1) = atan2(y(1));
  // // std::cout << "normed y" << y(1) << std::endl;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  // // std::cout << "new x radar" << std::endl;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
  // std::cout << "new P radar" << std::endl;
  // x_ = x_;
  // P_ = P_;

}
