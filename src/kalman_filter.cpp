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
  std::cout << "before predict" << std::endl;
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  std::cout << "after predict" << std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
      calc y, using Hlaser to move into sensor space
      calc S based on variance and laser diff
      calc K based on S and R (measurement err)
      calc x based on K
      calc P based on S and R
  */
  VectorXd y = VectorXd(2,1);
  y = z - H_ * x_;
  std::cout << "y is" << y << std::endl;

  MatrixXd S = MatrixXd(2, 2);
  S = H_ * P_ * H_.transpose() + R_;
  std::cout << "S is" << S << std::endl;

  MatrixXd K = MatrixXd(2, 2);
  K = P_ * H_.transpose() * S.inverse();

  x_ = x_ + K * y;

  long x_size = x_.size();
  std::cout << "xsize" << x_size << std::endl;
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_) * P_;
  std::cout << "pred complete" << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
