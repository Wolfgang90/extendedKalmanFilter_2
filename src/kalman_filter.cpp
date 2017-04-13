#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /**
  TODO:
    * predict the state
  */
    
  x_ = F_ * x_;
  MatrixXd F_T = F_.transpose();
  P_ = F_ * P_ * F_T + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd H_T = H_.transpose();
  MatrixXd S = H_ * P_ * H_T + R_;
  MatrixXd S_I = S.inverse();
  MatrixXd PH_T = P_ * H_T;
  MatrixXd K = PH_T * S_I;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float ro_pred = pow(pow(px,2)+pow(px,2), 0.5);
  float phi_pred = 0.0;
  if (fabs(ro_pred) > 0.1){
    phi_pred = atan2(py, px);
  }

  float ro_dot_pred = 0.0;
  if (fabs(rho_pred) > 0.1) {
  ro_dot_pred = (px * vx + py * vy) / ro_pred;
  }

  VectorXd z_pred(3);
  z_pred << ro_pred, phi_pred, ro_dot_pred;

  VectorXd y = z -z_pred;
  MatrixXd H_T = H_.transpose();

  MatrixXd S = H_ * P_ * H_T + R_;
  MatrixXd S_I = S.inverse();
  MatrixXd PH_T = P_ * H_T;
  MatrixXd K = PH_T * S_I;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
