#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in,
                        Eigen::MatrixXd &H_laser_in, Eigen::MatrixXd &R_laser_in,
                        Eigen::MatrixXd &R_radar_in, Eigen::MatrixXd &F_in) {
  x_ = x_in;
  P_ = P_in;
  H_laser_ = H_laser_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
  F_ = F_in;
  I_ = MatrixXd(4,4);
  I_ << MatrixXd::Identity(4,4);
  tools_ = Tools();
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_laser_ * x_;
  MatrixXd K = CalculateKalmanGain(H_laser_, P_, R_laser_);
  x_ = x_ + K * y;
  P_ = (I_ - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd y = z - get_polar_state();
  
  // Constrain phi to [-pi, pi]
  while (y(1) < -M_PI) {
    y(1) += 2 * M_PI;
  }
  while (y(1) > M_PI) {
    y(1) -= 2 * M_PI;
  }

  MatrixXd Hj = tools_.CalculateJacobian(x_);
  MatrixXd K = CalculateKalmanGain(Hj, P_, R_radar_);
  x_ = x_ + K * y;
  P_ = (I_ - K * Hj) * P_;
}

MatrixXd KalmanFilter::CalculateKalmanGain(MatrixXd H, MatrixXd P,
                                           MatrixXd R) {
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P * Ht + R;
  MatrixXd K = P * Ht * S.inverse();
  return K;
}

VectorXd KalmanFilter::get_polar_state() {
  VectorXd x_polar(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float rho = sqrt(px*px + py*py);
  float phi = atan2(py, px);
  if (rho < 0.000001) {
    rho = 0.000001;
  }
  float rho_dot = (px*vx + py*vy)/rho;
  
  x_polar << rho, phi, rho_dot;
  return x_polar;
}
