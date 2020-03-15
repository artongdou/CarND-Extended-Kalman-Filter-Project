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
                        Eigen::MatrixXd &R_radar_in) {
  x_ = x_in;
  P_ = P_in;
  H_laser_ = H_laser_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
  I_ = MatrixXd::Identity(4,4);
  tools_ = Tools();
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  MatrixXd Ht = H_laser_.transpose();
  VectorXd y = z - H_laser_ * x_;
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd K = P_ * Ht * S.inverse();
  x_ = x_ + K * y;
  P_ = (I_ - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  
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
  VectorXd h_prime(3);
  h_prime << rho, phi, rho_dot;
  VectorXd y = z - h_prime;
  
  // Constrain phi to [-pi, pi]
  while (y(1) < -M_PI) {
    y(1) += 2 * M_PI;
  }
  while (y(1) > M_PI) {
    y(1) -= 2 * M_PI;
  }

  MatrixXd Hj = tools_.CalculateJacobian(x_);
  MatrixXd Hjt = Hj.transpose();
  MatrixXd S = Hj * P_ * Hjt + R_radar_;
  MatrixXd K = P_ * Hjt * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K * Hj) * P_;
}
