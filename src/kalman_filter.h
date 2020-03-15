#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
private:
  // Identity Matrix
  Eigen::MatrixXd I_;
  
  /**
   * Calculate the Kalman gain
   * @param H mesaurement matrix
   * @param P covariance matrix
   * @param R mesasurment noise matrix
   */
  Eigen::MatrixXd CalculateKalmanGain(Eigen::MatrixXd H, Eigen::MatrixXd P,
                                      Eigen::MatrixXd R);
public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param H_laser_in Laser Measurement matrix
   * @param R_laser_in Laser Measurement covariance matrix
   * @param R_radar_in Radar Measurement covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &H_laser_in,
            Eigen::MatrixXd &R_laser_in, Eigen::MatrixXd &R_radar_in,
            Eigen::MatrixXd &F_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);
  
  /**
  * get polar state vector
  */
  Eigen::VectorXd get_polar_state();

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // laser measurement matrix
  Eigen::MatrixXd H_laser_;

  // laser measurement covariance matrix
  Eigen::MatrixXd R_laser_;
  
  // radar measurement covariance matrix
  Eigen::MatrixXd R_radar_;
  
  // Tools object
  Tools tools_;
};

#endif // KALMAN_FILTER_H_
