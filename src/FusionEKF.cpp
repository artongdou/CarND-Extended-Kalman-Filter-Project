#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;
using std::cerr;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  //measurement matrix for laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  noise_ax = 9.0;
  noise_ay = 9.0;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // initialize vector for first measurement
    VectorXd x = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      x << rho*cos(phi), rho*sin(phi), 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      float px = measurement_pack.raw_measurements_(0);
      float py = measurement_pack.raw_measurements_(1);
      x << px, py, 0, 0;
    } else {
      cerr << "Unknown sensor type - Error" << endl;
    }
    
    // initialize covariance matrix P
    MatrixXd P = MatrixXd(4, 4);
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;
    
    ekf_.Init(x, P, H_laser_, R_laser_, R_radar_);
    
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0,  dt,
             0, 0, 1,  0,
             0, 0, 0,  1;

  // set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0,               dt_3/2*noise_ax, 0,
              0,               dt_4/4*noise_ay, 0,               dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0,               dt_2*noise_ax,   0,
              0,               dt_3/2*noise_ay, 0,               dt_2*noise_ay;
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  Tools tools;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    std::cout << "RADAR update" << std::endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    std::cout << "Laser update" << std::endl;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "-------------------------" << endl;
  cout << "x_ = \n" << ekf_.x_ << endl;
  cout << "P_ = \n" << ekf_.P_ << endl;
  cout << "-------------------------" << endl;
}
