#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;


// Constructor
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  // measurement matrix - laser
  H_laser_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;
  // jacobian matrix - radar
  Hj_ << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;
  // covariance matrix Q
  ekf_.Q_ << 0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0;
  // state covariance matrix P
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ = << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;
  // transition matrix F
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ <<  1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;
  // measurement noises - both
  noise_ax = 9.0;
  noise_ay = 9.0;
  // create state vector (do not know the state values yet)
  ekf_.x_ = VectorXd(4)
  ekf_.x_ << 1,1,1,1;
}


// Destructor
FusionEKF::~FusionEKF() {}


// Process Measurements
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  // INITIALIZATION

  // unpack values on initialization
  if (!is_initialized_) {

    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
      float px = measurement_pack.raw_measurements_(0);
      float py = measurement_pack.raw_measurements_(1);
      // laser can't use doppler effect to measure velocity
      ekf_.x_ << px, py, 0.0, 0.0;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR){
      // convert polar to cartesian
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rho_d = measurement_pack.raw_measurements_(2);
      float px = rho*cos(phi);
      float py = rho*sin(phi);
      float vx = rho_d*cos(phi);
      float vy = rho_d*sin(phi);
      ekf_.x_ << px, py, vx, vy;
    }
    // update new time
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }


  // PREDICTION

  // get time elapsed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  // update state transition matrix
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  // update process noise covariance matrix
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  ekf_.Q_ <<  dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
              0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
              dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
              0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
  // call predict with updated values
  ekf_.Predict();

  
  // UPDATE

  // update state and covariance matrices
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
