#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  // predict the state
  x_ = F_ * x_ // set "u" to zero for convenience
  P_ = F_ * P_ * F_.transpose() + Q_
}

void KalmanFilter::Update(const VectorXd &z) {
  // update state (KF)
  VectorXd y = z - H_ * x_
  updateRoutine(y)
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // update state (EKF)
  v_polar = tools.ConvertToPolar(x_)
  double rho = v_polar(0);
  double phi = v_polar(1);
  double rho_d = v_polar(2);

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;

  // update y
  VectorXd y = z - z_pred

  // normalize angle
  while (y(1) > M_PI)
    y(1) -= 2 * M_PI;
  while(y(1) < M_PI)
    y(1) += 2 * M_PI;

  updateRoutine(y)
}
