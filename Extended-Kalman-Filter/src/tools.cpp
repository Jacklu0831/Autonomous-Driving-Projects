#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}


// calculate root mean square error
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // fill up root mean square array
  for (int i=0;i<estimations.size();i++){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }
  rmse = rmse.array().sqrt()

  return rmse;
}


// calculate jacobian matrix
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  // get state params
  const double& px = x_state(0);
  const double& py = x_state(1);
  const double& vx = x_state(2);
  const double& vy = x_state(3);

  // prepare terms for convenience
	double c1 = px * px + py * py;
  double c2 = sqrt(c1);
  double c3 = (c1 * c2);

  // avoid division by zero / very small number
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // fill Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py-vy*px)/c3, px*(px*vy-py*vx)/c3, px/c2, py/c2;

  return Hj;
}


// convert cartesian to polar
VectorXd Tools::ConvertToPolar(const VectorXd& v_cartesian){
  const double& px = v_cartesian(0);
  const double& py = v_cartesian(1);
  const double& vx = v_cartesian(2);
  const double& vy = v_cartesian(3);

  double rho, phi, rho_d:
  rho = sqrt(px*px+py*py);
  phi = atan2(py,px);

  if (rho<0.0001){
  	rho = 0.0001;
  }

  rho_d = (px*vx+py*vy)/rho;

  VectorXd v_polar = VectorXd(3);
  v_polar << rho, phi, rho_d;

  return v_polar;
}


// calculate covariant matrix (Q)
MatrixXd Tools::CalculateCovariantMatrix(const double dt, const double noise_ax, const double noise_ay) {
  MatrixXd Q(4, 4);

  const double dt2 = dt * dt;
  const double dt3 = dt * dt2;
  const double dt4 = dt * dt3;

  const double r11 = dt4 * noise_ax / 4;
  const double r13 = dt3 * noise_ax / 2;
  const double r22 = dt4 * noise_ay / 4;
  const double r24 = dt3 * noise_ay / 2;
  const double r31 = dt3 * noise_ax / 2;
  const double r33 = dt2 * noise_ax;
  const double r42 = dt3 * noise_ay / 2;
  const double r44 = dt2 * noise_ay;

  Q << r11, 0.0, r13, 0.0,
       0.0, r22, 0.0, r24,
       r31, 0.0, r33, 0.0,
       0.0, r42, 0.0, r44;

  return Q;















