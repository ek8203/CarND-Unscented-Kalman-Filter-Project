#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse.fill(0.0);

  // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if(estimations.size() == 0){
      cout << "CalculateRMSE() - Error - estimation vector size should not be zero" << endl;
      return rmse;
  }

  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()){
      cout << "CalculateRMSE() - Error - estimation vector size should equal ground truth vector size" << endl;
      return rmse;
  }
  // ... your code here

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
      // ... your code here
      //cout << estimations [i] - ground_truth[i] << endl;
      VectorXd res = estimations [i] - ground_truth[i];
      res = res.array() * res.array();
      rmse += res;
  }

  //calculate the mean
  // ... your code here
  rmse = rmse / estimations.size();

  //calculate the squared root
  // ... your code here
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

VectorXd Tools::CartesianToPolar(const VectorXd& x_in)  {
    /**
     * Convert Cartesian coordinates to polar:
     * Input is 4x vector in Cartesian coordinates: px, py, vx, vy
     * Output is a 3x vector in polar coordinate: rho, theta, rho_dot
     */
    VectorXd x_out = VectorXd(3);
    x_out.fill(0.0);

    float rho;
    float theta;
    float rho_dot;

    // Cartesian coordinates
    float   px = x_in(0),
            py = x_in(1),
            vx = x_in(2),
            vy = x_in(3);

    if (fabs(px) < 0.0001) {
        cout << "Tools::CartesianToPolar() - Error - Devide by zero!" << endl;
        px = 0.0001;
    }

    // calculate distance (range) rho
    rho = sqrt(px*px + py*py);

    if (fabs(rho) < 0.0001) {
        cout << "Tools::CartesianToPolar() - Error - Devide by zero!" << endl;
        rho = 0.0001;
    }

    // calculate angle theta in range -pi < theta < pi
    theta = atan2(py, px);

    // calculate range rate ro_dot
    rho_dot = (px*vx + py*vy)/rho;

    // output vector
    x_out << rho, theta, rho_dot;

    return x_out;
}

/*
 * Input <VectorXd> x_in: Polar state px, py
 * Output <VectorXd>: Cartesian state rho, theta(phi), rho_dot
 */
VectorXd Tools::PolarToCartesian(const VectorXd& x_in)  {
    /**
     * Convert polar coordinates to Cartesian:
     * Input is a 3x vector in polar coordinate: rho, theta, rho_dot
     * Output is 4x vector in Cartesian coordinates: px, py, vx, vy
     */

    VectorXd x_out = VectorXd(4);
    x_out.fill(0.0);

    float rho = x_in(0);
    float theta = x_in(1);
    float rho_dot = x_in(2);

    // normalize y
    float pi_2 = 2*M_PI;
    while(theta > M_PI)     theta -= pi_2;
    while(theta < -M_PI)    theta += pi_2;

    float px = rho * cos(theta);
    float py = rho * sin(theta);
    float vx = rho_dot * cos(theta);
    float vy = rho_dot * sin(theta);

    x_out << px, py, vx, vy;

    return x_out;
}
