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
