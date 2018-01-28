#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /*
   * A helper methods to convert switch between Cartesian and Polar coordinates
   */
  VectorXd CartesianToPolar(const VectorXd& x_in);
  VectorXd PolarToCartesian(const VectorXd& x_in);

};

#endif /* TOOLS_H_ */
