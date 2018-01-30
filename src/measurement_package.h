#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

/*
 *  Radar measurement dimension, radar can measure r, phi, and r_dot
 */
#define N_Z_RADAR  3

/*
 *  Laser measurement dimension, radar can measure px and py
 */
#define N_Z_LASER  2

class MeasurementPackage {
public:
  long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
