#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_ << 1, 1, 0, 0, 0;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.;//30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5; //30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // Initially not initialized
  is_initialized_ = false;

  // Initial time when the state is true, in us
  time_us_ = 0;

  // Init state dimension
  n_x_ = 5;

  // Init augmented state dimension
  n_aug_ = 7;

  // Init sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Init predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.);

  // Init weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for(int i = 1; i < 2 * n_aug_ + 1; i++) {
    //weights_(i) = 1 / (2 * (lambda_ + n_aug_));
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

  // Init radar measurement noise covariance matrix
  R_radar_ = MatrixXd(N_Z_RADAR, N_Z_RADAR);
  R_radar_ << std_radr_*std_radr_,  0,                        0,
              0,                    std_radphi_*std_radphi_,  0,
              0,                    0,                        std_radrd_*std_radrd_;

  // Init laser measurement noise covariance matrix
  R_laser_ = MatrixXd(N_Z_LASER, N_Z_LASER);
  R_laser_ << std_laspx_*std_laspx_,  0,
              0,                      std_laspy_*std_laspy_;

  // Init NIS
  NIS_lidar_ = 0;
  NIS_radar_ = 0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "UKF: " << endl;

    /*
     * init px and py, the rest set to zero
    */
    float px = 0.;
    float py = 0.;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      float rho = meas_package.raw_measurements_[0];
      float theta = meas_package.raw_measurements_[1];

      px = rho * cos(theta);
      py = rho * sin(theta);

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];
    }

    // Init process state
    x_ << px, py, 0., 0., 0.;
    //x_ << px, py, 0.5, 0., 0.;

    // update timestamp in microseconds
    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  //compute the time elapsed between the current and previous measurements
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;   // in seconds
  time_us_ = meas_package.timestamp_;

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  Prediction(delta_t);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && use_radar_)  {
      UpdateRadar(meas_package);
  }
  else if((meas_package.sensor_type_ == MeasurementPackage::LASER) && use_laser_)  {
    UpdateLidar(meas_package);
  }


  // print the output
  //cout << "x_ = " << x_ << endl;
  //cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // Predict sigma points
  SigmaPointPrediction(delta_t);

  // Predict the state, and the state covariance matrix
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  //mean predicted measurement
  VectorXd z_pred = VectorXd(N_Z_LASER);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(N_Z_LASER, N_Z_LASER);

  //matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(N_Z_LASER, 2 * n_aug_ + 1);

  //Predict measurement state mean and covariance
  PredictLaserMeasurement(&z_pred, &S, &Zsig);

  //get incoming radar measurement
  VectorXd z = VectorXd(N_Z_LASER);
  z <<  meas_package.raw_measurements_[0],   //px
        meas_package.raw_measurements_[1];   //py

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, N_Z_LASER);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //calculate cross correlation matrix
  Tc.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    VectorXd dx = Xsig_pred_.col(i) - x_;

    //angle normalization
    while (dx(3)> M_PI) dx(3)-=2.*M_PI;
    while (dx(3)<-M_PI) dx(3)+=2.*M_PI;

    VectorXd dz = Zsig.col(i) - z_pred;

    Tc += weights_(i) * dx * dz.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix

  VectorXd dz = z - z_pred;

  x_ = x_ + K * dz;
  P_ = P_ - K * S * K.transpose();

#if 0

  // Udacity version

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();

#endif


/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  //std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;

  //calculate lidar NIS
  NIS_lidar_ = dz.transpose() * S.inverse() * dz;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //mean predicted measurement
  VectorXd z_pred = VectorXd(N_Z_RADAR);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(N_Z_RADAR, N_Z_RADAR);

  //matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(N_Z_RADAR, 2 * n_aug_ + 1);

  //Predict measurement state mean and covariance
  PredictRadarMeasurement(&z_pred, &S, &Zsig);

  //get incoming radar measurement
  VectorXd z = VectorXd(N_Z_RADAR);
  z <<  meas_package.raw_measurements_[0],   //rho in m
        meas_package.raw_measurements_[1],   //phi in rad
        meas_package.raw_measurements_[2];   //rho_dot in m/s

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, N_Z_RADAR);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //calculate cross correlation matrix
  Tc.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    VectorXd dx = Xsig_pred_.col(i) - x_;

    //angle normalization
    while (dx(3)> M_PI) dx(3)-=2.*M_PI;
    while (dx(3)<-M_PI) dx(3)+=2.*M_PI;

    VectorXd dz = Zsig.col(i) - z_pred;

    //angle normalization
    while (dz(1)> M_PI) dz(1)-=2.*M_PI;
    while (dz(1)<-M_PI) dz(1)+=2.*M_PI;

    Tc += weights_(i) * dx * dz.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix

  VectorXd dz = z - z_pred;

  //angle normalization
  while (dz(1)> M_PI) dz(1)-=2.*M_PI;
  while (dz(1)<-M_PI) dz(1)+=2.*M_PI;

  x_ = x_ + K * dz;
  P_ = P_ - K * S * K.transpose();

#if 0

  // Udacity version

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();

#endif


/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  //std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;

  //calculate radar NIS
  NIS_radar_ = dz.transpose() * S.inverse() * dz;
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/
#if 1
  //create augmented mean state
  x_aug.fill(0);
  x_aug.head(n_x_) = x_;

  //create augmented covariance matrix
  // P_aug = | P 0 |
  //         | 0 Q |
  //
  // Q = | std_a^2           0 |
  //     | 0       std_yawdd^2 |

  P_aug.fill(0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  //create augmented sigma points
  //set first column of sigma point matrix
  Xsig_aug.fill(0);
  Xsig_aug.col(0)  = x_aug;

  //set remaining sigma points
  for (int i = 0; i < n_aug_; i++)
  {
    MatrixXd M = sqrt(lambda_ + n_aug_) * A.col(i);
    Xsig_aug.col(i + 1)           = x_aug + M;
    Xsig_aug.col(i + 1 + n_aug_)  = x_aug - M;
  }

#else
  // Udacity version:

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
#endif

/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  //write result
  *Xsig_out = Xsig_aug;

}

void UKF::SigmaPointPrediction(double delta_t) {

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(&Xsig_aug);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/
#if 1
  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  Xsig_pred_.fill(0);
  for (int k = 0; k < 2* n_aug_ + 1; k++) {

    // Inputs
    VectorXd Xk = VectorXd(n_x_);
    VectorXd dX = VectorXd(n_x_);
    VectorXd dU = VectorXd(n_x_);

    // Xk
    float px      =   Xsig_aug.col(k)(0);
    float py      =   Xsig_aug.col(k)(1);
    float v       =   Xsig_aug.col(k)(2);
    float phi     =   Xsig_aug.col(k)(3);
    float phi_d   =   Xsig_aug.col(k)(4);
    Xk << px, py, v, phi, phi_d;

    // dX
    float d_px;
    float d_py;
    if(fabs(phi_d) > 0.0001) {
      d_px = v/phi_d * (sin(phi + phi_d*delta_t) - sin(phi));
      d_py = v/phi_d * (-cos(phi + phi_d*delta_t) + cos(phi));
    }
    else {
      d_px = v * cos(phi) * delta_t;
      d_py = v * sin(phi) * delta_t;
    }
    dX << d_px, d_py, 0, phi_d * delta_t, 0;

    // dU
    float u_a = Xsig_aug.col(k)(5);
    float u_phi_dd = Xsig_aug.col(k)(6);

    float u_px = delta_t*delta_t/2 * cos(phi) * u_a;
    float u_py = delta_t*delta_t/2 * sin(phi) * u_a;
    float u_v = delta_t * u_a;
    float u_phi = delta_t*delta_t/2 * u_phi_dd;
    float u_phi_d = delta_t * u_phi_dd;
    dU << u_px, u_py, u_v, u_phi, u_phi_d;

    // Output
    Xsig_pred_.col(k) = Xk + dX + dU;
  }

#else

  // Udacity version

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

#endif
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //cout << "Xsig_pred = " << std::endl << Xsig_pred_ << endl;
}

void UKF::PredictMeanAndCovariance(void) {

/*******************************************************************************
 * Student part begin
 ******************************************************************************/
#if 1
  int n_a = 2 * n_aug_ + 1;

  //predict state mean
  x_.fill(0);
  for(int i = 0; i < n_x_; i++)  {
    VectorXd Xsig = Xsig_pred_.row(i);
    for (int j = 0; j < n_a; j++)   {
      x_(i) += weights_(j) * Xsig(j);
    }
  }

  //predict state covariance matrix
  P_.fill(0);
  for (int i = 0; i < n_a; i++) {
    VectorXd dx = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (dx(3)> M_PI) dx(3) -= 2. * M_PI;
    while (dx(3)<-M_PI) dx(3) += 2. * M_PI;
    P_ += weights_(i) * dx * dx.transpose();
  }

#else

  // Udacity version - with angle normalization

  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }

#endif
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "Predicted state" << std::endl;
  //std::cout << x << std::endl;
  //std::cout << "Predicted covariance matrix" << std::endl;
  //std::cout << P << std::endl;
}


void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(N_Z_RADAR, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(N_Z_RADAR);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(N_Z_RADAR, N_Z_RADAR);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/
  //transform sigma points into measurement space
  Zsig.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    double px  = Xsig_pred_(0,i);
    double py  = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double theta = atan2(py, px);

    double temp = sqrt(px*px + py*py);
    // divide by zero check
    if (temp < 0.0001) temp = 0.0001;
    double rho = temp;
    double rho_dot = (px*cos(yaw)*v + py*sin(yaw)*v) / rho;

    Zsig.col(i) << rho, theta, rho_dot;
  }

  //calculate mean predicted measurement
  z_pred.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //calculate innovation covariance matrix S
  S.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd dz = Zsig.col(i) - z_pred;
    //angle normalization
    while (dz(1)> M_PI) dz(1)-=2.*M_PI;
    while (dz(1)<-M_PI) dz(1)+=2.*M_PI;
    S += weights_(i) * dz * dz.transpose();
  }

  // add noise
  S += R_radar_;

#if 0
  // Udacity version

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr*std_radr, 0, 0,
          0, std_radphi*std_radphi, 0,
          0, 0,std_radrd*std_radrd;
  S = S + R;
#endif
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  //std::cout << "S: " << std::endl << S << std::endl;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
}

void UKF::PredictLaserMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out) {

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(N_Z_LASER, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(N_Z_LASER);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(N_Z_LASER, N_Z_LASER);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //transform sigma points into measurement space
  Zsig.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    float px = Xsig_pred_(0,i);
    float py = Xsig_pred_(1,i);
    //Zsig.col(i) << px, py;
    Zsig(0, i) = px;
    Zsig(1, i) = py;
  }

  //calculate mean predicted measurement
  z_pred.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //calculate innovation covariance matrix S
  S.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd dz = Zsig.col(i) - z_pred;
    S += weights_(i) * dz * dz.transpose();
  }

  // add noise
  S += R_laser_;

#if 0
  // Udacity version

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr*std_radr, 0, 0,
          0, std_radphi*std_radphi, 0,
          0, 0,std_radrd*std_radrd;
  S = S + R;
#endif
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  //std::cout << "S: " << std::endl << S << std::endl;

  //write result
  *z_out = z_pred;
  *S_out = S;
  *Zsig_out = Zsig;
}




