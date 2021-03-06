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

  // set the initialisation param to false
  is_initialized_ = false;

  // initialise delta_t to an initial value
  time_us_ = 0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

  // initial sigma point params
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  n_sigma_ = 2 * n_aug_ + 1;
  n_z_radar_ = 3;
  n_z_lidar_ = 2;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);

  // initial weights matrix
  weights_ = VectorXd(n_sigma_);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<n_sigma_; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  // initialise the sigma prediction matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sigma_);
  Xsig_pred_.fill(0.0);

  // intialise private variables
  delta_t_ = 0;

  // intialise sigma points variable
  Xsig_aug_ = MatrixXd(n_aug_, n_sigma_);
  Xsig_aug_.fill(0.0);

  //add measurement noise covariance matrix
  R_radar_ = MatrixXd(n_z_radar_,n_z_radar_);
  R_radar_ << std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;

  R_lidar_ = MatrixXd(n_z_lidar_,n_z_lidar_);
  R_lidar_ << std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;

}

UKF::~UKF() {}

void UKF::setDelta_t(const long long timestamp) {
  // calculate and update private variable
  delta_t_ = (timestamp - time_us_) / 1000000.0;
  time_us_ = timestamp;
}

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {
  // generate augmented sigma points and pass to matrix for storing...
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_temp = MatrixXd(n_aug_, n_sigma_);

  //create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_temp.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_temp.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_temp.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //write result
  *Xsig_out = Xsig_temp;

}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_temp = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //predict sigma points
  for (int i = 0; i< n_sigma_; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t_) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t_) );
    }
    else {
        px_p = p_x + v*delta_t_*cos(yaw);
        py_p = p_y + v*delta_t_*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t_;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t_*delta_t_ * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t_*delta_t_ * sin(yaw);
    v_p = v_p + nu_a*delta_t_;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t_*delta_t_;
    yawd_p = yawd_p + nu_yawdd*delta_t_;

    //write predicted sigma point into right column
    Xsig_temp(0,i) = px_p;
    Xsig_temp(1,i) = py_p;
    Xsig_temp(2,i) = v_p;
    Xsig_temp(3,i) = yaw_p;
    Xsig_temp(4,i) = yawd_p;
  }

  //write result
  *Xsig_out = Xsig_temp;
}

void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {

  // take the augmented proedicted sigma points and get a mean and covar

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);
  x.fill(0.0);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);
  P.fill(0.0);

  //predicted state mean
  for (int i = 0; i < n_sigma_; i++) {  //iterate over sigma points
    x = x + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  for (int i = 0; i < n_sigma_; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    //update P
    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }

  //write result
  *x_out = x;
  *P_out = P;

}

void UKF::PredictRadarMeasurement(VectorXd* z_pred_out, MatrixXd* S_out, MatrixXd* Z_out) {

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, n_sigma_);

  //transform sigma points into measurement space
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  z_pred.fill(0.0);
  for (int i=0; i < n_sigma_; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_,n_z_radar_);
  S.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  S = S + R_radar_;

  //write result
  *z_pred_out = z_pred;
  *S_out = S;
  *Z_out = Zsig;
}

void UKF::UpdateStateRadar(VectorXd z, VectorXd z_pred_out, MatrixXd S_out, MatrixXd Z_out) {

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(5, 3);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Z_out.col(i) - z_pred_out;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S_out.inverse();

  //residual
  VectorXd z_diff = z - z_pred_out;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  float nis = z_diff.transpose() * S_out.inverse() * z_diff;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_out*K.transpose();

  //print result
  std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
  std::cout << "RNIS|" << nis << std::endl;

}

void UKF::PredictLidarMeasurement(VectorXd* z_pred_out, MatrixXd* S_out, MatrixXd* Z_out) {

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_lidar_, n_sigma_);

  //transform sigma points into measurement space
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    //double v  = Xsig_pred_(2,i);
    //double yaw = Xsig_pred_(3,i);

    //double v1 = cos(yaw)*v;
    //double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_lidar_);
  z_pred.fill(0.0);
  for (int i=0; i < n_sigma_; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z_lidar_,n_z_lidar_);
  S.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  S = S + R_lidar_;

  //write result
  *z_pred_out = z_pred;
  *S_out = S;
  *Z_out = Zsig;

}

  // update lidar step
  void UKF::UpdateStateLidar(VectorXd z, VectorXd z_pred_out, MatrixXd S_out, MatrixXd Z_out) {
    //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(5, 2);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Z_out.col(i) - z_pred_out;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S_out.inverse();

  //residual
  VectorXd z_diff = z - z_pred_out;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_out*K.transpose();

  float nis = z_diff.transpose() * S_out.inverse() * z_diff;

  //print result
  std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
  std::cout << "LNIS|" << nis << std::endl;

  }


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
    Initialise on first measurement, then process
  */
  if (!is_initialized_) {
    // initialise X state based on sensor measurement
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // collect raw sensor values
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];

      //get the cartesian equivalents
      float p_x = rho * cos(phi);
      float p_y = rho * sin(phi);

      // set X - P is Identity so keep as-is
      x_(0) = p_x;
      x_(1) = p_y;

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // can set x from raw measurements if laser
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
    }

    // set time delta
    time_us_ = meas_package.timestamp_;
    cout << "Intialised: " << time_us_ << endl;
    cout << x_ << endl;
    cout << P_ << endl;
    cout << meas_package.sensor_type_ << endl;
    cout << endl;

    is_initialized_ = true;
    return;
  }

  // for measurements beyond the first...

  // update delta t
  setDelta_t(meas_package.timestamp_);

  // process measurement depending on type and status params
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR &&
      use_radar_ == true) {

    // predict
    cout << "predict R " << endl;
    Prediction(delta_t_);

    // then update
    cout << "update R " << endl;
    UpdateRadar(meas_package);

  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER &&
      use_laser_ == true) {

    // predict
    cout << "predict L " << endl;
    Prediction(delta_t_);

    // then update
    cout << "update L " << endl;
    UpdateLidar(meas_package);
  }
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

  // generate sigma points
  AugmentedSigmaPoints(&Xsig_aug_);

  // predict sigma points
  SigmaPointPrediction(&Xsig_pred_);

  // predict state mean and covariance
  PredictMeanAndCovariance(&x_, &P_);
  cout << x_ << endl;
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

  // predict into the measurement space
  VectorXd z_pred_out = VectorXd(n_z_lidar_);
  MatrixXd S_out = MatrixXd(n_z_lidar_, n_z_lidar_);
  MatrixXd Z_out = MatrixXd(n_z_lidar_, n_sigma_);

  PredictLidarMeasurement(&z_pred_out, &S_out, &Z_out);

  // now update the state
  VectorXd z = VectorXd(n_z_lidar_);
  z(0) = meas_package.raw_measurements_[0];
  z(1) = meas_package.raw_measurements_[1];

  UpdateStateLidar(z, z_pred_out, S_out, Z_out);
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

  // predict into the measurement space
  VectorXd z_pred_out = VectorXd(n_z_radar_);
  MatrixXd S_out = MatrixXd(n_z_radar_, n_z_radar_);
  MatrixXd Z_out = MatrixXd(n_z_radar_, n_sigma_);
  PredictRadarMeasurement(&z_pred_out, &S_out, &Z_out);

  // now update the state
  VectorXd z = VectorXd(n_z_radar_);
  z(0) = meas_package.raw_measurements_[0];
  z(1) = meas_package.raw_measurements_[1];
  z(2) = meas_package.raw_measurements_[2];

  UpdateStateRadar(z, z_pred_out, S_out, Z_out);

}
