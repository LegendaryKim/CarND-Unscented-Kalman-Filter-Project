#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5); //state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate]

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
//  std_a_ = 30;
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
//  std_yawdd_ = 30;
  std_yawdd_ = 0.5;

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


  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // State dimension
  n_x_ = x_.size();

  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  //predicted sigma points matrix
  Xsig_pred_= MatrixXd(n_x_, n_sig_);

  // Weights of sigma points
  weights_ = VectorXd(n_sig_);

  //Measurement noise covariance initialization: Laser
  R_laser_ = MatrixXd(2,2);
  R_laser_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  //Measurement noise covariance initialization: Radar
  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0, std_radrd_*std_radrd_;


}

UKF::~UKF() {}

/**
 * Angle normalization to [-pi, pi]
 * @param angle
 */
void UKF::NormalAng(double *angle) {
//  while (*angle > M_PI) *angle -= 2. * M_PI;
//  while (*angle < -M_PI) *angle += 2. * M_PI;
  *angle = atan2(sin(*angle), cos(*angle));
}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  //Initialization
  if (!is_initialized_) {
    // Initial covariance matrix
//    P_ << 1, 0, 0, 0, 0,
//          0, 1, 0, 0, 0,
//          0, 0, 1, 0, 0,
//          0, 0, 0, 1, 0,
//          0, 0, 0, 0, 1;
    P_ = MatrixXd::Identity(n_x_, n_x_);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "UKF: First measurement RADAR" << endl;

      // Convert radar from polar to cartesian coordinates and initialize state.
      double rho = meas_package.raw_measurements_[0]; //range
      double phi = meas_package.raw_measurements_[1]; //bearing
      double rho_d = meas_package.raw_measurements_[2]; // vel. of rho

      double px = rho * cos(phi);
      if (fabs(px) < 0.0001) {
        px = 0.0001;
      }
      double py = rho * sin(phi);
      if (fabs(py) < 0.0001) {
        py = 0.0001;
      }
      double vx = rho_d * cos(phi);
      double vy = rho_d * sin(phi);
      double v = sqrt(vx * vx + vy * vy);

      x_ << px, py, v, 0, 0; //state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate]

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      cout << "UKF: First measurement LASER" << endl;
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      // Deal with the special case initialisation problems
      if (fabs(x_(0)) < 0.0001){
        x_(0) = 0.0001;
      }
      if (fabs(x_(1)) < 0.0001){
        x_(1) = 0.0001;
      }


    }

    //Initialize weights
    weights_.fill(0.5/(lambda_ + n_aug_));
    weights_(0) = lambda_/(lambda_ + n_aug_);
//    for (int i = 1; i < weights_.size() ; ++i) {
//      weights_(i) = 0.5/(lambda_ + n_aug_);
//    }


    //Saving first timestamp for dt calculation
    time_us_ = meas_package.timestamp_;

    //Done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "Init::" << endl;
    cout << "x_" << x_ << endl;


    return;
  }

  double dt = (meas_package.timestamp_- time_us_)/1000000.0; //convert ms to s.
  time_us_ = meas_package.timestamp_;

  // Predicts
  Prediction(dt);

  // Update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  // Augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  // Augmented state covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  // Augmented sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  // Fill the x_aug and P_aug
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_*std_yawdd_;

  // Calculate square root of P_ and lambda + n_aug
  MatrixXd sqrt_P_aug = P_aug.llt().matrixL();
  double sqrt_lambda_n_aug_ = sqrt(lambda_ + n_aug_);
  VectorXd sqrt_lambda_n_aug_sqrt_P_aug;
  // Set sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_ ; ++i) {
    sqrt_lambda_n_aug_sqrt_P_aug = sqrt_lambda_n_aug_ * sqrt_P_aug.col(i);
    Xsig_aug.col(i+1) = x_aug + sqrt_lambda_n_aug_sqrt_P_aug;
    Xsig_aug.col(n_aug_ + i+1) = x_aug - sqrt_lambda_n_aug_sqrt_P_aug;
  }



  // Predict sigma points
  double delta_t2 = delta_t * delta_t;
  for (int i = 0; i < n_sig_; ++i) {
    //exact values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //Predict state values
    double px_p, py_p;
    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //avoid division by zero
    if (fabs(yawd) > 0.0001) {
      px_p = p_x + v/yawd * (sin(yaw_p) - sin(yaw));
      py_p = p_y + v/yawd * (-cos(yaw_p) + cos(yaw));
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    //add noise
    px_p += 0.5 * nu_a * delta_t2 * cos(yaw);
    py_p += 0.5 * nu_a * delta_t2 * sin(yaw);
    v_p += delta_t * nu_a;
    yaw_p += 0.5 * delta_t2 * nu_yawdd;
    yawd_p += delta_t * nu_yawdd;

    //write predicted sigma points into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;

  //Predict state mean
  x_ = Xsig_pred_ * weights_;

  //Predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sig_; ++i) {
    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormalAng(&(x_diff(3)));
//    while(x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
//    while(x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }

  // print the output
  cout << "Predict::" << endl;
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}



/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  //set measurement dimension (p_x, p_y)
  int n_z = 2;
  // Create matrix for sigma points in measurement space
  // Transform sigma points into measurement space
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);
  UpdateUKF(meas_package, Zsig, n_z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //set measurement dimension (r, phi, r_dot)
  int n_z = 3;
  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);
  // Transform sigma points into measurement space
  for (int i = 0; i < n_sig_ ; ++i) {
    //extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = v*cos(yaw);
    double v2 = v*sin(yaw);

    //measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y); //r
    Zsig(1,i) = atan2(p_y, p_x); // phi
    if (fabs(Zsig(1,i)) < 0.0001) {
      Zsig(1,i) = 0.0001;
    }
    Zsig(2,i) = (p_x*v1 + p_y*v2) / Zsig(0,i); // r_dot
  }
  UpdateUKF(meas_package, Zsig, n_z);
}

void UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z) {
  // Predicted Measurement mean
  VectorXd z_pred = VectorXd(n_z);
  z_pred = Zsig * weights_;

  // Calculate Predicted Measurement covariance matrix
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; ++i) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    if (meas_package.sensor_type_== MeasurementPackage::RADAR) {
      //angle normalization
      NormalAng(&(z_diff(1)));
//      while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
//      while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    }

    S += weights_(i) * z_diff * z_diff.transpose();
  }

    //Add Measurement noise covariance-
  MatrixXd R = MatrixXd(n_z, n_z);
  if (meas_package.sensor_type_== MeasurementPackage::LASER) {
    R = R_laser_;
  }
  else if (meas_package.sensor_type_== MeasurementPackage::RADAR) {
    R = R_radar_;
  }
  S += R;

  // calculate cross-correlation matrix between sigma points in state space & measurement space
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; ++i) {
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    if (meas_package.sensor_type_== MeasurementPackage::RADAR) {
      //angle normalization
      NormalAng(&(z_diff(1)));
//      while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
//      while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
    }

    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormalAng(&(x_diff(3)));
//    while (x_diff(3) > M_PI) x_diff(3) -= 2. *M_PI;
//    while (x_diff(3) < -M_PI) x_diff(3) += 2. *M_PI;

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate kalman gain K
  MatrixXd K = Tc * S.inverse();

  // State update
  VectorXd z = meas_package.raw_measurements_;
  cout << "z =" << z << endl;

  // residual
  VectorXd z_diff = z - z_pred;
  //angle normalization
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    NormalAng(&(z_diff(1)));
//    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
//    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
  }

  // Update state mean and covariance matrix
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  // Calculate Normalized Innovation Squared(NIS)
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    NIS_laser = z_diff.transpose() * S.inverse() *z_diff;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    NIS_radar = z_diff.transpose() * S.inverse() *z_diff;
  }

  // print the output
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    cout << "Update(Laser)::" << endl;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    cout << "Update(Radar)::" << endl;
  }
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}