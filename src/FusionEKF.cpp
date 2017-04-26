#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // F = MatrixXd(4, 4);
  // Q = MatrixXd(4, 4);
  // P = MatrixXd(4, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Hj_ << 0, 0, 0, 0,
  //        0, 0, 0, 0,
  //        0, 0, 0, 0;

  noise_ax_ = 9;
  noise_ay_ = 9;
  tools = Tools();
  num_printed = 0;

  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  // cout << "after f " << endl;

  MatrixXd Q_ = MatrixXd(4, 4);
  // Q_ << 1, 0, 1, 0,
  //       0, 1, 0, 1,
  //       1, 0, 1, 0,
  //       0, 1, 0, 1;

  MatrixXd P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  VectorXd x_ = VectorXd(4);
  x_ << 1, 1, 1, 1;

  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


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
    cout << "EKF: " << endl;
    // x_ = VectorXd(4);
    // ekf_.x_ << 1, 1, 1, 1;
    previous_timestamp_ = measurement_pack.timestamp_;

    // cout << "after p " << endl;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // TODO: initialize based on conversion polor to cartesian
      double ro = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      ekf_.x_ << ro*std::cos(phi), ro*std::sin(phi), 0, 0;
      // Hj_ = CalculateJacobian(ekf_.x_);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      // cout << "start inti: " << endl;

      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      // cout << "after x " << endl;


    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     x Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     x Update the process noise covariance matrix.
     x Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  long long incoming = measurement_pack.timestamp_;
  float dt = (incoming - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = incoming; 
  // cout << "incoming time: " << incoming << endl; 
  cout << "dt: " << dt << endl;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  cout << "f set" << endl;

  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  ekf_.Q_ << dt_4 / 4 * noise_ax_, 0, dt_3 / 2 * noise_ax_, 0,
             0, dt_4 / 4 * noise_ay_, 0, dt_3 / 2 * noise_ay_,
             dt_3 / 2 * noise_ax_, 0, dt_2 * noise_ax_, 0,
             0, dt_3 /2 * noise_ay_, 0, dt_2 * noise_ay_;  
  cout << "q set " << endl;

  ekf_.Predict();

  // std::cout << "fusion after predict" << std::endl;
  /*****************************************************************************
      num_printed += 1;
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    std::cout << "radar before jacobian" << std::endl;
    ekf_.R_ = R_radar_;
    std::cout << "test successful" << std::endl;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    std::cout << "r after jacobian" << std::endl;
    // std::cout << "before P is" << ekf_.P_ << std::endl;
    // std::cout << "before F is" << ekf_.F_ << std::endl;
    // std::cout << "before Q is" << ekf_.Q_ << std::endl;
    // ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, Hj_, R_radar_, ekf_.Q_);
    // std::cout << "curr h is" << ekf_.H_ << std::endl;
    // ekf_.H_ << 0, 0, 0, 0,
    //            0, 0, 0, 0,
    //            0, 0, 0, 0;
    // ekf_.H_ << Hj_;
    // std::cout << "HJ set" << std::endl;
    // ekf_.R_ << R_radar_;
    // std::cout << "measurements" << measurement_pack.raw_measurements_ << std::endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    // std::cout << "r after update" << std::endl;
  } else {
    // Laser updates
    // cout << "before update" << endl;
    // ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, H_laser_, R_laser_, ekf_.Q_);
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
    // cout << "after update" << endl;
  }

  // if (num_printed < 5) {
  // cout << "jacobian" << Hj_ << endl;
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
    // num_printed += 1;
  // }
  // print the output
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}
