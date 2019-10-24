#include "FusionEKF.h"
#include <iostream>
#include "eigen-323c052e1731/Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  covMatrixLaser_ = MatrixXd(2, 2);
  covMatrixRadar_ = MatrixXd(3, 3);
  laserMeasurementMatrix_ = MatrixXd(2, 4);
  radarJacobianMatrix_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser, R_laser 
  covMatrixLaser_ << 0.0225, 0,
      0, 0.0225;

  // measurement covariance matrix - radar, R_radar
  covMatrixRadar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;
  
  // measurement matrix for laser readings, H 
  laserMeasurementMatrix_ << 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0;

  // Jacobian matrix of measurement function for radar readings, Hj
  radarJacobianMatrix_ << 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0;

  // Set the process and measurement noises

  // acceleration noises. they can be set to study the effect of 
  axNoise_ = 9.0;
  ayNoise_ = 9.0;

  ekf_.init();
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF()
{
  
}

void FusionEKF::processMeasurement(const MeasurementPackage &measurementPack)
{
  /**
   * Initialization
   */
  if (!is_initialized_)
  {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.states_ = VectorXd(4);
    ekf_.states_ << 1, 1, 1, 1;

    if (measurementPack.sensor_type_ == MeasurementPackage::RADAR)
    {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
    }
    else if (measurementPack.sensor_type_ == MeasurementPackage::LASER)
    {
      // TODO: Initialize state.
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // kf_.predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurementPack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // TODO: Radar updates
  }
  else
  {
    // TODO: Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.states_ << endl;
  cout << "P_ = " << ekf_.stateCovMatrix_ << endl;
}
