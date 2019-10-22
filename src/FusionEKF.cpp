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
  measurementMatrixLaser_ = MatrixXd(2, 4);
  jacobianMatrixRadar_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser, R_laser 
  covMatrixLaser_ << 0.0225, 0,
      0, 0.0225;

  // measurement covariance matrix - radar, R_radar
  covMatrixRadar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;
  
  // measurement matrix for laser readings, H 
  measurementMatrixLaser_ << 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0;

  // Jacobian matrix of measurement function for radar readings, Hj
  jacobianMatrixRadar_ << 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0;

  // Set the process and measurement noises

  // initial state vector X
  VectorXd states = VectorXd(4);
  states << 0.0, 0.0, 0.0, 0.0;

  // initial state covariance matrix P
  MatrixXd stateCovMatrix = MatrixXd(4, 4);
  stateCovMatrix << 1.0, 0.0, 0.0, 0.0, 
    0.0, 1.0, 0.0, 0.0, 
    0.0, 0.0, 1000.0, 0.0, 
    0.0, 0.0, 0.0, 1000.0;

  // initialize state transition matrix, F
  MatrixXd stateTransMatrix = MatrixXd(4, 4);
  stateTransMatrix << 1.0, 0.0, 0.0, 0.0, 
    0.0, 1.0, 0.0, 0.0, 
    0.0, 0.0, 1.0, 0.0, 
    0.0, 0.0, 0.0, 1.0;

  // initialize process covariance matrix, Q
  MatrixXd processCovMatrix = MatrixXd(4, 4);
  processCovMatrix << MatrixXd(4, 4);

  // acceleration noises. they can be set to study the effect of 
  axNoise_ = 9.0;
  ayNoise_ = 9.0;

  ekf_.init(states, stateCovMatrix, stateTransMatrix, );
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::processMeasurement(const MeasurementPackage &measurement_pack)
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

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
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

  ekf_.predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
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
