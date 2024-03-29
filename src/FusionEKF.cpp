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
  isInitialized_ = false;

  previousTimestamp_ = 0;

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

  timeCounter_ = 0;

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
  if (!isInitialized_)
  {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.states_ = VectorXd(4);
    ekf_.states_ << 1, 1, 1, 1;

    if (measurementPack.sensorType_ == MeasurementPackage::RADAR)
    {
      // Convert radar from polar to cartesian coordinates and initialize state.

      const float &rho = measurementPack.rawMeasurements_[0];
      const float &theta = measurementPack.rawMeasurements_[1];
      const float &rhoDot = measurementPack.rawMeasurements_[2];

      ekf_.states_ << rho * cos(theta), 
        rho * sin(theta), 
        rhoDot * cos(theta), 
        rhoDot * sin(theta);
    }
    else if (measurementPack.sensorType_ == MeasurementPackage::LASER)
    {
      // Initialize state.

      ekf_.states_ << measurementPack.rawMeasurements_[0], 
        measurementPack.rawMeasurements_[1], 
        0.0, 
        0.0;
    }

    previousTimestamp_ = measurementPack.timestamp_;

    // done initializing, no need to predict or update
    isInitialized_ = true;

    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float deltaT = (measurementPack.timestamp_ - previousTimestamp_) / 1e6;
  previousTimestamp_ = measurementPack.timestamp_;

  ekf_.predict(deltaT, axNoise_, ayNoise_);

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  const float &x = ekf_.states_[0];
  const float &y = ekf_.states_[1];
  const float &xDot = ekf_.states_[2];
  const float &yDot = ekf_.states_[3];

  // Eigen::VectorXd mappedStates;
  Eigen::MatrixXd measurementMatrix;
  Eigen::MatrixXd measurementCovMatrix;
  float jacobianTol = 1e-4;

  // https://github.com/udacity/self-driving-car-sim/releases

  timeCounter_++;
  Eigen::VectorXd yVector;

  if (measurementPack.sensorType_ == MeasurementPackage::RADAR)
  {
    // Radar updates
    ekf_.mappedStates_ = Eigen::VectorXd(3);

    float commonTerm = sqrt(x*x + y*y);

    ekf_.mappedStates_ << commonTerm, 
      atan2(y, x), 
      (x * xDot + y * yDot) / commonTerm;

    // Y_{k} = Z_{k_m} - h( X_{kp} )
    yVector = measurementPack.rawMeasurements_ - ekf_.mappedStates_;
    yVector(1) = tools.fixAngle(yVector(1));
    
    measurementMatrix = tools.calculateJacobian(ekf_.states_, jacobianTol);
    measurementCovMatrix = covMatrixRadar_;
  }
  else
  {
    // Laser updates

    ekf_.mappedStates_ = laserMeasurementMatrix_ * ekf_.states_;
    // Y_{k} = Z_{k_m} - H * X_{kp}
    yVector = measurementPack.rawMeasurements_ - ekf_.mappedStates_;
    measurementMatrix = laserMeasurementMatrix_; 
    measurementCovMatrix = covMatrixLaser_;
  }

  ekf_.updateEKF(yVector, measurementMatrix, measurementCovMatrix);

  // print the output
  // cout << "x_ = " << ekf_.states_ << endl;
  // cout << "P_ = " << ekf_.stateCovMatrix_ << endl;
}

void FusionEKF::writeResultsToFile(const MeasurementPackage &measurementPack, const Eigen::VectorXd &groundTruths)
{
  std::string sensorTypeString;
  std::string phiString;

  if (measurementPack.sensorType_ == MeasurementPackage::RADAR)
  {
    sensorTypeString = "R";
    phiString = std::to_string(ekf_.mappedStates_[1]);
  }
  else
  {
    sensorTypeString = "L";
    phiString = " ";
  }

  tools.outputFile_ << sensorTypeString
    << "," << ekf_.states_[0] 
    << "," << ekf_.states_[1] 
    << "," << ekf_.states_[2] 
    << "," << ekf_.states_[3]

    << "," << phiString

    << "," << rmseVector_[0] 
    << "," << rmseVector_[1] 
    << "," << rmseVector_[2] 
    << "," << rmseVector_[3]

    << "," << groundTruths[0] 
    << "," << groundTruths[1] 
    << "," << groundTruths[2] 
    << "," << groundTruths[3]

    << "\n";
}
