#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter()
{

}

KalmanFilter::~KalmanFilter()
{

}

// void KalmanFilter::init(const Eigen::MatrixXd &H_in, const Eigen::MatrixXd &Hj_in, const Eigen::MatrixXd &R_laser_in, 
//  const Eigen::MatrixXd &R_radar_in, const float &axNoiseIn, const float &ayNoiseIn)

void KalmanFilter::init(void)
{
  /*
  states_ = x_in;
  stateCovMatrix_ = P_in;
  stateTransMatrix_ = F_in;
  measurementMatrixLaser_ = H_in;
  jacobianMatrixRadar_ = Hj_in;
  measurementCovMatrix_ = R_in;
  processCovMatrix_ = Q_in;
  */

  // initial state vector X
  states_ = VectorXd(4);
  states_ << 0.0, 0.0, 0.0, 0.0;

  // initial state covariance matrix P
  stateCovMatrix_ = MatrixXd(4, 4);
  stateCovMatrix_ << 1.0, 0.0, 0.0, 0.0, 
    0.0, 1.0, 0.0, 0.0, 
    0.0, 0.0, 1000.0, 0.0, 
    0.0, 0.0, 0.0, 1000.0;

  // initialize state transition matrix, F
  stateTransMatrix_ = MatrixXd(4, 4);
  stateTransMatrix_ << 1.0, 0.0, 0.0, 0.0, 
    0.0, 1.0, 0.0, 0.0, 
    0.0, 0.0, 1.0, 0.0, 
    0.0, 0.0, 0.0, 1.0;

  // initialize process covariance matrix, Q
  processCovMatrix_ = MatrixXd(4, 4);

  // initialize measurement matrix, H
  // laserMeasurementMatrix_ = H_in;

  // initialize measurement Jacobian matrix, Hj
  // radarMeasurementMatrix_ = Hj_in;

  // initialize measurement covariance matrix for laser
  // laserMeasurementCovMatrix_ = R_laser_in;

  // initialize measurement covariance matrix for radar
  // laserMeasurementCovMatrix_ = R_radar_in;

  // axNoise_ = axNoiseIn;
  // ayNoise_ = ayNoiseIn;
}

void KalmanFilter::predict(const float &deltaT, float axNoise, float ayNoise)
{
  /**
   * predict the state
   */

  // updating state transition matrix 
  stateTransMatrix_ << 1.0, 0.0, deltaT, 0.0, 
    0.0, 1.0, 0.0, deltaT, 
    0.0, 0.0, 1.0, 0.0, 
    0.0, 0.0, 0.0, 1.0;

  // updating process covariance matrix 
  float dt2 = deltaT * deltaT;
  float dt3 = dt2 * deltaT;
  float dt4 = dt3 * deltaT;

  processCovMatrix_ << 0.25 * dt4 * axNoise, 0.0, 0.5 * dt3 * axNoise, 0.0, 
    0.0, 0.25 * dt4 * ayNoise, 0.0, 0.5 * dt3 * ayNoise, 
    0.0, 0.0, dt2 * axNoise, 0.0, 
    0.0, 0.0, 0.0, dt2 * ayNoise;
  processCovMatrix_(2, 0) = processCovMatrix_(0, 2);
  processCovMatrix_(3, 1) = processCovMatrix_(1, 3);

  // X_{kp} = F * X_{k-1} + B u_{k-1} + w_k
  states_ = stateTransMatrix_ * states_;

  // P_{kp} = F * P_{k-1} * F^{T} + Q_{k}
  MatrixXd stateTransMatrixTranspose = stateTransMatrix_.transpose();
  stateCovMatrix_ = stateTransMatrix_ * stateCovMatrix_ * stateTransMatrixTranspose + processCovMatrix_;
}

void KalmanFilter::update(const VectorXd &measurements, const MatrixXd &measurementMatrix, const MatrixXd &measurementCovMatrix)
{

}

//void KalmanFilter::updateEKF(const VectorXd &measurements, const VectorXd &mappedStates, 
//  const MatrixXd &measurementMatrix, const MatrixXd &measurementCovMatrix)
void KalmanFilter::updateEKF(const VectorXd &yVector, const MatrixXd &measurementMatrix, const MatrixXd &measurementCovMatrix)
{
  /**
   * update the state by using Extended Kalman Filter equations
   */

  // laserMeasurementMatrix_ = H_in;
  // radarMeasurementMatrix_ = Hj_in;
  // laserMeasurementCovMatrix_ = R_laser_in;
  // laserMeasurementCovMatrix_ = R_radar_in;

  // S = H * P_{k} * H^{T} + R
  // S = Hj * P_{k} * Hj^{T} + R
  MatrixXd measurementMatrixTranspose = measurementMatrix.transpose();
  MatrixXd sMatrix = measurementMatrix * stateCovMatrix_ * measurementMatrixTranspose + measurementCovMatrix;
  MatrixXd sMatrixInv = sMatrix.inverse();

  // K = P_{kp} * H * S^{-1}
  // K = P_{kp} * Hj * S^{-1}
  MatrixXd kalmanGain = stateCovMatrix_ * measurementMatrixTranspose * sMatrixInv;

  // X_{k} = X_{kp} + K * Y_{k}
  states_ += kalmanGain * yVector;

  // P_{k} = (I - K * H) * P{kp}
  // P_{k} = (I - K * Hj) * P{kp}
  stateCovMatrix_ = (MatrixXd::Identity(4, 4) - kalmanGain * measurementMatrix) * stateCovMatrix_;
}
