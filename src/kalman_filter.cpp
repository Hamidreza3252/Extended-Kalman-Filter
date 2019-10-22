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

void KalmanFilter::init(const Eigen::MatrixXd &H_in, const Eigen::MatrixXd &Hj_in, const Eigen::MatrixXd &R_laser_in, 
  const Eigen::MatrixXd &R_radar_in, const float &axNoiseIn, const float &ayNoiseIn)
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
  measurementMatrixLaser_ = H_in;

  // initialize measurement Jacobian matrix, Hj
  jacobianMatrixRadar_ = Hj_in;

  // initialize measurement covariance matrix for laser
  laserMeasurementCovMatrix_ = R_laser_in;

  // initialize measurement covariance matrix for radar
  laserMeasurementCovMatrix_ = R_radar_in;

  axNoise_ = axNoiseIn;
  ayNoise_ = ayNoiseIn;
}

void KalmanFilter::predict(const float &deltaT)
{
  /**
   * predict the state
   */

  stateTransMatrix_ << 1.0, deltaT, 0.0, 0.0, 
    0.0, 1.0, 0.0, 0.0, 
    0.0, 0.0, 1.0, deltaT, 
    0.0, 0.0, 0.0, 1.0;

  // X_{kp} = F * X_{k-1} + B u_{k-1} + w_k
  states_ = stateTransMatrix_ * states_;

  // P_{kp} = F * P_{k-1} * F^{T} + Q_{k}
  MatrixXd stateTransMatrixTranspose = stateTransMatrix_.transpose();
  stateCovMatrix_ = stateTransMatrix_ * stateCovMatrix_ * stateTransMatrixTranspose + processCovMatrix_;
}

void KalmanFilter::update(const VectorXd &measurements)
{
  /**
   * update the state by using Kalman Filter equations
   */

  // S = H * P_{k} * H^{T} + R_{km}
  MatrixXd measurementMatrixTranspose = measurementMatrixLaser_.transpose();
  MatrixXd sMatrix = measurementMatrixLaser_ * stateCovMatrix_ * measurementMatrixTranspose + laserMeasurementCovMatrix_;
  MatrixXd sMatrixInv = sMatrix.inverse();

  // Y_{k} = Z_{k_m} - H * X_{kp}
  VectorXd yVector = measurements - measurementMatrixLaser_ * states_;

  // K = P_{kp} * H * S^{-1}
  MatrixXd kalmanGain = stateCovMatrix_ * measurementMatrixLaser_ * sMatrixInv;

  // X_{k} = X_{kp} + K * Y_{k}
  states_ = states_ + kalmanGain * yVector;
}

void KalmanFilter::updateEKF(const VectorXd &measurements)
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
