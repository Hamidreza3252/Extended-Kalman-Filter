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

void KalmanFilter::init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  states_ = x_in;
  stateCovMatrix_ = P_in;
  stateTransMatrix_ = F_in;
  measurementMatrix_ = H_in;
  measurementCovMatrix_ = R_in;
  processCovMatrix_ = Q_in;
}

void KalmanFilter::predict()
{
  /**
   * predict the state
   */

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
  MatrixXd measurementMatrixTranspose = measurementMatrix_.transpose();
  MatrixXd sMatrix = measurementMatrix_ * stateCovMatrix_ * measurementMatrixTranspose + measurementCovMatrix_;
  MatrixXd sMatrixInv = sMatrix.inverse();

  // Y_{k} = Z_{k_m} - H * X_{kp}
  VectorXd yVector = measurements - measurementMatrix_ * states_;

  // K = P_{kp} * H * S^{-1}
  MatrixXd kalmanGain = stateCovMatrix_ * measurementMatrix_ * sMatrixInv;

  // X_{k} = X_{kp} + K * Y_{k}
  states_ = states_ + kalmanGain * yVector;
}

void KalmanFilter::updateEKF(const VectorXd &measurements)
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
