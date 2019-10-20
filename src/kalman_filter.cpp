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
   * TODO: predict the state
   */
}

void KalmanFilter::update(const VectorXd &measurements)
{
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::updateEKF(const VectorXd &measurements)
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
