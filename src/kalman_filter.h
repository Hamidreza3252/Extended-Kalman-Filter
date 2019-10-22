#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "eigen-323c052e1731/Eigen/Dense"

class KalmanFilter
{
public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param measurements The measurement at k+1
   */
  void update(const Eigen::VectorXd &measurements);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param measurements The measurement at k+1
   */
  void updateEKF(const Eigen::VectorXd &measurements);

  // state vector, X
  Eigen::VectorXd states_;

  // state covariance matrix, P
  Eigen::MatrixXd stateCovMatrix_;

  // state transition matrix, F
  Eigen::MatrixXd stateTransMatrix_;

  // process covariance matrix, Q
  Eigen::MatrixXd processCovMatrix_;

  // measurement matrix
  Eigen::MatrixXd measurementMatrix_;

  // measurement covariance matrix
  Eigen::MatrixXd measurementCovMatrix_;
};

#endif // KALMAN_FILTER_H_
