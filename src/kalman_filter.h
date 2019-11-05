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
   * @param R_laser_in Measurement covariance matrix for laser 
   * @param R_radar_in Measurement covariance matrix for radar
   * @param Q_in Process covariance matrix
   */
  //void init(const Eigen::MatrixXd &H_in, const Eigen::MatrixXd &Hj_in, const Eigen::MatrixXd &R_laser_in, 
  //  const Eigen::MatrixXd &R_radar_in, const float &axNoiseIn, const float &ayNoiseIn);

  void init(void);

  /* 
  void init(const Eigen::VectorXd &x_in, const Eigen::MatrixXd &P_in, const Eigen::MatrixXd &F_in,
            const Eigen::MatrixXd &H_in_laser, const Eigen::MatrixXd &Hj_in, const Eigen::MatrixXd &R_in, 
            const Eigen::MatrixXd &Q_in);
  */

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void predict(const float &deltaT, float axNoise, float ayNoise);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param measurements The measurement at k+1
   */
  void update(const Eigen::VectorXd &measurements, const Eigen::MatrixXd &measurementMatrix, const Eigen::MatrixXd &measurementCovMatrix);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param measurements The measurement at k+1
   */
  void updateEKF(const Eigen::VectorXd &yVector, const Eigen::MatrixXd &measurementMatrix, const Eigen::MatrixXd &measurementCovMatrix);

  // state vector, X
  Eigen::VectorXd states_;

  // state covariance matrix, P
  Eigen::MatrixXd stateCovMatrix_;

  // state transition matrix, F
  Eigen::MatrixXd stateTransMatrix_;

  // process covariance matrix, Q
  Eigen::MatrixXd processCovMatrix_;

  /*
  // measurement matrix for laser
  Eigen::MatrixXd laserMeasurementMatrix_;

  // measurement Jacobian matrix for radar
  Eigen::MatrixXd radarMeasurementMatrix_;

  // measurement covariance matrix for laser
  Eigen::MatrixXd laserMeasurementCovMatrix_;

  // measurement covariance matrix for radar
  Eigen::MatrixXd radarMeasurementCovMatrix_;

  float axNoise_;
  float ayNoise_;
  */
};

#endif // KALMAN_FILTER_H_
