#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "eigen-323c052e1731/Eigen/Dense"

class MeasurementPackage
{
public:
  enum SensorType
  {
    LASER,
    RADAR
  } sensor_type_;

  long long timestamp_;

  Eigen::VectorXd raw_measurements_;
};

#endif // MEASUREMENT_PACKAGE_H_
