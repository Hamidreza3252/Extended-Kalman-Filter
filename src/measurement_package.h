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
  } sensorType_;

  long long timestamp_;

  Eigen::VectorXd rawMeasurements_;
};

#endif // MEASUREMENT_PACKAGE_H_
