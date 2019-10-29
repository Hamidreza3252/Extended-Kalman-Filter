#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

// #include "Dense"
#include "Dense"

class MeasurementPackage
{
public:
    enum SensorType
    {
        LASER,
        RADAR
    } sensorType_;

    Eigen::VectorXd rawMeasurements_;

    int64_t timestamp_;
};

#endif // MEASUREMENT_PACKAGE_H_