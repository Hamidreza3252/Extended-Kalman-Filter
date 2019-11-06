#ifndef TOOLS_H_
#define TOOLS_H_

#include <fstream>
#include <vector>
#include "eigen-323c052e1731/Eigen/Dense"

class Tools
{
public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  std::ofstream outputFile_;

  void openResultsLogFile(const std::string &fileName);
  void closeResultsLogFile(void);

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                const std::vector<Eigen::VectorXd> &groundTruths);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd calculateJacobian(const Eigen::VectorXd &states, float tol);

  float fixAngle(const float &angle);
};

#endif // TOOLS_H_
