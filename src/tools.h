#ifndef TOOLS_H_
#define TOOLS_H_

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

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd calculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                const std::vector<Eigen::VectorXd> &groundTruths);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd jacobian(const Eigen::VectorXd &states, float tol);
};

#endif // TOOLS_H_
