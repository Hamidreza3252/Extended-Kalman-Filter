#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools()
{
}

Tools::~Tools()
{
}

VectorXd Tools::calculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &groundTruths)
{
   /**
   * Calculate the RMSE here.
   */

   VectorXd rsmeVector = Eigen::VectorXd::Zero(4);

   if (estimations.size() != groundTruths.size() || estimations.size() == 0)
   {
      std::cout << "Invaid estimation or groundtruth data" << std::endl;

      rsmeVector = VectorXd::Ones(4) * -1.0;

      return rsmeVector;
   }

   for (unsigned int i = 0; i < estimations.size(); i++)
   {
      VectorXd residuals = estimations[i] - groundTruths[i];

      // coefficient-wise multiplication
      residuals = residuals.array() * residuals.array();
      rsmeVector += residuals;
   }

   rsmeVector = (rsmeVector / estimations.size()).array().sqrt();

   return rsmeVector;
}

MatrixXd Tools::calculateJacobian(const VectorXd &states, float tol)
{
   /**
   * Calculate a Jacobian here.
   */
   // MatrixXd jacobianMatrix = MatrixXd(3, 4);
   MatrixXd jacobianMatrix = MatrixXd::Zero(3, 4);

   const float &x = states(0);
   const float &y = states(1);
   const float &vx = states(2);
   const float &vy = states(3);

   float denomBase = x * x + y * y;

   if (fabs(denomBase) < tol)
   {
      std::cout << "Jacobian matrix error - division by zero" << std::endl;

      return jacobianMatrix;
   }

   float denomBase_1_2 = sqrt(denomBase);
   float denomBase_3_2 = denomBase * denomBase_1_2;

   jacobianMatrix << x / denomBase_1_2, y / denomBase_1_2, 0.0, 0.0,
       -y / denomBase, x / denomBase, 0.0, 0.0,
       y * (vx * y - vy * x) / denomBase_3_2, x * (vy * x - vx * y) / denomBase_3_2, x / denomBase_1_2, y / denomBase_1_2;

   return jacobianMatrix;
}
