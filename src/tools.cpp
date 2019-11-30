#include "tools.h"
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools()
{
	openResultsLogFile("results", "results.txt");
}

Tools::~Tools()
{
	closeResultsLogFile();
}

// ====================================================================================================================
void Tools::openResultsLogFile(const std::string &filePath, const std::string &fileName)
{
	int status = mkdir(filePath.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);

	if (outputFile_.is_open())
	{
		outputFile_.close();
	}

	const std::string fullFileName = filePath + "/" + fileName;
	
	std::ios_base::iostate exceptionMask = outputFile_.exceptions() | std::ios::failbit;
	outputFile_.exceptions(exceptionMask);

	try
	{
		outputFile_.open(fullFileName, std::fstream::out);
		// outputFile_.open(fileName, std::ios::out);

		if (!outputFile_.is_open())
		{
			std::cerr << "Could not open file " << fullFileName << " to write the results \n";

			return;
		}
	}
	catch (const std::ios_base::failure &e)
	{
		std::cout << e.code().value() << std::endl;
		std::cout << e.code().message() << std::endl;
		closeResultsLogFile();
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << '\n';
		closeResultsLogFile();
	}
}

// ====================================================================================================================
void Tools::closeResultsLogFile(void)
{
	if (outputFile_)
	{
		outputFile_.close();
	}
}

// ====================================================================================================================
VectorXd Tools::calculateRMSE(const vector<VectorXd> &estimations,
							  const vector<VectorXd> &groundTruths)
{
	/**
   * Calculate the RMSE here.
   */

	VectorXd incrementalErrors;
	Eigen::VectorXd rmseVector = Eigen::VectorXd::Zero(4);

	if (estimations.size() != groundTruths.size() || estimations.size() == 0)
	{
		std::cout << "Invaid estimation or groundtruth data" << std::endl;

		rmseVector = VectorXd::Ones(4) * -1.0;

		return rmseVector;
	}

	for (unsigned int i = 0; i < estimations.size(); i++)
	{
		incrementalErrors = groundTruths[i] - estimations[i];

		// coefficient-wise multiplication
		rmseVector = rmseVector.array() + incrementalErrors.array() * incrementalErrors.array();
	}

	rmseVector = (rmseVector / estimations.size()).array().sqrt();

	return rmseVector;
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

float Tools::fixAngle(const float &angle)
{
	// float aa = fmod(angle + M_PI, 2.0);

	return fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}
