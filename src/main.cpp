#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;

// for convenience
using json = nlohmann::json;

// ====================================================================================================================
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// ====================================================================================================================
int main()
{
  enum InputDataSource
  {
    SOCKET,
    FILE
  } inputDataSource;

  inputDataSource = InputDataSource::SOCKET;

  uWS::Hub h;

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&fusionEKF, &tools, &estimations, &ground_truth, &inputDataSource](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data));

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          MeasurementPackage measurementPack;

          // j[1] is the data JSON object
          string sensor_measurement = j[1]["sensor_measurement"];

          std::istringstream iss(sensor_measurement);

          long long timestamp;

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type.compare("L") == 0)
          {
            measurementPack.sensorType_ = MeasurementPackage::LASER;
            measurementPack.rawMeasurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            measurementPack.rawMeasurements_ << px, py;
            iss >> timestamp;
            measurementPack.timestamp_ = timestamp;

            /*
            if(fusionEKF.timeCounter_ > 0)
            {
              string msg = "42[\"manual\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

              return;
            }
            */
          }
          else if (sensor_type.compare("R") == 0)
          {
            measurementPack.sensorType_ = MeasurementPackage::RADAR;
            measurementPack.rawMeasurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            measurementPack.rawMeasurements_ << ro, theta, ro_dot;
            iss >> timestamp;
            measurementPack.timestamp_ = timestamp;

            // string msg = "42[\"manual\",{}]";
            // ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            // return;
          }

          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;

          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt;
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          ground_truth.push_back(gt_values);

          bool skipOneSensorType = false;

          if(skipOneSensorType)
          {
            // Uncomment it to skip either Radar or Lidar for Studying purposes
            if(!fusionEKF.isInitialized_)
            {
              fusionEKF.processMeasurement(measurementPack);
            }

            if (sensor_type.compare("L") == 0)
            {
              fusionEKF.processMeasurement(measurementPack);
            }
          }
          else
          {
            // Call ProcessMeasurement(meas_package) for Kalman filter
            fusionEKF.processMeasurement(measurementPack);
          }

          // Push the current estimated x,y positon from the Kalman filter's
          //   state vector

          VectorXd estimate(4);

          double p_x = fusionEKF.ekf_.states_(0);
          double p_y = fusionEKF.ekf_.states_(1);
          double v1 = fusionEKF.ekf_.states_(2);
          double v2 = fusionEKF.ekf_.states_(3);

          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;

          estimations.push_back(estimate);

          // VectorXd RMSE = tools.calculateRMSE(estimations, ground_truth);
          fusionEKF.rmseVector_ = tools.calculateRMSE(estimations, ground_truth);

          std::cout << "Accuracy RMSE: " << fusionEKF.rmseVector_ << std::endl;

          fusionEKF.writeResultsToFile(measurementPack, gt_values);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] = fusionEKF.rmseVector_(0);
          msgJson["rmse_y"] = fusionEKF.rmseVector_(1);
          msgJson["rmse_vx"] = fusionEKF.rmseVector_(2);
          msgJson["rmse_vy"] = fusionEKF.rmseVector_(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        } // end "telemetry" if
      }
      else
      {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}