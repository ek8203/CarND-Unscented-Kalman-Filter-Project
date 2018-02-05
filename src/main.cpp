#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"
#include <fstream>
#include <string>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, const char * argv[])
{
  uWS::Hub h;

  // Create a Kalman Filter instance
  UKF ukf;

  // Get command string parameters
  if(argc > 1) {
    try {
      ukf.std_a_ = stof(argv[1]);
    } catch (...)  {
      cout << "p1" << endl;
      return -1;
    }
  }

  if(argc > 2) {
    try {
      ukf.std_yawdd_ = stof(argv[2]);
    } catch (...)  {
      cout << "p2" << endl;
      return -1;
    }
  }


  if (argc > 3)
  {
    try {
      if(stoi(argv[3]) > 0) {
        ukf.use_laser_ = true;
      }
      else  {
        ukf.use_laser_ = false;
      }
    } catch (...)  {
      cout << "p3" << endl;
      return -1;
    }
  }

  if(argc > 4) {
    try {
      if(stoi(argv[4]) > 0) {
        ukf.use_radar_ = true;
      }
      else  {
        ukf.use_radar_ = false;
      }
    } catch (...)  {
      cout << "p4" << endl;
      return -1;
    }
  }

  cout << "std_a=" << ukf.std_a_ << endl;
  cout << "std_yawdd=" << ukf.std_yawdd_ << endl;
  cout << "use_laser=" << ukf.use_laser_ << endl;
  cout << "use_radar=" << ukf.use_radar_ << endl;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // Save results in a file
  ofstream fout;
  const string  fname = "../data/ukf_output.txt";

  fout.open(fname);

  if (!fout.is_open()) {
    cout << "Unable to open output file " << fname << endl;
    return -1;
  }

  h.onMessage([&ukf,&tools,&estimations,&ground_truth, &fout](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
          long long timestamp;

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type.compare("L") == 0) {
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;
          		meas_package.raw_measurements_ = VectorXd(2);
          		float px;
      	  		float py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		float ro;
      	  		float theta;
      	  		float ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
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
          
          //Call ProcessMeasurment(meas_package) for Kalman filter
          ukf.ProcessMeasurement(meas_package);

          //Push the current estimated x,y positon from the Kalman filter's state vector

          VectorXd estimate(4);

          double p_x = ukf.x_(0);
          double p_y = ukf.x_(1);
          double v  = ukf.x_(2);
          double yaw = ukf.x_(3);

          double v1 = cos(yaw)*v;
          double v2 = sin(yaw)*v;

          // save results
          // Output file format: est_px est_py est_vx est_vy meas_px meas_py gt_px gt_py gt_vx gt_vy
          // est_px est_py est_vx est_vy
          fout << p_x << "\t" << p_y << "\t" << v1 <<"\t" << v2 << "\t";

          // meas_px meas_py
          float meas_px,
                meas_py;
          if (sensor_type.compare("L") == 0) {
            meas_px = meas_package.raw_measurements_[0];
            meas_py = meas_package.raw_measurements_[1];
          }
          else if (sensor_type.compare("R") == 0) {
            meas_px = meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]);
            meas_py = meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]);
          }
          fout << meas_px << "\t"<< meas_py << "\t";

          // gt_px gt_py gt_vx gt_vy
          fout << x_gt << "\t" << y_gt << "\t" << vx_gt << "\t" << vy_gt << "\t";

          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;
    	  
          estimations.push_back(estimate);

          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
          cout << "Accuracy - RMSE:" << endl << RMSE << endl;

          // RMSE(0) RMSE(1) RMSE(2) RMSE(3)
          fout << RMSE(0) << "\t" << RMSE(1) << "\t" << RMSE(2) << "\t" << RMSE(3) << "\t";

          // NIS_lidar NIS_radar
          fout << ukf.NIS_lidar_ << "\t" << ukf.NIS_radar_ << endl;
          cout << "Lidar NIS: " << ukf.NIS_lidar_ << endl;
          cout << "Radar NIS: " << ukf.NIS_radar_ << endl;

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
    fout.close();
    return -1;
  }
  h.run();

  fout.close();
  cout <<"Results saved in file " << fname << endl;
}























































































