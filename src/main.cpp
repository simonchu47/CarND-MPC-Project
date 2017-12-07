#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// define the time delay
#define TIME_DELAY 0.1

// define the coefficient of MPH to MPS
// which is 1.609344*1000/60/60=0.44704
#define MPH_TO_MPS 0.44704

// define LF of the car
#define LF_OF_CAR 2.67
 
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steering = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];
          std::cout << "steering is " << steering << ", and acceleration is " << acceleration << std::endl;
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Calculate the car position after 0.1s delay
          /*          
          double px_delay = px + v*MPH_TO_MPS*cos(psi)*TIME_DELAY;
          double py_delay = py + v*MPH_TO_MPS*sin(psi)*TIME_DELAY;
          double psi_delay = psi + v*MPH_TO_MPS*steering/LF_OF_CAR*TIME_DELAY;
          double v_delay = v + acceleration*TIME_DELAY;
          */
          //Eigen::VectorXd way_x = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          //Eigen::VectorXd way_y = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

          // Coordinate transforming based on the position and heading after delay time
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for (size_t i = 0; i < ptsx.size(); i++) {
            double delta_x = ptsx[i] - px;
            double delta_y = ptsy[i] - py;
            double dist = sqrt(delta_x*delta_x + delta_y*delta_y);
            double theta = atan2(delta_y, delta_x);
            double next_x = dist*cos(theta - psi);
            double next_y = dist*sin(theta - psi);
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }
          Eigen::VectorXd way_x = Eigen::VectorXd::Map(next_x_vals.data(), next_x_vals.size());
          Eigen::VectorXd way_y = Eigen::VectorXd::Map(next_y_vals.data(), next_y_vals.size());
          
          double px_delay = v*MPH_TO_MPS*TIME_DELAY;
          double py_delay = 0;
          double psi_delay = v*MPH_TO_MPS*(-steering)/LF_OF_CAR*TIME_DELAY;
          double v_delay = v + acceleration*TIME_DELAY;

          // The polynimial is fitted to the way points
          auto coeffs = polyfit(way_x, way_y, 3);
          
          //double cte = polyeval(coeffs, 0) - 0;
          /*
          double wp_last_x = ptsx[0];
          double wp_last_y = ptsy[0];
          double wp_next_x = ptsx[1];
          double wp_next_y = ptsy[1];
          double psi_wp = atan2(wp_next_y - wp_last_y, wp_next_x - wp_last_x);
          double phi_car = atan2(wp_next_y - py, wp_next_x - px);
          double dst_wp_car = sqrt(pow(wp_next_x - px, 2) + pow(wp_next_y - py, 2));
          double cte = dst_wp_car*sin(phi_car - psi_wp);
          */
          //double slope = 3*coeffs[3]*pow(px_delay, 2) +
          //               2*coeffs[2]*px_delay +
          //               coeffs[1];
          double slope = coeffs[1];

          //double epsi = psi - atan(slope);
          double epsi = 0 - atan(slope);
          double cte = polyeval(coeffs, 0) - 0;
          double epsi_delay = epsi + v*MPH_TO_MPS*(-steering)/LF_OF_CAR*TIME_DELAY;
          double cte_delay = cte + v*MPH_TO_MPS*sin(epsi)*TIME_DELAY;

          Eigen::VectorXd state(6);
          //state << px, py, psi, v, cte, epsi;
          //state << 0, 0, 0, v_delay, cte, epsi;
          //state << px_delay, py_delay, 0, v, cte, epsi;
          state << px_delay, py_delay, psi_delay, v_delay, cte_delay, epsi_delay;

          auto vars = mpc.Solve(state, coeffs);

          std::cout << "cte = " << cte << ", slope = " << slope << std::endl;
          std::cout << "vars[0] = " << vars[0] << std::endl;
          double steer_value = -vars[0]/deg2rad(25);
          double throttle_value = vars[1];
          //double throttle_value = 0.01;
          //double steer_value = -0.99;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          size_t predict_size = (vars.size() - 2)/2;
          size_t predict_x_start = 2;
          size_t predict_y_start = predict_x_start + predict_size;
          for (size_t i = predict_x_start; i < predict_y_start; i++) {
            //mpc_x_vals.push_back(vars[i] - px);
            /*
            double delta_x = vars[i] - px;
            double delta_y = vars[i + predict_size] - py;
            double dist = sqrt(delta_x*delta_x + delta_y*delta_y);
            double theta = atan2(delta_y, delta_x);
            double mpc_x = dist*cos(theta - psi);
            double mpc_y = dist*sin(theta - psi);
            mpc_x_vals.push_back(mpc_x);
            mpc_y_vals.push_back(mpc_y);
            */
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i + predict_size]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          //vector<double> next_x_vals;
          //vector<double> next_y_vals;
          /*
          for (size_t i = 0; i < ptsx.size(); i++) {
            double delta_x = ptsx[i] - px;
            double delta_y = polyeval(coeffs, ptsx[i]) - py;
            //double delta_y = ptsy[i] - py;
            double dist = sqrt(delta_x*delta_x + delta_y*delta_y);
            double theta = atan2(delta_y, delta_x);
            double next_x = dist*cos(theta - psi);
            double next_y = dist*sin(theta - psi);
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }*/

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
