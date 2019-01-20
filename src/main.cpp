#include "json.hpp"
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include <fstream>

// for convenience
using json = nlohmann::json;

// ***** FOR DEBUGGING ONLY - writes values to file *****
// text file for log
// std::ofstream myfile;

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
    
  // ***** FOR DEBUGGING ONLY - writes values to file *****
  //myfile.open("log.txt", std::ios::out | std::ios::app);

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
      
    // ***** FOR DEBUGGING ONLY - Display values to terminal *****
    //cout << sdata << endl;
      
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
            
          // steering angle +clock(R) -anticlock(L)
          // set current steering angle and acceleration reported from simulator
          double steer_angle = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];
            
          // ***** FOR DEBUGGING ONLY - writes values to file *****
          /*
          for (int c = 0; c < ptsx.size(); c++) {
              std::cout << "No:" << c << " ptsx:" << ptsx[c] << " ptsy:" << ptsy[c] << std::endl;
              myfile << ptsx[c] << ","<< ptsy[c] << ",";
            }
           */
            
            // ***** FOR DEBUGGING ONLY - writes values to file *****
            //myfile << px << "," << py << "," << psi << "," << v << "," << steer_angle << "," << acceleration << ",";
            // ***** FOR DEBUGGING ONLY - writes values to terminal window *****
            //std::cout << "px:" << px << " py:" << py << " psi:" << psi << " v:" << v << std::endl;
            //std::cout << "Steering Angle:" << steer_angle << " Accel:" << acceleration << std::endl;

          
          // fixed physical characteristic of vehicle - turning circle radius
          double Lf = 2.67;
          // latency of 100ms
          double dt = 0.1;
          
            
          // **** transform target path from map coordinates to vehicle coordintates ****
          // target path (ptsx,ptsy) and current vehicle position (px,py) are both in map co-ordinate system - to get a target vehicle orientated path the position of the vehicle to each point in the path needs to be calculated and then 'rotated' to be orientated in same haeding as the vehicle - this is putting it into vehicle coordinates.
          
          // variable to hold length of target path
          const int number_of_points = ptsx.size();
          // create Eigen Vectors to hold transforms
          Eigen::VectorXd ptsx_transformed(number_of_points);
          Eigen::VectorXd ptsy_transformed(number_of_points);

          // loop to transform target path from map coordinates to vehicle local coordinates
          for (int i = 0; i < number_of_points; i++) {
              
              // calculate position of ego vehicle relative to each point in target path
              // replicates target path into coordinates referenced from vehicle position
              double dx = ptsx[i] - px;
              double dy = ptsy[i] - py;
              
              double x_postion = dx * cos(psi) + dy * sin(psi);
              
              // if vehicle has not yet travelled past point on target path
              if (x_postion > 0) {
                  // add point into vector to be transformed
                  ptsx_transformed[i] = dx * cos(psi) + dy * sin(psi);
                  ptsy_transformed[i] = -dx * sin(psi) + dy * cos(psi);

              } else {
                  ptsx_transformed[i] = 0.0;
                  ptsy_transformed[i] = 0.0;

                  
              }

              // ***** FOR DEBUGGING ONLY - writes values to terminal window *****
              // std:: cout << "dx: " << dx << " dy:" << dy << "  trans x: " << ptsx_transformed[i] << " trans y: " << ptsy_transformed[i] << std::endl;
              
              // ***** FOR DEBUGGING ONLY - writes values to file *****
              //myfile << ptsx_transformed[i] << "," << ptsy_transformed[i] << ",";

          }
          
          // ***** FOR DEBUGGING ONLY - writes values to file *****
          //myfile << "end" << std::endl;
        
          // create polynominal with order of 3 to fit to points
          auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);
          
          // evaluate crosstrack error - calculate the lateral offset from target path and the vehicle rotation required to 'head' towards the target path
          // double cte = polyeval(coeffs, px) - py;
          double cte = polyeval(coeffs, 0);
          // evaluate heading error
          double epsi = - atan(coeffs[1]);
          
          // **** pass target path to model peredictive controller ****
            
          // ***** FOR REFERENCE ONLY - states algorithms *****
          /* setup states for model - standard state setup algorithms
             
            px = px + v * cos(psi) * dt;
            py = py + v * sin(psi) * dt;
            psi = psi + (v/Lf) * steer_angle * dt;
            v = v + acceleration * dt;
            cte = polyeval(coeffs, px) - py + v * sin(epsi) * dt;
            epsi = psi - atan(coeffs[1]) + (v/Lf) * steer_angle * dt;
             
          */
        
          // create a vector to holds states [x,y,psi,v,cte,epsi]
          Eigen::VectorXd state(6);
          
          // update states including latency
          // px0 = 0.0 as now transformed into car co-ordinate system
          px = 0.0 + v * cos(0) * dt;
          // py: 0 = 0.0 as now transformed into car co-ordinate system
          py = 0.0 + v * sin(0) * dt;
          // psi = 0.0 as transformed into car co-ordinate system
          psi = 0.0 + v * (-steer_angle) / Lf * dt;
          // update acceleration
          v = v + acceleration * dt;
          // update cross track error
          cte = cte + v * sin(epsi) * dt;
          // update heading
          epsi = epsi + v * (-steer_angle) / Lf * dt;
          
          // populate state vector with states
          state << px, py, psi, v, cte, epsi;
            
          // ***** FOR DEBUGGING ONLY - Display values to terminal *****
          // std::cout << "\nstate vector px:" << std::fixed << std::setprecision(6) << px << " py:" << py << " psi:"<< psi << " v:"<< v << " cte:"<< cte << " epsi:" << epsi << std::endl;
            
          // activate solver - Calculate steering angle and throttle using MPC.
          auto vars = mpc.Solve(state, coeffs);
            
          // ***** FOR DEBUGGING ONLY - Display values to terminal *****
          //std::cout << "\nSolver Output: Steering:" << vars[0] << " Throttle:" << vars [1] << std::endl;
            
          // variables to hold returned solutions
          double steer_value;
          double throttle_value;
          
          // update steering value with solver output - divide by deg2rad(25) to get steering angle
          steer_value = vars[0]/(deg2rad(25)*Lf);
          // update throttle value with solver output
          throttle_value = vars[1];

          // variable to hold simulator messages
          json msgJson;
            
          // pass target steering angle and throttle to simulator
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // populate simulator predicted trajectory (green line) from mpc results
          // displays line in green on simulator to show vehicle trajectory to get back to target path
          // loop through mpc result vetor vars and populate predicted trajectory simulator vector
          // ignore first two values as they are steering and acceleration
          for (int i = 2; i < vars.size(); i++) {
              // vector is stored as series of x,y,x,y.....
              // x values are in even positions starting from 2
              // if this position in vector is divisible by 2 - grab x position
              if (i % 2 == 0) {
                  mpc_x_vals.push_back(vars[i]);
              } else {
                  // this position must be odd in vector - so contains y postion
                  mpc_y_vals.push_back(vars[i]);
              }
          }

          // pass predicted trajectory to simulator: displayed on road as green line with interval nodes
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
            
          // Display the waypoints/reference line - yellow line in simulator
          // create vectors to store target path in road
          vector<double> next_x_vals;
          vector<double> next_y_vals;
        
          // set size of reference line vector to same size as number of points
          // push simulator target path into vector for simulator to display as yellow path
          // transformed earlier to vehicle coordinates
          // loop through transformed path
          for (int i = 0; i < ptsx_transformed.size(); i++) {
              // push x & y values from transformed path into vector whihc simulator will display
              next_x_vals.push_back(ptsx_transformed[i]);
              next_y_vals.push_back(ptsy_transformed[i]);
          }
            
          // pass target path to simulator: displayed on road as yellow line with interval nodes
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          // add all messages to simulator message
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            
          // ***** FOR DEBUGGING ONLY - Display values to terminal *****
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

          // send message containing target paht, planned trajectory, steering and accleration to simulator
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
