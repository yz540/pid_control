#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   *  Initialize the pid variable.
   */
//   double p[3] = {0.2, 0.0001, 1.5};
  double p[3] = {0.186248, 0.0001, 1.49};
  pid.Init(p[0], p[1], p[2]);
  double dp[3] = {.01, .001, .1};

  bool twiddle = false;
  double tol = 0.01;
  int max_step = 500;
  int n = 0;
  int p_it = 0; 
  double best_cte = 100000;
  double total_cte = 0;
  double sum_dp = dp[0] + dp[1] + dp[2];
  bool up_run = true;
  bool init = true;
  h.onMessage([twiddle, &init, &sum_dp, &pid, &p, &dp, &p_it, &n, &total_cte, max_step, tol, &up_run, &best_cte ](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                                                                                                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          json msgJson;
          msgJson["throttle"] = 0.3;
          /**
           * Calculate steering value here, remember the steering value is
           *   [-1, 1].
           *
           */
          //           Find the best parameters
          if(twiddle){
            if(sum_dp > tol){
//               std::cout <<"n is..." <<  n << std::endl;
              // Initialize the parameters and car locations for the following max step simulation
              if(p_it < 3){
                if(init){
                  p[p_it] += dp[p_it];
                  pid.Init(p[0], p[1], p[2]);

                  // reset car position
                  std::string reset_msg = "42[\"reset\",{}]";
                  std::cout <<"Initialize for p <<<<<<<<<<<<<<<<<<<<<<<" <<  p[0] << p[1] << p[2]<< std::endl;
                  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                  init = false;
                  up_run = true;
                  n = 0;
                  total_cte = 0;
                }
                if(n < max_step){
                  // run with the same parameters for max_steps and calculate their overall cte
                  total_cte += pow(cte,2);
                  n += 1;
                  pid.UpdateError(cte);
                  steer_value = pid.TotalError();
                  msgJson["steering_angle"] = steer_value;
                  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                  //                 std::cout << msg << std::endl;
                  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                } else { // finished the calculation of err of max steps
                  total_cte /= max_step;
                  std::cout << "total cte is: " << total_cte << std::endl;
                  // find the best cte
                  if(total_cte < best_cte){
                    best_cte = total_cte;
                    std::cout << "best cte is: " << best_cte << std::endl;
                    dp[p_it] *= 1.1;
                    p_it += 1;
                    init = true;
                  }else {
                    std::cout << "up run " << up_run << std::endl;
                    if(up_run){
                      p[p_it] -= 2 * dp[p_it];
                      n = 0;
                      total_cte = 0;
                      up_run = false;
                    }else{
                      p[p_it] += dp[p_it];
                      dp[p_it] *= 0.9;
                      p_it += 1;
                      init = true;
                    }
                  }
                  // reset car position
                  std::string reset_msg = "42[\"reset\",{}]";
                  std::cout << "p[0] p[1] p[2] after: " << p[0] << p[1] << p[2] << " " << std::endl;
                  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                }
              } else{ // if(p_it < 3)
                p_it = 0;
                init = true;
              }
              sum_dp = dp[0]+dp[1]+dp[2];
              std::cout << "dp sum is " << dp[0]+dp[1]+dp[2] << std::endl;    
            }else{ //if(sum_dp > tol)
              std::cout << "Best p[0] p[1] p[2]: " << p[0] << p[1] << p[2] << " " << std::endl;
              std::string reset_msg = "42[\"reset\",{}]";
              std::cout << reset_msg << std::endl;
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

            } 
          }else{ // twiddle if
            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
              << std::endl;
            pid.Init(p[0], p[1], p[2]);
            pid.UpdateError(cte);
            steer_value = pid.TotalError();
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          } // // end twiddle if
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}