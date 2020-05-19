#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"

#define TOT_NUM_STEPS 2000

// for convenience
using nlohmann::json;
using namespace std;

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
  Twiddle tw;
  double kp_init;
  double ki_init;
  double kd_init;
  unsigned long step_counter = 0;
  bool sim_reset = false;

  kp_init = 0.2;
  ki_init = 0.004;
  kd_init = 3.0;
  /* Initializations of Twiddle and PID Instances */
  pid.Init(kp_init, ki_init,kd_init);
  tw.Init(kp_init, ki_init,kd_init);

  h.onMessage([&pid, &tw, &step_counter, &kp_init, &ki_init, &kd_init, &sim_reset](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          double steer_value = 0.0;
          
          std::cout<<"Current step is:"<< step_counter << std::endl;
          if( sim_reset == false ) {
            /* Needed for the case of internal Simulator Resets
             * while performing hyper-parameter tuning
             */
            if( ( step_counter  == 0 ) )
            {
              pid.ResetError();
            }
            
            //update the error 
            pid.UpdateError(cte);
            tw.SetError(cte);
            //Increment the time-step counter
            step_counter++;

            /* Modify the Hyper params for the next iteration */
            tw.HypTuning(step_counter,false);

            /* Set the hyperparams of PID to the modified values */
            pid.SetTwiddleParams(tw);
              
            //tw.PrintDPParams();
            if( ( step_counter == TOT_NUM_STEPS) )
            {
                //This setting acts as a baseline to start Twiddling hyp param tuning.
                tw.SetBestError(TOT_NUM_STEPS);

                /* Modify the Hyper params for the next iteration */
                int res = tw.HypTuning(step_counter,true);

                /* Set the hyperparams of PID to the modified values via Twiddle */
                pid.SetTwiddleParams(tw);
                
                //tw.PrintDPParams();
                //tw.PrintBestParams();
                
                /* This occurs when sum of dp params is less than threshold in which case
                 * stop tuning
                 */
                if( res == 0 )
                {
                 cout<<"Tuning Done \n";
                  //tw.PrintDPParams();
                  /* Reset the count, the current error and Reset the simulator now */
                 cout<<"Reset Simulator \n";
                 step_counter = 0;
                 sim_reset = true;
                 std::string msg = "42[\"reset\",{}]";
                 ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                 exit(0);
                }
              }
          }
          //calculate steering value
          steer_value = pid.TotalError();
          if( steer_value > 1 ) steer_value = 1.0;
          else if( steer_value < -1 ) steer_value = -1.0;
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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