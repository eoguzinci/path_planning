#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"

#include "helpers.h"
#include "planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // lane IDs: left: [0], middle [1], right: [2] 
  // starts in the middle lane
  int lane = 1;

  // reference velocity to target is as close as the speed limit of 50 mph
  double ref_vel = 49.5;

  // number of waypoints
  const int num_points = 50;

  // period of refreshing - the vehicle visits a point at each sampling
  const double sampling = 0.02;

  // lane size in meters
  const double lane_size = 4;

  // maximum acceleration
  const double max_acc = 0.224;

  // speed of the car_ahead
  static double car_ahead_speed = numeric_limits<double>::max();

  // counter
  static unsigned int counter = 0;

  // reaction time
  const int reaction_time = 2;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &num_points, &sampling, &lane_size, &max_acc, &car_ahead_speed
    , &counter, & reaction_time](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            json msgJson;
            
            int prev_size;

            if ( previous_path_x.size() != previous_path_y.size() ) // !! Division by zero
              throw "previous data has different size";
            else
              prev_size = previous_path_x.size();

            if(prev_size > 0){
              car_s = end_path_s;
            }

            bool too_close = false;

            // PERCEPTION
            // to check if we have a car ahead in the same lane
            // we need to go through the sensor_fusion list
            // sensor_fusion list holds [ id, x, y, vx, vy, s, d] for each element
            bool car_ahead = false;
            bool left_go = true;
            bool right_go = true;
            for (int i = 0; i < sensor_fusion.size(); i++)
            {
              float d = sensor_fusion[i][6]; // to determine the lane of the other car
              
              int check_lane;

              if ( d > 0 && d < lane_size ){
                check_lane = 0;
              }else if( d > lane_size && d < 2 * lane_size ){
                check_lane = 1;
              } else if ( d > lane_size && d < 2 * lane_size ){
                check_lane = 2;
              }else{
                check_lane = -2;
              }

              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy+vy);
              double check_s = sensor_fusion[i][5];

              // if the car is ahead and distance is lower than 30m.
              // lower the velocity 
              check_s += (double)prev_size * sampling * check_speed;             
              if( check_lane == lane )
              {
                if( check_s > car_s ){
                  // Do some logic here to lower the velocity lower reference velocity
                  car_ahead = true;
                  car_ahead_speed = check_speed;
                  counter++ 
                  // ref_vel = check_speed; //mph
                  // 2 s. response time 
                  if ( check_s - car_s < mph2ms(car_speed)*reaction_time - mph2ms(max_acc)*pow(reaction_time,2)/2){
                    too_close = true;
                  }
                }
              }
              if ( check_lane = lane-1)
              {
                if(check_s >= car_s){
                  left_go = false;
                } else{
                  if (check_speed > car_speed && car_s - check-s > (check_speed - car_speed) * reaction_time){
                    left_go = false;
                  }
                }
              }
              if ( check_lane = lane+1)
              {
                if(check_s >= car_s){
                  right_go = false;
                } else{
                  if (check_speed > car_speed && car_s - check-s > (check_speed - car_speed) * reaction_time){
                    right_go = false;
                  }
                }
              }
            }

            // ACTION
            if(!car_ahead)
              counter = 0;

            int patience_time = 10;

            // if we are in middle lane
            if(lane == 1){
              if(too_close){
                if( counter > patience_time)
                  if(left_go)
                    lane--;
                  else if(right_go)
                    lane++;
                  else{
                    if (car_speed > car_ahead_speed)
                      ref_vel -= max_acc;
                  }
              }
              else if( ref_vel < 49.5){
                ref_vel += max_acc;
              }
            }

            // if we are in the right lane
            if(lane == 2){
              if(too_close){
                if( counter > patience_time){
                  if(left_go){
                    lane--;
                  } else {
                    if (car_speed > car_ahead_speed)
                      ref_vel -= max_acc;
                    else
                      lane--
                  }
                }
              }
              else if( ref_vel < 49.5){
                ref_vel += max_acc;
              } 
            }

            // if we are in the left lane
            // if we are in the right lane
            if(lane == 0){
              if(too_close){
                ref_vel -= max_acc;
                if( counter > patience_time){
                  if(right_go){
                    lane = lane-1;
                  } else {
                    if (car_speed > car_ahead_speed)
                      ref_vel -= max_acc;
                    else
                      lane--
                  }
                }
              }
              else if( ref_vel < 49.5){
                ref_vel += max_acc;
              } 
            }

            // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // Later we will interpolate these points with a spline and fill it with more points that control the speed.
            vector<double> ptsx, ptsy;

            // reference x,y, yaw states
            // either we will reference the starting point as where the car is or at the previous paths and point
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // Making the path tangent to the previous path - C1-continuity
          	// if the previous size is almost empty, use the car as starting reference
            if (prev_size < 2)
            {
              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);

            } else {
              // Redefine reference states as previous path endpoint
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);

            }

            // In FreNet add evenly 30m spaced points ahead of the starting reference
            vector<double> next_wp0 = getXY(car_s + 30, lane_size/2+lane_size*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, lane_size/2+lane_size*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, lane_size/2+lane_size*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // Transform the waypoints with respect to the local car coordinates
            // https://en.wikipedia.org/wiki/Rotation_of_axes
            for (int i = 0; i < ptsx.size(); i++)
            {
              // shift car reference angle to 0 degrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
            }

            // create a spline
            tk::spline s;

            // set (x,y) points to the spline
            s.set_points(ptsx,ptsy);

            // define the actual (x,y) points we will use for the planner
            vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // let's start with all the previous path points from last time
            for (int i = 0; i < prev_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // NOTE: previous_path_x does not have to be 50 points but what is not eaten by the car.
            // if car has eaten 3 points already in the previous path the remaining 47 points is the 
            // previous path points.

            //Calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x * target_x + target_y *target_y);


            // Fill up the rest of the planner after filling it with previous points, here we will always output 50 points
            double x_add_on = 0.0;
            for (int i = 1; i <= num_points - prev_size; i++)
            {
              double N = target_dist / (sampling * mph2ms(ref_vel));
              double x_point = x_add_on + target_x / N; 
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // Inverse transformation - from local to global coordinates
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
