#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h" 

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const int MAX_HIGHWAY_LANES = 3;
const float MAX_SPEED = 49.5;  // [mph]
const float MAX_ACCELERATION = .224; // [mph^2] 
const float LANE_WIDTH = 4.0;  // [m]
const int SAFETY_GAP_FRONT = 30; // [m]


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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  int car_lane = 1; 
  double ref_vel = 0.0;
  
  
  h.onMessage([&car_lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
		  int prev_size = previous_path_x.size(); 
          if (prev_size > 0) {
              car_s = end_path_s;
           }
          json msgJson;
			
          
           
			/*
            * Prediction : Use data from sensor fusion to generate predictions 
            * about likely behavior of moving objects 
          	*/
            bool car_ahead = false;
          	bool car_left = false;
          	bool car_right = false;
        	double front_speed = 0.0; 
          	
            for(int i = 0; i < sensor_fusion.size(); i++){
              	// data format for each car is: [ id, x, y, vx, vy, s, d]
                float neigh_d = sensor_fusion[i][6];
                int neigh_lane = int(neigh_d/LANE_WIDTH);
              
                if (neigh_lane < 0 || neigh_lane>MAX_HIGHWAY_LANES) {
                  continue;
                }
               
                
                double neigh_vx = sensor_fusion[i][3];
                double neigh_vy = sensor_fusion[i][4];
                double neigh_speed = sqrt(neigh_vx*neigh_vx+ neigh_vy*neigh_vy);
                double neigh_s = sensor_fusion[i][5];
              	
              	// check where the car will be in the future 
                neigh_s += ((double)prev_size*0.02*neigh_speed);
			                         
                if ( car_lane == neigh_lane ) {
                  car_ahead |= (neigh_s > car_s) &&  (neigh_s - car_s < SAFETY_GAP_FRONT);
                  front_speed = neigh_speed; 
                } else if ( neigh_lane - car_lane == -1 ) {
                  car_left |= ((car_s - SAFETY_GAP_FRONT) < neigh_s) && ((car_s + SAFETY_GAP_FRONT) > neigh_s);
                } else if ( neigh_lane - car_lane == 1 ) {
                  car_right |= ((car_s - SAFETY_GAP_FRONT) < neigh_s) && ((car_s + SAFETY_GAP_FRONT) > neigh_s);
                }
            }
          
         /*
          * Behavior Planning  
          */
           if (car_ahead) { 
              	if (!car_left && car_lane > 0) { 
                  	car_lane --; 
                } else if (!car_right && car_lane <2){
                  	car_lane ++; 
              	} else { //if lane change is not possible 
                  	if (ref_vel > front_speed) {
						ref_vel -= MAX_ACCELERATION;
					} else {
						ref_vel = front_speed;
					}
              	}
            }
          	else {
             	// check if the vehicle is on the middle lane, if not change back to the middle lane when safe 
            	 if (car_lane != 1) { 
                	if (car_lane == 0 && !car_right || car_lane == 2 && !car_left){
                      car_lane = 1; 
                    }
                }
          
              	if (ref_vel < MAX_SPEED) {
                	ref_vel += MAX_ACCELERATION;
              	}
            }
          
          /*
           * Trajectory Generation : build candidate trajectories for the vehicle to follow 
          */
         
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          vector<double> ptsx;
          vector<double> ptsy;
          
          // reference x, y, yaw  
          double ref_x = car_x;
          double ref_y = car_y; 
          double ref_yaw = deg2rad(car_yaw);
          
          // if previous size is almost empty, starting reference is where the car is at 
          // if not use the end point from the previous path as the start point 
          if(prev_size<2){ 
            double pre_car_x = car_x - cos(car_yaw); 
            double pre_car_y = car_y - sin(car_yaw); 
            ptsx.push_back(pre_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(pre_car_y);
            ptsy.push_back(car_y);
          } 
          else{ 
            ref_x = previous_path_x [prev_size-1];
            ref_y = previous_path_y [prev_size-1];
            double prev_ref_x = previous_path_x [prev_size-2];
            double prev_ref_y = previous_path_y [prev_size-2];
            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
            
            ref_yaw =atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
          }
          
          vector<double> next_wp0 = getXY(car_s + 30, LANE_WIDTH/2 + LANE_WIDTH*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, LANE_WIDTH/2 + LANE_WIDTH*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, LANE_WIDTH/2+ LANE_WIDTH*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);  
		
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          //local car coordinates 
          for ( int i = 0; i < ptsx.size(); i++ ) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }
	
          tk::spline s;
          s.set_points(ptsx, ptsy);
          
          for (int i = 0; i < prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
           }
          
          // Calculate how to break up spline points according to the desired reference velocity 
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0;
          
          for( int i = 1; i < 50 - prev_size; i++ ) {
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);
              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;
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

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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