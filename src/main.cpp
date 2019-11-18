#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

/**
 * helper function for printing a vector
 * */
void print(vector<double> const &input)
{
	for (int i = 0; i < input.size(); i++) {
		std::cout << input.at(i) << ' ' << std::flush;
	}
}

/**
 * Functions returns a double value which 
 * is subtructed from the lane_cost.
 * 
**/
double laneCost(vector<vector<double>> const sensor_fusion,double prev_path_size, double end_path_s,int lane){
  double distAway = 999;
  for(int i = 0; i < sensor_fusion.size(); i++){
    
    float d = sensor_fusion[i][6];
    // std::cout<<"checking lane"<<d<<std::flush<<std::endl;
    
    if(d< (2+4*lane+2) && d > (2+4*lane-2)){
    std::cout<<"peek lane"<<d<<std::flush<<std::endl;
      //get car's velocity
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_car_s = sensor_fusion[i][5];
      double check_speed = sqrt(vx*vx+vy*vy);

      // how close will the car be in our future path?
      check_car_s+=(double)prev_path_size*.02*check_speed;
      double distance_from_other_cars = check_car_s - end_path_s;
      // we don't want to be less than 30meters close to other cars
      if (abs(distance_from_other_cars) > 30){
        // 30m clear lane
        std::cout<<"lane clear. nearest vehicle "<<distance_from_other_cars<<std::flush<<std::endl;
        distAway += check_speed*abs(distance_from_other_cars); // speed*distance_away to help pick fastest moving lane
      }else{ // else object in lane
        std::cout<<"occupied lane. nearest car "<<distance_from_other_cars<<std::flush<<std::endl;
        distAway = 0.0;
        break;
      }
    }
  }
  // if nothing was matched for that lane then lane is empty
  return distAway;
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
  int lane = 1;
  double ref_vel = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel]
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          // state vector, ordered: cl_left,cl_right,keep_lane
          // start with high costs
          vector<double> states = {999.0,999.0,998.0}; // default is keep_lane
      
          int prev_path_size = previous_path_x.size();

          // set ref s to end of points
          if (prev_path_size > 0){
            car_s = end_path_s;
          }

          bool too_close = false;

          for(int i = 0; i < sensor_fusion.size(); i++){
            //check if the car is in our lane
            float d = sensor_fusion[i][6];
            if(d < (2+4*lane+2) && d > (2+4*lane-2)){
              //car is in our lane
              // but how close ?
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];

              // how close will the car be in our future path?
              check_car_s+=(double)prev_path_size*.02*check_speed;
              // we don't want to be less than 30meters close to that car
              if ((check_car_s > car_s) && (check_car_s - car_s) < 30){
                too_close = true;
                std::cout<<"too_close"<<std::flush<<std::endl;
                // see if we can change lanes by updating lane costs
                if (lane==1){ // if in center lane update costs of left nd right lanes
                  states[0] -= laneCost(sensor_fusion,prev_path_size,end_path_s,0);//cl_left
                  states[1] -= laneCost(sensor_fusion,prev_path_size,end_path_s,2);//cl_right
                }else if (lane==0){ //if in left lane update cost of right lane only
                  states[1] -= laneCost(sensor_fusion,prev_path_size,end_path_s,1);//cl_right
                }else{ // if in right lane update cost of left lane only
                  states[0] -= laneCost(sensor_fusion,prev_path_size,end_path_s,1);//cl_left
                }
                std::cout<<"States Vector"<<std::flush<<std::endl;
                print(states);
              }
            }
          }

          //get least cost state
          int leastCostStateIndex = std::min_element(states.begin(),states.end()) - states.begin();
          std::cout<<"selected state"<<leastCostStateIndex<<std::flush<<std::endl;
          if(too_close){ //too close
            ref_vel -= .224;
            std::cout<<"decrease speed"<<std::flush<<std::endl;
          }else if (ref_vel < 48.5){
            std::cout<<"increase speed"<<std::flush<<std::endl;
            ref_vel+=.324;
          }
          // check if we should change lane
          if (leastCostStateIndex == 0){ //cl_left
            lane = (lane+1)%2;
            std::cout<<"Change Lane Left"<<std::flush<<std::endl;
          }
          if (leastCostStateIndex == 1){ //cl_right
            lane = lane+1;
            std::cout<<"Change lane Right"<<std::flush<<std::endl;
          }
          std::cout<<"Current lane"<<lane<<std::flush<<std::endl;
          


          vector<double> ptsx;
          vector<double> ptsy;

          double pos_x = car_x;
          double pos_y = car_y;
          double pos_yaw = deg2rad(car_yaw);

          if (prev_path_size < 2) {
            double prev_x = car_x - cos(car_yaw);
            double prev_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_y);
            ptsy.push_back(car_y);
            // std::cout<<"no prev path:generating one"<<std::endl<<std::flush;

          } else {
            pos_x = previous_path_x[prev_path_size-1];
            pos_y = previous_path_y[prev_path_size-1];

            double pos_x2 = previous_path_x[prev_path_size-2];
            double pos_y2 = previous_path_y[prev_path_size-2];
            pos_yaw = atan2(pos_y-pos_y2,pos_x-pos_x2);

            ptsx.push_back(pos_x2);
            ptsx.push_back(pos_x);

            ptsy.push_back(pos_y2);
            ptsy.push_back(pos_y);
            // std::cout<<"generating from prev path"<<std::endl<<std::flush;
          }

          // generate 3 waypoints 30m apart
          vector<double> spread_waypoints;
          for (int i = 1; i <= 3; i++) {
           spread_waypoints = getXY(car_s+(30*i),(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
           ptsx.push_back(spread_waypoints[0]);
           ptsy.push_back(spread_waypoints[1]);
          }

          // vector<double> spread_waypoints = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          // ptsx.push_back(spread_waypoints[0]);
          // ptsy.push_back(spread_waypoints[1]);
          
          //convert pts to ferret coordinates i guess
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i]-pos_x;
            double shift_y = ptsy[i]-pos_y;

            ptsx[i] = (shift_x*cos(0-pos_yaw)-shift_y*sin(0-pos_yaw));
            ptsy[i] = (shift_x*sin(0-pos_yaw)+shift_y*cos(0-pos_yaw));
            
          }
          
          //create spline
          tk::spline s;

          //set (x,y) planes to the spline
          s.set_points(ptsx,ptsy);

          // double<vector> next_x_vals;
          // double<vector> next_y_vals;
          double push_back_size = prev_path_size;
          if (too_close){
            push_back_size = prev_path_size/2;
          }
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

          double x_addon = 0;             
          double N = (target_dist/(.02*ref_vel/2.24)); //divide by 2.24 for mps
          for (int i = 1; i <= 50-previous_path_x.size(); i++) {
            double x_point = x_addon+(target_x)/N;
            double y_point = s(x_point);
            x_addon = x_point;

            // rotate points back
            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref*cos(pos_yaw)-y_ref*sin(pos_yaw);
            y_point = x_ref*sin(pos_yaw)+y_ref*cos(pos_yaw);
            
            x_point += pos_x;
            y_point += pos_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          // double dist_inc = 0.5;
          // for (int i = 0; i < 50; ++i) {

          //   double next_s = car_s+(i+1)*dist_inc;
          //   double next_d = 6;
          //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //   next_x_vals.push_back(xy[0]);
          //   next_y_vals.push_back(xy[1]);
          //   // next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
          //   // next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          // }

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
    }  // end websockset if
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