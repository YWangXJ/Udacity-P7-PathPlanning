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
  //Initialize Constant
  
  //start in lane 1
  int lane = 1;
  int target_lane = 1;
  //Have a reference velocity to target
  double ref_vel = 0;//mph
  double DES_VEL = 49.5;//desired speed if no vehicle in front
  double SPACING = 50;
  double LANE_WIDTH = 4.0;
  
  h.onMessage([&lane, &ref_vel, &DES_VEL, &SPACING, &LANE_WIDTH, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          //initialize bools of lane condition
          bool close_f = false;//front of ego vehicle
          bool close_left = false;// left lane of ego vehicle
          bool close_right = false; // right lane of ego vehicle
          bool close_lf = false;// vehicle in left front
          bool close_lb = false;// vehicle in left behind
          bool close_rf = false;// vehicle in right front
          bool close_rb = false;// vehicle in right behind        
          
          //prevent vehicle driving outside the current direction
          if(lane == 0){ 
            close_left = true;
          }
          // If car is in right lane car can't turn to the right
          // Equivalent to an obstacle ahead
          else if(lane == 2){
            close_right = true;
          }
          
          //initialize the varibles of other close vehicles
          float close_ref_vel = 0;
          float close_f_s = SPACING*2;
          float close_lb_s = SPACING*2;
          float close_rb_s = SPACING*2;
          
          
          //find ref_v to use
          for(int i = 0; i < sensor_fusion.size(); i++){   
            
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];
            float check_d = sensor_fusion[i][6];  
            check_car_s += ((double)prev_size*.02*check_speed); //if using porevious points, we can project s value out
//             std::cout <<  "check_speed" << check_speed << std::endl;
//             std::cout <<  "my_speed" << car_speed << std::endl;
            
            //check if any car in front
            if((check_car_s>car_s) && (check_car_s-car_s<SPACING))
            {
              //check if the front car is in current lane
              if(check_d < (LANE_WIDTH*0.5+LANE_WIDTH*lane+LANE_WIDTH*0.5) && check_d>(LANE_WIDTH*0.5+LANE_WIDTH*lane-LANE_WIDTH*0.5))
              {
                close_f = true;
                close_f_s = check_car_s - car_s;
                close_ref_vel = check_speed * 2.24; 
              }
            }
          }
          
          //check left 
          for(int i = 0; i < sensor_fusion.size(); i++){
              double vx_l = sensor_fusion[i][3];
              double vy_l = sensor_fusion[i][4];
              double check_speed_l = sqrt(vx_l*vx_l+vy_l*vy_l);
              double check_car_s_l = sensor_fusion[i][5];
              float check_d_l = sensor_fusion[i][6];  
            check_car_s_l += ((double)prev_size*.02*check_speed_l); //if using porevious points, we can project s value out
            if(fabs(check_car_s_l-car_s)<SPACING)
            {
              //check left
              if(check_d_l < (LANE_WIDTH*.5+LANE_WIDTH*(lane-1)+LANE_WIDTH*.5) && check_d_l > (LANE_WIDTH*.5+LANE_WIDTH*(lane-1)-LANE_WIDTH*.5) && lane !=0)
              {
                if(check_car_s_l-car_s>0 && ref_vel>check_speed_l*2.24)
                {
                  close_lf = true;
                }
                else if(fabs(check_car_s_l-car_s)<SPACING*0.5)
                {
                  
                    close_lb = true;
                  
                  if(abs(check_car_s_l-car_s)>SPACING*0.2 && ref_vel>check_speed_l*2.24*1.3)
                  {
                    close_lb = false;
                  }                
                }
                
              }
            }
          }
          
          //check right
          for(int i = 0; i < sensor_fusion.size(); i++){
            double vx_r = sensor_fusion[i][3];
            double vy_r = sensor_fusion[i][4];
            double check_speed_r = sqrt(vx_r*vx_r+vy_r*vy_r);
            double check_car_s_r = sensor_fusion[i][5];
            float check_d_r = sensor_fusion[i][6];  
            check_car_s_r += ((double)prev_size*.02*check_speed_r); //if using porevious points, we can project s value out
            if(fabs(check_car_s_r-car_s)<SPACING)
            {
              //check right
              if(check_d_r < (LANE_WIDTH*.5+LANE_WIDTH*(lane+1)+LANE_WIDTH*.5) && check_d_r > (LANE_WIDTH*.5+LANE_WIDTH*(lane+1)-LANE_WIDTH*.5) && lane !=2)
              {
                if(check_car_s_r-car_s>0 && ref_vel>check_speed_r*2.24)
                {
                  close_rf = true;
                }
                else if(fabs(check_car_s_r-car_s)<SPACING*0.5)
                {
                  
                    close_rb = true;
                  
                  if(abs(check_car_s_r-car_s)>SPACING*0.2 && ref_vel>check_speed_r*2.24*1.3)
                  {
                    close_rb = false;
                  }                
                }
                
              }
            }
          }
          //if lane change is available
          if(close_rb||close_rf){
            close_right = true;
          }
          if(close_lb||close_lf){
            close_left = true;
          }
        
         
          //apply acceleration or lane change
          // return to center lane
          
          if((lane==0 && !close_right)||(lane==2 && !close_left)){
            lane = 1;
          }
          
          // If there is a vehicle in the front
          if(close_f){
            if(ref_vel > close_ref_vel){
              
              // start slow down
              if(close_f_s>SPACING*0.05){
                ref_vel -= .224*3*(1.-close_f_s/SPACING);//decelerates more when close_f_s is small
              }
              else if(close_f_s<SPACING*0.4)
              {
                ref_vel = close_ref_vel;
              }
              // slow down  more when other vehicle is very close
              else if(close_f_s<SPACING*0.6){
                ref_vel -= .224*3*(1.5-close_f_s/SPACING);;
              }

              // make sure vehicle is moving formward
              if(ref_vel<=0){
                ref_vel=0;
              }

            }
            // Change lane
            if(!close_left){//change left first
              lane -= 1;
            }
            else if(!close_right){
              lane += 1;
            }
   
          }

          
          // If no vehicle ahead
          else if(ref_vel < DES_VEL)
          {
            ref_vel +=  .224*2;
          }
          std::cout<<"|car_d: "<< car_d<<"| L_Turn: "<<!close_left<<"| R_Turn: "<<!close_right<<"| C_Obs: "<<close_f
            <<"| C_Obs_s: "<<close_f_s<<"| Lane: "<<lane<<"| Ref_vel: "<<ref_vel<< std::endl;
            
            
         //============below are creating spline points==================//
          
          //create a list of sparse (x,y) waypoints, evenly spaced at 30m
          // will interpolate these waypoints with spline
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          //reference x,y,yaw states
          //either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          //if previous size is almost empty, use the car as starting reference
          if(prev_size<2){
          //use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          //use the previous path's end point as the starting reference
          else
          {
      
            //redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            
            //use two points to make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
              
          }
          
          //In Frenet add evenly 30m spaced points ahead of the starting reference
          
          vector<double> next_wp0 = getXY(car_s+30,(2+LANE_WIDTH*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+LANE_WIDTH*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+LANE_WIDTH*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i = 0; i < ptsx.size(); i++){
            
            //shift car reference angle to zero degree (for the simplicty of coding later)
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0-ref_yaw)- shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+ shift_y*cos(0-ref_yaw));
          }
          
          //create a spline
          tk::spline s;
          
          //set (x,y) points to the spline
//           for (auto i = ptsx.begin(); i != ptsx.end(); ++i){
//     			std::cout << *i << ' ';
//           }
//           for (auto i = ptsx.begin(); i != ptsx.end(); ++i){
//     			std::cout << *i << ' ';
//           }
          s.set_points(ptsx,ptsy);
          
          //define the actual (x,y) points we will use for the path planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          //start with all of the previous path points from last time
          for(int i = 0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
         
          double x_add_on = 0;
          
          //fill up the rest of the path planner after filling it with previous points, here we will always output 50 points
          for (int i = 0; i <= 70 - previous_path_x.size(); i++){
            
            double N = (target_dist/(.02*ref_vel/2.24)); // car move to next points for every 0.02 sec; 2.24 is convert from mph to m/s
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            //rotate back to normal after rotating it eariler
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          
          json msgJson;         
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