#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <set>
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

using namespace std;
using namespace Eigen;

// for convenience
using json = nlohmann::json;

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

  
  double ref_speed = 0.00; //reference speed of a car
  int car_lane = 1; //reference lane number
  
  h.onMessage([&car_lane,&ref_speed,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    
    cout << "--------------------------New iteration--------------------------" << endl;
    
    //the index of robocar in global variable - 0, 1, 2
    //robot starts in middle lane - 1
    
 
    const double ahead_buffer = 40.0; //buffer zone length for ahead vehicles
    const double side_forward_buffer = 30.0;//forward buffer zone length for side vehicles
    const double side_rear_buffer = 10.0;//rear buffer zone length for side vehicles
    const double max_acc = .224; //max acceleration
    
    
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
          
          cout << "Telemetry:" << endl;
          cout << "Yaw = " << car_yaw << endl;
          cout << "car_x = " << car_x << endl;
          cout << "car_y = " << car_y << endl;
          cout << "car_speed = " << car_speed << endl;

                    
          
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int prev_size = previous_path_x.size();

          if(prev_size > 0)
            car_s = end_path_s;

          cout << "prev_size = " << prev_size << endl;
          for(auto i = 0; i < prev_size; i++)
          {
            cout << "prev_path = " << previous_path_x[i] << ", " << previous_path_y[i] << endl;
          }
          
          //check if there is any cars in lanes
          bool obj_left = false;
          bool obj_ahead = false;
          bool obj_right = false;          
          
          for(auto obj: sensor_fusion)
          {
            //calculation object's lane index
            double obj_d = obj[6];
            int obj_lane = getLaneNumber(obj_d);
            
            if(obj_lane < 0)
              continue;
            
            //calculating object's velocity to predict it's position in lane
            double obj_vx = obj[3];
            double obj_vy = obj[4];
            double obj_vel = sqrt(obj_vx*obj_vx + obj_vy*obj_vy);
            double obj_s = obj[5];
            obj_s += prev_size*0.02*obj_vel; // predicting trajectory for cycles for previous trajectory
            
            if ((obj_lane == car_lane) && (obj_s - car_s < ahead_buffer) && (obj_s - car_s > 0))
            {
              obj_ahead = true;
            }
            else if ((((obj_s - car_s) < side_forward_buffer) && (obj_s > car_s)) || (((car_s - obj_s) < side_rear_buffer) && (car_s > obj_s)))
            {
              if ((obj_lane - car_lane) == 1)
                obj_right = true;
              else if((obj_lane - car_lane) == -1)
                obj_left = true;
            }
          }
          
          cout << "Relative lane info:" << endl;
          cout << "left = " << obj_left << endl;
          cout << "ahead= " << obj_ahead << endl;
          cout << "right = " << obj_right << endl;
          
          
          //simple finite automata for behavior planning
          //if there is a car ahead, then try to go left else try to go right
          double max_speed = 49.5; //max velocity by default
          if(obj_ahead)
          {
            //try to go left if it is not left lane and it is free
            if((car_lane > 0) && (!obj_left))
            {
              //chanching two lanes at once is restricted
              if(fabs(car_lane - getLaneNumber(car_d)) < 1)
              {
                car_lane--;
              }
            }
            //else try to go right if it is not right lane and it is free
            else if ((car_lane != 2) && (!obj_right))
            {
              //chanching two lanes at once is restricted
              if(fabs(car_lane - getLaneNumber(car_d)) < 1)
              {
                car_lane++;
              }
            }
            //if all lanes are busy just slow down a little bit
            else
            {
              ref_speed -= max_acc;
            }
          }
          else
          {
            //stay at the same lane
            car_lane = car_lane;
            if(ref_speed + max_acc < max_speed)
            {
              ref_speed += max_acc;
            }
          }
          
          
          //trajectory calculation
          //points in global frame
          std::vector<double> pts_x;
          std::vector<double> pts_y;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;
          
          //calculating two points
          if(prev_size < 2)
          {
            
            pts_x.push_back(car_x - cos(deg2rad(car_yaw)));
            pts_y.push_back(car_y - sin(deg2rad(car_yaw)));
            
            pts_x.push_back(car_x);
            pts_y.push_back(car_y);
          }
          else //or using previous points if available
          { 
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            
            ref_yaw = rad2deg(atan2(ref_y - ref_y_prev, ref_x - ref_x_prev));
            
            pts_x.push_back(ref_x_prev);
            pts_y.push_back(ref_y_prev);
            
            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y);
          }
          
          cout << "car_s = " << car_s << endl;
          
          vector<double> next = getXY(car_s + 50, 2 + 4*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          
          pts_x.push_back(next[0]);
          pts_y.push_back(next[1]);
          
          next = getXY(car_s + 80, 2 + 4*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          pts_x.push_back(next[0]);
          pts_y.push_back(next[1]);
          
          next = getXY(car_s + 100, 2 + 4*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          pts_x.push_back(next[0]);
          pts_y.push_back(next[1]);
          
          for(auto i = 0; i < pts_x.size(); i++)
          {
            cout << setprecision(5) << fixed;
            cout << "point = " << pts_x[i] << ", " << pts_y[i] << endl; 
          }
          
          set<double> x_points_set;
          vector<double> vr_x;
          vector<double> vr_y;
          
          vector<double> carPose = {ref_x, ref_y, ref_yaw};
          
          
          for(auto i = 0; i < pts_x.size(); i++)
          {

            vector<double> gmPt = {pts_x[i], pts_y[i]};
            vector<double> vrPt = gm2vr(gmPt, carPose);
            
            //checking if there were doublers for x coordinate
            if(x_points_set.insert(vrPt[0]).second)
            {
              //x_points_set.insert(vrPt[0]);
              vr_x.push_back(vrPt[0]);
              vr_y.push_back(vrPt[1]);
            }
            
          }
          
          for(auto i = 0; i < prev_size; i++)
          {
            /*
            vector<double> gmPt = {previous_path_x[i], previous_path_y[i]};
            vector<double> vrPt = gm2vr(gmPt, carPose);
            
            if(x_points_set.insert(vrPt[0]).second)
            {
              vr_x.push_back(vrPt[0]);
              vr_y.push_back(vrPt[1]); 
            }
            */
            
            
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          /*
          //sorting points to obtain correct order for x coordinate
          //putting two vectors in one vector
          vector<pair<double, double>> spl;
          
          for(auto i = 0; i < vr_x.size(); i++)
          {
            spl.push_back(pair<double, double>(vr_x[i], vr_y[i]));
          }
          
          //sorting x coordinates only
          std::sort(spl.begin(), spl.end(), cmp);
          
          //putting everything back into two arrays
          vr_x.clear();
          vr_y.clear();
          for(auto p: spl)
          {
            vr_x.push_back(p.first);
            vr_y.push_back(p.second);
            cout << "vr_pt = " << p.first << ", " << p.second << endl;
          }
          */
          
          tk::spline s;
          s.set_points(vr_x, vr_y);
          
          double target_x = 40.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          for( int i = 0; i < 50 - prev_size; i++ ) {
            
            cout << "ref_speed = " << ref_speed << endl;
            cout << "max_acc = " << max_acc << endl;
            
            double N = target_dist/(0.02*ref_speed/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;
            
            vector<double> vrPt = {x_point, y_point};            
            
            vector<double> gmPt = vr2gm(vrPt, carPose);

            next_x_vals.push_back(gmPt[0]);
            next_y_vals.push_back(gmPt[1]);
          }

          
          for (auto i = 0; i < next_x_vals.size(); i++)
          {
            cout << setprecision(5) << fixed;
            cout << "next_point = " << next_x_vals[i] << ", " << next_y_vals[i] << endl;
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
