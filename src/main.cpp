#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "vehicle.cpp"
#include "spline.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

double laneSpeed(int lane, vector<Vehicle> &vehicles, Vehicle &car)
{
  double lane_speed = MPH2mps(50.0);
  for (Vehicle vehicle : vehicles)
  {
    if (vehicle.lane == lane)
    {
      double distance_to_vehicle_ahead = wrappedDistance(car.s, vehicle.s);
      double distance_to_vehicle_behind = wrappedDistance(vehicle.s, car.s);
      if (((distance_to_vehicle_ahead < 100) || (distance_to_vehicle_behind < 10)) && (vehicle.speed < lane_speed))
      {
        lane_speed = vehicle.speed;
      }
    }
  }
  return lane_speed;
}

int fastestLane(vector<Vehicle> &vehicles, Vehicle &car)
{
  int fastest_lane = car.lane;
  double fastest_speed = laneSpeed(fastest_lane, vehicles, car);

  for (int lane = 0; lane < 3; lane++)
  {
    double lane_speed = laneSpeed(lane, vehicles, car);
    if ((lane_speed > fastest_speed) || ((lane_speed == fastest_speed) && (fabs(lane - car.lane) < fabs(fastest_lane - car.lane))))
    {
      fastest_speed = lane_speed;
      fastest_lane = lane;
    }
  }
  return fastest_lane;
}

double safetyCosts(int lane, vector<Vehicle> &vehicles, Vehicle &car)
{
  double safety_costs = 0.0;
  for (Vehicle vehicle : vehicles)
  {
    if (vehicle.lane == lane)
    {
      if ((wrappedDistance(vehicle.s, car.s) < 15) 
          || (wrappedDistance(car.s, vehicle.s) < 15)) 
      {
        safety_costs += 1.0;
      }
    }
  }
  return safety_costs;
}

int main()
{
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
  while (getline(in_map_, line))
  {
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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);
      int lane = 1;
      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          vector<Vehicle> vehicles = {};
          // j[1] is the data JSON object

          // Main car's localization Data
          Vehicle car = Vehicle(j, true);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          for (json sf : sensor_fusion)
          {
            Vehicle v = Vehicle(sf, false);
            vehicles.push_back(v);
          }
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          vector<double> pt_x;
          vector<double> pt_y;
          double ref_x = car.x;
          double ref_y = car.y;
          double ref_yaw = car.yaw_rad;
          double ref_speed = car.speed;
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          std::cout << "previous_path_x.size(): " << previous_path_x.size() << std::endl;
          int pathsize = previous_path_x.size();

          double car_s = car.s;

          if (pathsize > 0)
          {
            car_s = end_path_s;
          }

          bool too_close = false;

          for (Vehicle vehicle : vehicles)
          {
            if ((vehicle.lane == car.lane) || (vehicle.lane == lane))
            { // check the current lane and the destination lane
              double check_speed = vehicle.speed;
              double check_car_s = vehicle.s;

              check_car_s += ((double)pathsize * 0.02 * check_speed);

              double wrapped_distance = wrappedDistance(car_s, check_car_s);
              if (wrapped_distance < 15)
              {
                int fastest_lane = fastestLane(vehicles, car);
                if (lane == car.lane)
                {
                  // car has reached new lane
                  if (fastest_lane > lane)
                  {
                    // change lane if change is safe
                    if (safetyCosts(lane + 1, vehicles, car) < 0.2)
                    {
                      lane += 1;
                    }
                    else
                    {
                      std::cout << "Changing to " << fastest_lane << ". Waiting for car to clear safety range in lane " << lane + 1 << std::endl;
                    }
                  }
                  else if (fastest_lane < lane)
                  {
                    // change lane if change is safe
                    if (safetyCosts(lane - 1, vehicles, car) < 0.2)
                    {
                      lane -= 1;
                    }
                    else
                    {
                      std::cout << "Changing to " << fastest_lane << ". Waiting for car to clear safety range in lane " << lane - 1 << std::endl;
                    }
                  }
                }

                // reduce speed if lane change is not possible
                if (lane == car.lane)
                {
                  too_close = true;
                }
              }
            }
          }

          if (too_close)
          {
            ref_speed -= 0.224;
          }
          else if (ref_speed < MPH2mps(50.0))
          {
            ref_speed += 0.224;
          }

          if (previous_path_x.size() >= 2)
          {
            //             car.s = end_path_s;
            ref_x = previous_path_x[pathsize - 1];
            ref_y = previous_path_y[pathsize - 1];
            double ref_x_prev = previous_path_x[pathsize - 2];
            double ref_y_prev = previous_path_y[pathsize - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            pt_x.push_back(ref_x_prev);
            pt_y.push_back(ref_x);

            pt_x.push_back(ref_y_prev);
            pt_y.push_back(ref_y);
          }
          else
          {
            double prev_car_x = car.x - cos(car.yaw);
            double prev_car_y = car.y - sin(car.yaw);

            pt_x.push_back(prev_car_x);
            pt_x.push_back(car.x);

            pt_y.push_back(prev_car_y);
            pt_y.push_back(car.y);
          }

          
          double d = 4.0 * (0.5 + lane);
          for (int offset : {40, 60, 90, 120})
          {
            vector<double> next_wp = getXY(car.s + offset, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            pt_x.push_back(next_wp[0]);
            pt_y.push_back(next_wp[1]);
          }
          for (int i = 0; i < pt_x.size(); i++)
          {
            // shift car reference angle to 0 degree
            double shift_x = pt_x[i] - ref_x;
            double shift_y = pt_y[i] - ref_y;

            pt_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            pt_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
            
          }

          tk::spline spline;
          std::cout << "spline do" << std::endl;
          std::sort(pt_x.begin(), pt_x.end());
          std::sort(pt_y.begin(), pt_y.end());
          spline.set_points(pt_x, pt_y);
          std::cout << "spline done" << std::endl;
          double target_x = 40.0;
          double target_y = spline(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points

          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          for (int i = 1; i <= 50 - previous_path_x.size(); ++i)
          {

            double N = (target_dist / (0.02 * ref_speed)); // each 0.02 seconds a new point is reached, transform miles per hour to m/s
            double xp = x_add_on + (target_x) / N;
            double yp = spline(xp);
            x_add_on = xp;

            double x_ref = xp;
            double y_ref = yp;

            // rotating back to normal after rotating it earlier
            xp = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            yp = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            xp += ref_x;
            yp += ref_y;

            next_x_vals.push_back(xp);
            next_y_vals.push_back(yp);
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
