#include "planner.h"
#include "algorithm"
#include "helpers.h"

Planner::Planner(vector<double> map_waypoints_x,
                 vector<double> map_waypoints_y,
                 vector<double> map_waypoints_s,
                 vector<double> map_waypoints_dx,
                 vector<double> map_waypoints_dy)
{
    wp_x = map_waypoints_x;
    wp_y = map_waypoints_y;
    wp_s = map_waypoints_s;
    wp_dx = map_waypoints_dx;
    wp_dy = map_waypoints_dy;
}

void Planner::updateSensorFusion(json data)
{
    car = Vehicle(data, true);

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    vehicles.clear();
    json sensor_fusion = data[1]["sensor_fusion"];
    for (json sensor_fusion_vehicle : sensor_fusion)
    {
        vehicles.push_back(Vehicle(sensor_fusion_vehicle, false));
    }

    // Previous path data given to the Planner
    previous_path_x = data[1]["previous_path_x"];
    previous_path_y = data[1]["previous_path_y"];
    // Previous path's end s and d values
    end_path_s = data[1]["end_path_s"];
    end_path_d = data[1]["end_path_d"];
}

double Planner::laneSpeed(int lane)
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

double Planner::safetyCosts(int lane)
{
    double safety_costs = 0.0;
    for (Vehicle vehicle : vehicles)
    {
        if (vehicle.lane == lane)
        {
            if ((wrappedDistance(vehicle.s, car.s) < ref_velocity*1.1) 
                || (wrappedDistance(car.s, vehicle.s) < ref_velocity*1.1))
            {
                safety_costs += 1.0;
            }
        }
    }
    return safety_costs;
}

int Planner::fastestLane()
{
    int fastest_lane = car.lane;
    double fastest_speed = laneSpeed(fastest_lane);

    for (int lane = 0; lane < 3; lane++)
    {
        double lane_speed = laneSpeed(lane);
        if ((lane_speed > fastest_speed) || ((lane_speed == fastest_speed) && (fabs(lane - car.lane) < fabs(fastest_lane - car.lane))))
        {
            fastest_speed = lane_speed;
            fastest_lane = lane;
        }
    }
    return fastest_lane;
}

double Planner::centerLaneD(int lane)
{
    return 4 * (0.5 + lane);
}

double Planner::wrappedDistance(double back_s, double front_s)
{
    double distance = (front_s - back_s + 6945.554) - 6945.554;

    if (distance < 0)
    {
        distance = 6945.554 + distance;
    }
    return distance;
}

json Planner::path()
{

    // define the actual (x,y) points we will use for the planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    int prev_size = previous_path_x.size();

    double car_s = car.s;

    if (prev_size > 0)
    {
        car_s = end_path_s;
    }

    bool too_close = false;

    for (Vehicle vehicle : vehicles)
    {
        if ((vehicle.lane == car.lane) || (vehicle.lane == lane))
        {
            double check_speed = vehicle.speed;
            double check_car_s = vehicle.s;

            check_car_s += ((double)prev_size * 0.02 * check_speed);

            double wrapped_distance = wrappedDistance(car_s, check_car_s);
            if (wrapped_distance < ref_velocity*1.1)
            {
                int fastest_lane = fastestLane();
                if (lane == car.lane)
                {
                    if (fastest_lane > lane)
                    {
                        if (safetyCosts(lane + 1) < 0.2)
                        {
                            lane += 1;
                        }
                    }
                    else if (fastest_lane < lane)
                    {
                        if (safetyCosts(lane - 1) < 0.2)
                        {
                            lane -= 1;
                        }
                    }
                }
                if (lane == car.lane)
                {
                    too_close = true;
                }
            }
        }
    }

    if (too_close)
    {
        ref_velocity -= 0.25;
    }
    else if (ref_velocity < 49.0)
    {
        ref_velocity += 0.25;
    }

    vector<double> pt_x;
    vector<double> pt_y;

    double ref_x = car.x;
    double ref_y = car.y;
    double ref_yaw = car.yaw_rad;

    if (prev_size < 2)
    {
        double prev_x = car.x - cos(car.yaw);
        double prev_y = car.y - sin(car.yaw);

        pt_x.push_back(prev_x);
        pt_x.push_back(car.x);

        pt_y.push_back(prev_y);
        pt_y.push_back(car.y);
    }
    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        pt_x.push_back(ref_x_prev);
        pt_x.push_back(ref_x);

        pt_y.push_back(ref_y_prev);
        pt_y.push_back(ref_y);
    }

    double d = centerLaneD(lane);
    for (int offset : {30, 60, 90, 120})
    {
        vector<double> next_wp = getXY(car_s + offset, d, wp_s, wp_x, wp_y);
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

    // create a spline
    tk::spline s;

    // set (x,y) points to the spline
    s.set_points(pt_x, pt_y);

    // start with the previous path points from last time
    for (int i = 0; i < previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = 40.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

    double x_add_on = 0;

    // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points

    for (int i = 1; i <= 70 - previous_path_x.size(); i++)
    {

        double N = (target_dist / (0.02 * MPH2mps(ref_velocity))); // each 0.02 seconds a new point is reached, transform miles per hour to m/s
        double x_point = x_add_on + (target_x) / N;
        double y_point = s(x_point);
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotating back to normal after rotating it earlier
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
        ;

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    json msgJson;
    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;

    return msgJson;
}
