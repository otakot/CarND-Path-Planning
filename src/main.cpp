#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <utility>
#include <limits>
#include <math.h>

#include "Eigen/Core"
#include "Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "driving_context.h"
#include "driving_state.h"
#include "utilities.h"

using nlohmann::json;
using std::string;
using std::vector;

const string kTrackMapFilePath = "../data/highway_map.csv";  // Waypoint map to read from
const double kMaxS = 6945.554; // The max s value before wrapping around the track back to 0
const uint8_t kTotalLanes = 3;
const uint8_t kStartLane = 1; //start counting from 0 t(left most lane)
static const double kReferenceVelocity = 49.5; //mph
static const double kSafeDistanceToSpeedRatio = 1.5;
static const double kMpsToMphRatio = 2.23694;
const uint8_t kTotalTrajectoryPoints = 50;
const uint8_t kTotalTrajectoryAnchorPoints = 5;
const uint8_t kDistanceBetweenTrajectoryKeyPoints = 30; // in meters
const uint8_t kLaneWidth = 4; // in meters
static const double kCarPositionRefreshTime = 0.02; // in seconds
static const double kDecelerationConformLimit = 9; // in m/s2
static const double kAccelerationConformLimit = 9; // in m/s2
constexpr std::uint8_t kEgoVehicleId = std::numeric_limits<std::uint8_t>::max();

json ProcessTelemetryData(const std::shared_ptr<DrivingContext> context, const json& telemetry_data,
                          Vehicle& ego_vehicle, DrivingState& target_driving_state){

  // Update ego vehicle driving parameters using the sensor data received from simulator
  ego_vehicle.x_ = (double) telemetry_data["x"];
  ego_vehicle.y_ = (double) telemetry_data["y"];
  ego_vehicle.velocity_ = (double) telemetry_data["speed"] / kMpsToMphRatio;
  ego_vehicle.s_ = (double) telemetry_data["s"];
  ego_vehicle.d_ = (double) telemetry_data["d"];
  ego_vehicle.yaw_ = telemetry_data["yaw"];

  // Previous path data given to the Planner
  auto previous_path_x = telemetry_data["previous_path_x"];
  auto previous_path_y = telemetry_data["previous_path_y"];

  // Previous path's end s and d values
  double end_path_s = telemetry_data["end_path_s"];
  double end_path_d = telemetry_data["end_path_d"];

  // Sensor Fusion Data, contains information about other cars on the road.
  auto sensor_fusion = telemetry_data["sensor_fusion"];

  // generate a list of all other cars on the same side of the road.
  vector<Vehicle> other_vehicles;
  other_vehicles.reserve(sensor_fusion.size());
  for (const json vehicle_data : sensor_fusion) {
    other_vehicles.push_back(CreateVehicle(context, vehicle_data));
  }

  // DEFINE SAFE SPEED FOR CURRENT CAR MOVE

  const std::size_t previous_path_size = previous_path_x.size();

  bool slow_vehicle_ahead = false;

  std::shared_ptr<Vehicle> vehicle_ahead;
  if(ego_vehicle.GetClosestVehicleInLane(ego_vehicle.lane_index_, other_vehicles, true, vehicle_ahead)){
    std::cout << "Vehicle ahead in ego lane in " << (int)(vehicle_ahead->s_ - ego_vehicle.s_) <<"m" <<std::endl;
    if ((vehicle_ahead->s_ - ego_vehicle.s_) <  target_driving_state.kinematics.velocity * context->safe_distance_to_speed_ratio_){
        std::cout << "Slow vehicle ahead detected!" <<std::endl;
        slow_vehicle_ahead = true;
    }
  }


  // adapt the ego speed smoothly to prevent max acceleration/deceleration exceed
  if(slow_vehicle_ahead) {
    //  change lane if possible
    //if (LaneChangeFeasable()) {
      // ChangeLane();
    //} else { // or decrease the speed and wait for opportunity to change lane
      target_driving_state.kinematics.velocity-=
        context->ego_postion_refresh_interval_ * context->max_deceleration_;
      // PrepareForLaneChange();
    //}
  } else if (target_driving_state.kinematics.velocity < context->max_speed_) {
    target_driving_state.kinematics.velocity+=
      context->ego_postion_refresh_interval_ * context->max_accceleration_;
  }


  // KEEP LANE WITH SAME SPEED

  // build smooth trajectory using points from previous path
  double ref_x = ego_vehicle.x_;
  double ref_y = ego_vehicle.y_;
  double ref_yaw = deg2rad(ego_vehicle.yaw_);

  vector<double> anchor_points_x;
  vector<double> anchor_points_y;

  if(previous_path_size < 2) { //if previous path is almost empty use ccp as start point

    // calculate previous waypoints using current car yaw
    double previous_car_x = ego_vehicle.x_ - cos(ego_vehicle.yaw_);
    double previous_car_y = ego_vehicle.y_ - sin(ego_vehicle.yaw_);

    // use two last  waypoints from previous path for interpolation to make transition smoother
    anchor_points_x.push_back(previous_car_x);
    anchor_points_x.push_back(ego_vehicle.x_);

    anchor_points_y.push_back(previous_car_y);
    anchor_points_y.push_back(ego_vehicle.y_);

  } else { // otherwise use last point of previous path as reference waypoint

    ref_x = previous_path_x[previous_path_size - 1];
    ref_y = previous_path_y[previous_path_size - 1];
    double ref_x_previous = previous_path_x[previous_path_size - 2];
    double ref_y_previous = previous_path_y[previous_path_size - 2];
    ref_yaw = atan2(ref_y - ref_y_previous, ref_x - ref_x_previous);

    // use two last  waypoints from previous path for interpolation to make transition smoother
    anchor_points_x.push_back(ref_x_previous);
    anchor_points_x.push_back(ref_x);

    anchor_points_y.push_back(ref_y_previous);
    anchor_points_y.push_back(ref_y);
  }

  // set the starting point of new trajectory generation to last point of previous trajectory
  // since all previous points are just inherited by new trajectory
  double new_trajectory_points_start_s = (previous_path_size > 0) ? end_path_s : ego_vehicle.s_;

  // generate key points of new trajectory (to interpolate through them later)
  for (int i = 0; i < kTotalTrajectoryAnchorPoints; i++) {

    std::pair<double, double> next_key_point =
      getXY(new_trajectory_points_start_s + (i+1) * kDistanceBetweenTrajectoryKeyPoints,
        context->lane_width_/2 + context->lane_width_ * ego_vehicle.lane_index_,
        context->map_waypoints_s_, context->map_waypoints_x_, context->map_waypoints_y_);
    anchor_points_x.push_back(next_key_point.first);
    anchor_points_y.push_back(next_key_point.second);

  }

  // change reference angle to local coordinates system frame
  for (int i = 0; i < anchor_points_x.size(); i++) {
    double shift_x = anchor_points_x[i] - ref_x;
    double shift_y = anchor_points_y[i] - ref_y;

    anchor_points_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    anchor_points_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
  }

  // create spline for interpolation
  tk::spline trajectory_spline;
  trajectory_spline.set_points(anchor_points_x, anchor_points_y);


  vector<double> trajectory_x;
  vector<double> trajectory_y;

  // use remaining waypoints from previous trajectory as part of new trajectory
  for (int i = 0; i < previous_path_size; i++){
    trajectory_x.push_back(previous_path_x[i]);
    trajectory_y.push_back(previous_path_y[i]);
  }

  // calculate trajectory points by breaking down the distance between anchor points into segments on the spline
  // in the way that ego velocity will not exceed speed limit
  double target_x = kDistanceBetweenTrajectoryKeyPoints;
  double target_y = trajectory_spline(target_x);
  const double target_dist = sqrt(target_x*target_x + target_y*target_y);
  const double total_segments = target_dist /
    (context->ego_postion_refresh_interval_ * target_driving_state.kinematics.velocity);


  //fill in the rest of trajectory points
  double updated_x = 0;
  for (int i = 0; i < (kTotalTrajectoryPoints - previous_path_size); i++) {
    double point_x = updated_x + target_dist / total_segments;
    double point_y = trajectory_spline(point_x);

    updated_x = point_x;

    double x_tmp = point_x;
    double y_tmp = point_y;

    // change reference angle back to global coordinates system frame
    point_x = x_tmp * cos(ref_yaw) - y_tmp * sin(ref_yaw);
    point_y = x_tmp * sin(ref_yaw) + y_tmp * cos(ref_yaw);

    point_x+= ref_x;
    point_y+= ref_y;


    trajectory_x.push_back(point_x);
    trajectory_y.push_back(point_y);

  }

  // create and populate the response to be send back to simulator
  json response_message;
  response_message["next_x"] = trajectory_x;
  response_message["next_y"] = trajectory_y;
  return response_message;
}

int main() {

  // Load track map (values for x,y,s and d normalized normal vectors for track waypoints)
  vector<double> map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy;
  LoadRoadMap(kTrackMapFilePath, map_waypoints_x, map_waypoints_y,
    map_waypoints_s, map_waypoints_dx, map_waypoints_dy);

  // create driving environment context
  DrivingContext context{kTotalLanes, kLaneWidth, kReferenceVelocity/kMpsToMphRatio, kCarPositionRefreshTime,
    kSafeDistanceToSpeedRatio, kDecelerationConformLimit, kAccelerationConformLimit, std::move(map_waypoints_x),
    std::move(map_waypoints_y),std::move(map_waypoints_s), std::move(map_waypoints_dx), std::move(map_waypoints_dy)};

  // create instance of ego vehicle and configure it with initial settings
  Vehicle ego_vehicle(std::make_shared<DrivingContext>(context), kEgoVehicleId, kStartLane, 0, 0, 0, 0, 0);
  DrivingState target_driving_state{kStartLane, {.0, .0, .0}, State::KEEP_LANE};

  uWS::Hub web_socket_hub;
  web_socket_hub.onMessage([&context, &ego_vehicle, &target_driving_state]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the telemetry data JSON object
          json msgJson = ProcessTelemetryData(
            std::make_shared<DrivingContext>(context), j[1], ego_vehicle, target_driving_state);

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

  web_socket_hub.onConnection([&web_socket_hub](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  web_socket_hub.onDisconnection([&web_socket_hub](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (web_socket_hub.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  web_socket_hub.run();
}
