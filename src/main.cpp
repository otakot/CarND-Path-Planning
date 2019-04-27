#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <math.h>

#include "Eigen/Core"
#include "Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "driving_context.h"
#include "utilities.h"

using nlohmann::json;
using std::string;
using std::vector;

const string kTrackMapFilePath = "../data/highway_map.csv";  // Waypoint map to read from
const double kMaxS = 6945.554; // The max s value before wrapping around the track back to 0
const uint8_t kTotalLanes = 3;
const uint8_t kStartLane = 1; //start counting from 0 t(left most lane)
static const double kReferenceVelocity = 49.5; //mph
static const double kSafeDistanceToSpeedRatio = 0.5;
static const double kMphToMpsRatio = 2.24;
const uint8_t kTotalTrajectoryPoints = 50;
const uint8_t kTotalTrajectoryAnchorPoints = 5;
const uint8_t kDistanceBetweenTrajectoryKeyPoints = 30; // in meters
const uint8_t kLaneWidth = 4; // in meters
static const double kCarPositionRefreshTime = 0.02; // in seconds
static const double kDecelerationConformLimit = 5; // in m/s2
static const double kAccelerationConformLimit = 9; // in m/s2

json ProcessTelemetryData(const json& telemetry_data, const DrivingContext&& context,
  double& target_ego_velocity, std::uint8_t& ego_lane){

  // Ego localization Data
  double car_x = telemetry_data["x"];
  double car_y = telemetry_data["y"];
  double car_s = telemetry_data["s"];
  double car_d = telemetry_data["d"];
  double car_yaw = telemetry_data["yaw"];
  double car_speed = telemetry_data["speed"];

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
    other_vehicles.push_back(CreateVehicle(std::make_shared<DrivingContext>(context), vehicle_data));
  }

  // DEFINE SAFE SPEED FOR CURRENT CAR MOVE

  const std::size_t previous_path_size = previous_path_x.size();
  if(previous_path_size > 0) {
    car_x = end_path_s;
  }
  bool slow_car_ahead = false;
  for (const Vehicle vehilce : other_vehicles){
    if (ego_lane == vehilce.lane_index_){  //check whether detected car is in our lane

      double detected_car_s = vehilce.s_;
      // project longitudinal position of detected car into the future:
      // where detected car would be if ego car position would evolve to the last point of previous path
      detected_car_s+=(double)previous_path_size * context.ego_postion_refresh_interval_ * vehilce.velocity_;
      if((detected_car_s > car_s) && ((detected_car_s - car_s) <
        target_ego_velocity * context.safe_distance_to_speed_ratio_ * kMphToMpsRatio)) {
        // if detected car is in front of ego and is too close
        slow_car_ahead = true;
      }
    }
  }

  // adapt the ego speed smoothly to prevent max acceleration/deceleration exceed
  if(slow_car_ahead) {
    //  change lane if possible
    //if (LaneChangeFeasable()) {
      // ChangeLane();
    //} else { // or decrease the speed and wait for opportunity to change lane
      target_ego_velocity-= context.ego_postion_refresh_interval_ * context.max_deceleration_ * kMphToMpsRatio;
      // PrepareForLaneChange();
    //}
  } else if (target_ego_velocity < context.max_speed_) {
    target_ego_velocity+= context.ego_postion_refresh_interval_ * context.max_accceleration_ * kMphToMpsRatio;
  }


  // KEEP LANE WITH SAME SPEED

  // build smooth trajectory using points from previous path
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  vector<double> anchor_points_x;
  vector<double> anchor_points_y;

  if(previous_path_size < 2) { //if previous path is almost empty use ccp as start point

    // calculate previous waypoints using current car yaw
    double previous_car_x = car_x - cos(car_yaw);
    double previous_car_y = car_y - sin(car_yaw);

    // use two last  waypoints from previous path for interpolation to make transition smoother
    anchor_points_x.push_back(previous_car_x);
    anchor_points_x.push_back(car_x);

    anchor_points_y.push_back(previous_car_y);
    anchor_points_y.push_back(car_y);

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

  // generate key points of new trajectory (to interpolate through them later)
  for (int i = 0; i < kTotalTrajectoryAnchorPoints; i++) {

    std::pair<double, double> next_key_point =
      getXY(car_s + (i+1) * kDistanceBetweenTrajectoryKeyPoints,
        context.lane_width_/2 + context.lane_width_ * ego_lane,
        context.map_waypoints_s_, context.map_waypoints_x_, context.map_waypoints_y_);
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
    (context.ego_postion_refresh_interval_ * target_ego_velocity / kMphToMpsRatio);


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
  DrivingContext context{kTotalLanes, kLaneWidth, kReferenceVelocity, kCarPositionRefreshTime,
    kSafeDistanceToSpeedRatio, kDecelerationConformLimit, kAccelerationConformLimit, std::move(map_waypoints_x),
    std::move(map_waypoints_y),std::move(map_waypoints_s), std::move(map_waypoints_dx), std::move(map_waypoints_dy)};

  // initial ego configuration
  std::uint8_t ego_lane = kStartLane;
  double target_ego_velocity = 0; // in m/s. we start smoothly to prevent max acceleration exceed

  // create instance of ego vehile and configure it with initial settings
  //  Vehicle ego = Vehicle(lane_num, s, this->lane_speeds[lane_num], 0);
  //  ego.configure(config_data);
  //  ego.state = "KL";


  uWS::Hub web_socket_hub;
  web_socket_hub.onMessage([&context, &target_ego_velocity, &ego_lane]
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
          // j[1] is the telemetry data JSON object
          json msgJson = ProcessTelemetryData(j[1], std::move(context), target_ego_velocity, ego_lane);

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
