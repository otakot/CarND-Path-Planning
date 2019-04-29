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
#include "vehicle.h"
#include "driving_context.h"
#include "driving_state.h"
#include "utilities.h"
#include "logger.h"

using nlohmann::json;
using std::string;
using std::vector;
using std::pair;

const string kTrackMapFilePath = "../data/highway_map.csv";  // Waypoint map to read from
const double kMaxS = 6945.554; // The max s value before wrapping around the track back to 0
const uint8_t kTotalLanes = 3;
const uint8_t kStartLane = 1; //start counting from 0 t(left most lane)
static const double kReferenceVelocity = 49.5; //mph
static const double kSafeDistanceToSpeedRatio = 1.5;
const uint8_t kLaneWidth = 4; // in meters
static const double kCarPositionRefreshTime = 0.02; // in seconds
static const double kDecelerationConformLimit = 9; // in m/s2
static const double kAccelerationConformLimit = 9; // in m/s2
constexpr std::uint8_t kEgoVehicleId = std::numeric_limits<std::uint8_t>::max();

json ProcessTelemetryData(const std::shared_ptr<DrivingContext> context, const json& telemetry_data,
                          Vehicle& ego_vehicle, DrivingState& target_driving_state){

  // update ego vehicle current driving params with data received from simulator
  UpdateEgoVehileWithLatestDrivingParams(telemetry_data, target_driving_state, ego_vehicle);
  std::cout << "Current Ego vehicle driving params: " << LogVehilceDrivingParams(ego_vehicle) << std::endl;

  // read remaining (uncovered by ego) part of previous  trajectory from telemetry data
  const vector<pair<double, double>> remaining_previous_trajectory = FetchRemaingPrevousTrajectory(telemetry_data);

  // previous trajectory end s and d values
  const double end_path_s = telemetry_data["end_path_s"];
  //double end_path_d = telemetry_data["end_path_d"];

  // generate a list of all other cars on the same side of the road.
  const vector<Vehicle> other_vehicles = FetchOtherVehicles(context, telemetry_data);

  // calculate optimal driving state for next cycle
  target_driving_state = ego_vehicle.CalculateNextOptimalDrivingState(other_vehicles);

  // generate optimal trajectory for target state
  const pair<vector<double>, vector<double>> trajectory = ego_vehicle.CalculateDrivingTrajectory(
    remaining_previous_trajectory, end_path_s, target_driving_state);

  // create and populate the response to be send back to simulator
  json response_message;
  response_message["next_x"] = trajectory.first;
  response_message["next_y"] = trajectory.second;
  return response_message;
}

int main() {

  // Load track map (values for x,y,s and d normalized normal vectors for track waypoints)
  vector<double> map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy;
  LoadRoadMap(kTrackMapFilePath, map_waypoints_x, map_waypoints_y,
    map_waypoints_s, map_waypoints_dx, map_waypoints_dy);

  // create driving environment context
  const DrivingContext context{kTotalLanes, kLaneWidth, kReferenceVelocity/kMpsToMphRatio, kCarPositionRefreshTime,
    kSafeDistanceToSpeedRatio, kDecelerationConformLimit, kAccelerationConformLimit, std::move(map_waypoints_x),
    std::move(map_waypoints_y),std::move(map_waypoints_s), std::move(map_waypoints_dx), std::move(map_waypoints_dy)};

  // create instance of ego vehicle and configure it with initial settings
  // after every vehicle movement step these two instances will be updated with new calculated data
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
          const json msgJson = ProcessTelemetryData(
            std::make_shared<DrivingContext>(context), j[1], ego_vehicle, target_driving_state);

          const auto msg = "42[\"control\","+ msgJson.dump()+"]";

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
