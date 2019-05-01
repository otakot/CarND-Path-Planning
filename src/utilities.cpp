#include "utilities.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdint>
#include <math.h>
#include <assert.h>

#include "json.hpp"
#include "vehicle.h"

using std::string;
using std::vector;
using nlohmann::json;
using std::pair;

void LoadRoadMap(const string& map_file, vector<double>& map_waypoints_x, vector<double>& map_waypoints_y,
  vector<double>& map_waypoints_s, vector<double>& map_waypoints_dx, vector<double>& map_waypoints_dy){

  std::ifstream in_map_(map_file, std::ifstream::in);

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
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
const double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
const int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
const int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
const pair<double,double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
const pair<double, double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

const std::uint8_t CalculateLaneIndex(double d, double lane_width) {
  return d / lane_width;
}

Vehicle CreateVehicle(const std::shared_ptr<DrivingContext>& context, const json& sensor_data){

  const double x = sensor_data[1];
  const double y = sensor_data[2];
  const double v_x = sensor_data[3];
  const double v_y = sensor_data[4];
  const double velocity = sqrt(v_x*v_x + v_y*v_y);
  const std::uint8_t id = sensor_data[0];
  const double s = sensor_data[5];
  const double d = sensor_data[6];
  const std::uint8_t lane_index = CalculateLaneIndex(d, context->lane_width);
  return Vehicle(context, id, lane_index, x, y, velocity, s, d);
}

std::string GetStateName(State state) {
  switch(state)
  {
      case State::KEEP_LANE  : return "Keep Lane";
      case State::PREP_LANE_CHANGE_LEFT  : return "Prepare Lane Change Left";
      case State::LANE_CHANGE_LEFT  : return "Lane Change Left";
      case State::PREP_LANE_CHANGE_RIGHT  : return "Prepare Lane Change Right";
      case State::LANE_CHANGE_RIGHT  : return "Lane Change Right";
      default:
        throw std::runtime_error("Unsupported state");
  }
}

const double GetLaneCenterLineD(const uint8_t lane_index, const double& lane_width){
  return lane_width * lane_index + lane_width / 2;
}

const vector<pair<double, double>> FetchRemaingPrevousTrajectory(const json& telemetry_data) {
  json remaining_trajectory_x = telemetry_data["previous_path_x"];
  json remaining_trajectory_y = telemetry_data["previous_path_y"];

  //make sure that number of x coordinates matches the number of y coordinates
  assert(remaining_trajectory_x.size() == remaining_trajectory_y.size());

  vector<pair<double, double>> remaining_trajectory;
  remaining_trajectory.reserve(remaining_trajectory_x.size());
  for (int i = 0; i < remaining_trajectory_x.size(); i++){
    remaining_trajectory.push_back(std::make_pair(remaining_trajectory_x[i],remaining_trajectory_y[i]));
  }
  return remaining_trajectory;
}

const vector<Vehicle> FetchOtherVehicles(
  const std::shared_ptr<DrivingContext>& context, const json& telemetry_data){

  // sensor fusion data, contains information about other cars on the road.
  const auto sensor_fusion = telemetry_data["sensor_fusion"];

  vector<Vehicle> other_vehicles;
  other_vehicles.reserve(sensor_fusion.size());
  for (const json vehicle_data : sensor_fusion) {
    other_vehicles.push_back(CreateVehicle(context, vehicle_data));
  }
  return other_vehicles;
}

void UpdateEgoVehileWithLatestDrivingParams(const json& telemetry_data,
  DrivingState& current_driving_state, Vehicle& ego_vehicle){

  // update current state of ego vehicle
  ego_vehicle.state_ = current_driving_state.state;

  // update ego lane index
  ego_vehicle.lane_index_ = current_driving_state.target_lane_index;

  // update ego vehicle driving parameters using the telemetry data received from simulator
  ego_vehicle.x_ = (double) telemetry_data["x"];
  ego_vehicle.y_ = (double) telemetry_data["y"];
  ego_vehicle.s_ = (double) telemetry_data["s"];
  ego_vehicle.d_ = (double) telemetry_data["d"];
  ego_vehicle.yaw_ = telemetry_data["yaw"];

  // we assume that ego velocity and acceleration were set in simulator in previous step using
  // provided target driving state
  ego_vehicle.velocity_ = current_driving_state.kinematics.velocity;
  ego_vehicle.acceleration_ = current_driving_state.kinematics.acceleration;

  // this somehow does not work: resulting speed is way smaller than set into target state in previous step
  // ego_vehicle.velocity_ = (double) telemetry_data["speed"] / kMpsToMphRatio;
}
