#ifndef UTILITIES_H
#define UTILITIES_H

#include <math.h>
#include <string>
#include <vector>
#include <cstdint>
#include <memory>

#include "driving_context.h"
#include "json.hpp"
#include "vehicle.h"


using std::string;
using std::vector;
using nlohmann::json;

static const double kMpsToMphRatio = 2.23694;

void LoadRoadMap(const string& map_file, vector<double>& map_waypoints_x, vector<double>& map_waypoints_y,
  vector<double>& map_waypoints_s, vector<double>& map_waypoints_dx, vector<double>& map_waypoints_dy);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);

// Calculate distance between two points
const double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
const int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
const int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
const std::pair<double,double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
const std::pair<double, double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x, const vector<double> &maps_y);

bool GetLaneSpeed(const vector<Vehicle>& predictions, const uint8_t lane_index, float& lane_speed);

const double GetLaneCenterLineD(const uint8_t lane_index, const double& lane_width);

/**
 *  Calculates index of the lane by given valued of D (Frenet coordinates) and lane width information
 *  lane indices start from 0 (left most lane)
 */
const std::uint8_t CalculateLaneIndex(double d, double lane_width);

Vehicle CreateVehicle(const std::shared_ptr<DrivingContext>& context, const json& sensor_data);

std::string GetStateName(State state);

const vector<std::pair<double, double>> FetchRemaingPrevousTrajectory(const json& telemetry_data);

const vector<Vehicle> FetchOtherVehicles(
  const std::shared_ptr<DrivingContext>& context, const json& telemetry_data);

void UpdateEgoVehileWithLatestDrivingParams(const json& telemetry_data,
  DrivingState& current_driving_state, Vehicle& ego_vehicle);

#endif  // UTILITIES_H
