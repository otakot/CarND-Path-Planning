#ifndef DRIVING_CONTEXT_H
#define DRIVING_CONTEXT_H

#include <vector>
#include <cstdint>

using std::vector;

struct DrivingContext {

  std::uint8_t total_lanes;
  double lane_width; // in meters
  double max_speed; // in mps
  double ego_postion_refresh_interval; // in seconds
  double safe_ratio_distance_to_speed;
  double safe_distance_to_vehicle_behind; // in meters
  double max_deceleration; // in m/s2
  double max_accceleration; // in m/s2
  double safe_time_to_maneuver_start;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

};

#endif // DRIVING_CONTEXT_H
