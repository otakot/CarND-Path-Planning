#ifndef DRIVING_CONTEXT_H
#define DRIVING_CONTEXT_H

#include <vector>
#include <cstdint>

using std::vector;

struct DrivingContext {

  std::uint8_t total_lanes_;
  double lane_width_; // in meters
  double max_speed_; // in meters per second
  double ego_postion_refresh_interval_; // in seconds
  double safe_distance_to_speed_ratio_;
  double max_deceleration_ ; // in m/s2
  double max_accceleration_; // in m/s2
  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;
  vector<double> map_waypoints_dx_;
  vector<double> map_waypoints_dy_;

};

#endif // DRIVING_CONTEXT_H
