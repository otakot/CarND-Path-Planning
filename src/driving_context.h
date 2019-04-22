#ifndef DRIVING_CONTEXT_H
#define DRIVING_CONTEXT_H

#include <utility>

class DrivingContext {
  public:
   DrivingContext(int total_lanes, int lane_width, float max_speed, float ego_postion_refresh_interval,
    float safe_distance_to_speed_ratio, float max_deceleration, float max_accceleration,
    vector<double>&& map_waypoints_x, vector<double>&& map_waypoints_y, vector<double>&& map_waypoints_s,
    vector<double>&& map_waypoints_dx, vector<double>&& map_waypoints_dy):
      total_lanes_(total_lanes),
      lane_width_(lane_width),
      max_speed_(max_speed),
      ego_postion_refresh_interval_(ego_postion_refresh_interval),
      safe_distance_to_speed_ratio_(safe_distance_to_speed_ratio),
      max_deceleration_(max_deceleration),
      max_accceleration_(max_accceleration),
      map_waypoints_x_(std::move(map_waypoints_x)),
      map_waypoints_y_(std::move(map_waypoints_y)),
      map_waypoints_s_(std::move(map_waypoints_s)),
      map_waypoints_dx_(std::move(map_waypoints_dx)),
      map_waypoints_dy_(std::move(map_waypoints_dy)){};

  const int total_lanes_;
  const int lane_width_; // in meters
  const float max_speed_; // in meters per second
  const float ego_postion_refresh_interval_; // in seconds
  const float safe_distance_to_speed_ratio_;
  const float max_deceleration_ ; // in m/s2
  const float max_accceleration_; // in m/s2
  const vector<double> map_waypoints_x_;
  const vector<double> map_waypoints_y_;
  const vector<double> map_waypoints_s_;
  const vector<double> map_waypoints_dx_;
  const vector<double> map_waypoints_dy_;

};

#endif // DRIVING_CONTEXT_H
