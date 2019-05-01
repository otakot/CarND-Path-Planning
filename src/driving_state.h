#ifndef DRIVING_STATE_H
#define DRIVING_STATE_H

#include <cstdint>

enum class State {
  KEEP_LANE, // initial state
  PREP_LANE_CHANGE_LEFT,
  LANE_CHANGE_LEFT,
  PREP_LANE_CHANGE_RIGHT,
  LANE_CHANGE_RIGHT
};

struct DrivingKinematics {
  double velocity; // in m/s
  double acceleration; // in m/s2
};

/**
 * By target and intended lanes it is meant the scenario when we still want to keep driving in lane 1
 * (target lane) but meanwhile prepare for changing to lane 2 soon (intended lane)
 */
struct DrivingState {
  std::uint8_t target_lane_index;
  std::uint8_t intended_lane_index;
  DrivingKinematics kinematics;
  State state;
};

#endif // DRIVING_STATE_H
