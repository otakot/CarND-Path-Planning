#ifndef DRIVING_STATE_H
#define DRIVING_STATE_H

#include <utility>
#include <cstdint>

enum class State {
  CONSTANT_SPEED, //initial state
  KEEP_LANE,
  PREP_LANE_CHANGE_LEFT,
  LANE_CHANGE_LEFT,
  PREP_LANE_CHANGE_RIGHT,
  LANE_CHANGE_RIGHT
};

struct DrivingKinematics {
  double s; // in meters
  double velocity; // in m/s
  double acceleration; // in m/s2
};

struct DrivingState {
  std::uint8_t lane_index; // not sure about initial lane index
  DrivingKinematics kinematics;
  State state;
};

#endif // DRIVING_STATE_H
