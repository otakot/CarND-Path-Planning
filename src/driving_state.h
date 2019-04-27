#ifndef DRIVING_STATE_H
#define DRIVING_STATE_H

#include <utility>

enum class State {
  CONSTANT_PEED, //initial state
  KEEP_LANE,
  PREP_LANE_CHANGE_LEFT,
  LANE_CHANGE_LEFT,
  PREP_LANE_CHANGE_RIGHT,
  LANE_CHANGE_RIGHT
};

struct DrivingKinematics {
  double s; // in meters
  double velocity; // in m/s
  double acceleration;
};

struct DrivingState {
  int lane_index;
  DrivingKinematics kinematics;
  State state;
};

#endif // DRIVING_STATE_H
