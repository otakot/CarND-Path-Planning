#include "logger.h"
#include "utilities.h"

#include <iostream>
#include <sstream>

std::string LogDrivingState(const DrivingState& driving_state) {
  std::stringstream  ss;
  ss <<  "  Target State: " << GetStateName(driving_state.state) << ",  Target Lane: " <<
   (int) driving_state.target_lane_index << ",  Intended Lane: " <<
   (int) driving_state.intended_lane_index  << " m, Velocity: " <<
   driving_state.kinematics.velocity * kMpsToMphRatio << " mph, Acceleration: " <<
   driving_state.kinematics.acceleration << " m/s2";
   return ss.str();
}

std::string LogVehilceDrivingParams(const Vehicle& ego){
  std::stringstream  ss;
  ss << " Lane: " << (int)ego.lane_index_ << ", Speed: " << ego.velocity_  * kMpsToMphRatio<<
  "mph, Position: " << ego.s_ << " m, Yaw: " << ego.yaw_ << ", Acceleration: " << ego.acceleration_<<
  " m/s2, Safe foresight distance: " << ego.GetSafeForesightDistance() << " m";
  return ss.str();
}
