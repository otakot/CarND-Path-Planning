#include "logger.h"
#include "utilities.h"

#include <iostream>
#include <sstream>

std::string LogDrivingState(const DrivingState& driving_state) {
  std::stringstream  ss;
  ss <<  "  State: " << GetStateName(driving_state.state) << ",  Lane index: " <<
   (int) driving_state.lane_index <<  ", Position:" <<  driving_state.kinematics.s <<
  " m, Velocity: " << driving_state.kinematics.velocity * kMpsToMphRatio << " mph, Acceleration: " <<
  driving_state.kinematics.acceleration << " m/s2";
   return ss.str();
}

std::string LogVehilceDrivingParams(const Vehicle& ego){
  std::stringstream  ss;
  ss << " Lane: " << (int)ego.lane_index_ << ", Speed: " << ego.velocity_  * kMpsToMphRatio<<
  "mph, Position: " << ego.s_ << ", Yaw: " << ego.yaw_ << ", Acceleration: " << ego.acceleration_;
  return ss.str();
}
