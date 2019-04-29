#ifndef LOGGER_H
#define LOGGER_H

#include <string>

#include "driving_state.h"
#include "vehicle.h"

std::string LogDrivingState(const DrivingState& driving_state);
std::string LogVehilceDrivingParams(const Vehicle& ego);

#endif  // LOGGER_H
