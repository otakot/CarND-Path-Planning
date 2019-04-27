#ifndef COST_H
#define COST_H

#include "vehicle.h"


float CalculateTargetStateCost(const std::shared_ptr<DrivingContext> driving_context,
                               const Vehicle& ego_vehicle,
                               const vector<Vehicle>& predictions,
                               const DrivingState& target_state);

/**
 * Cost becomes higher for trajectories with target lane
 * that have traffic slower than reference max speed.
 */
float CalculateSuboptimalSpeedCost(const Vehicle& vehicle,
                        const vector<Vehicle>& predictions,
                        const DrivingState target_state,
                        const double& reference_speed);

#endif  // COST_H
