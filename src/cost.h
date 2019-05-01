#ifndef COST_H
#define COST_H

#include <memory>

#include "vehicle.h"
#include "driving_context.h"
#include "driving_state.h"

/**
 * Calculates the total cost of driving in given state
 *
 * @param driving_context driving environment parameters
 * @param ego_vehicle Ego vehicle
 * @param predictions A list of detected other vehicles of the road
 * @param target_state Target state for cost evaluation
 */
double CalculateDrvingStateCost(const std::shared_ptr<DrivingContext>& driving_context,
                               const Vehicle& ego_vehicle,
                               const vector<Vehicle>& predictions,
                               const DrivingState& target_state);

#endif  // COST_H
