#include "cost.h"
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "utilities.h"
#include "vehicle.h"

using std::vector;

const float kSuboptimalSpeedCostWeight = 2;


float CalculateSuboptimalSpeedCost(const Vehicle& vehicle,
                        const vector<Vehicle>& predictions,
                        const DrivingState target_state,
                        const double& reference_speed) {

  float proposed_target_speed;
  if (!GetLaneSpeed(predictions, target_state.lane_index, proposed_target_speed)) {
    proposed_target_speed = reference_speed;
  }
  double speed_delta = 2* reference_speed - vehicle.velocity_ - proposed_target_speed;
  float cost = 1 - exp(-(std::abs(speed_delta) / reference_speed));

  return cost * kSuboptimalSpeedCostWeight;
}

float CalculateTargetStateCost(const std::shared_ptr<DrivingContext> driving_context,
                               const Vehicle& ego_vehicle,
                               const vector<Vehicle>& predictions,
                               const DrivingState& target_state) {

  // Sum weighted cost functions to get total cost for trajectory.
  float total_cost = 0.0;
  total_cost+= CalculateSuboptimalSpeedCost(ego_vehicle, predictions, target_state, driving_context->max_speed_);

  //TODO: add other fancy cost functions here

  return total_cost;
}

