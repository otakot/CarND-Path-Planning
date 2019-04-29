#include "cost.h"
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "utilities.h"
#include "vehicle.h"

using std::vector;

const float kSuboptimalSpeedCostWeight = 1;


float CalculateSuboptimalSpeedCost(const Vehicle& vehicle,
                        const vector<Vehicle>& predictions,
                        const DrivingState target_state,
                        const double& optimal_speed) {

  float target_lane_speed;
  if (!GetLaneSpeed(predictions, target_state.lane_index, target_lane_speed)) {
    target_lane_speed = optimal_speed;
  }
  double speed_delta = 2* optimal_speed - vehicle.velocity_ - target_lane_speed;
  float cost = 1 - exp(-(std::abs(speed_delta) / optimal_speed));

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

