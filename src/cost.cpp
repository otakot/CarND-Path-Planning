#include "cost.h"
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <iostream>

#include "utilities.h"
#include "vehicle.h"

using std::vector;

const double kSuboptimalSpeedCostWeight = 20;
const double kPreparationForManeuverCostWeight = 1;

/**
 * Cost becomes higher for trajectories with target lane
 * that have traffic slower than reference max speed.
 */
double CalculateSuboptimalSpeedCost(const Vehicle& ego_vehicle,
                        const vector<Vehicle>& predictions,
                        const std::uint8_t target_lane_index,
                        const double& optimal_speed) {

  double target_lane_speed = ego_vehicle.CalculateLaneSpeed(predictions, target_lane_index);
  double speed_delta = 2 * optimal_speed - ego_vehicle.velocity_ - target_lane_speed;
  double cost = 1 - exp(-(std::abs(speed_delta) / optimal_speed));
  std::cout << "  Cost of target driving speed: Target driving lane :" <<  (int)target_lane_index <<
    ", intended lane speed: " << target_lane_speed * kMpsToMphRatio << " mph, calculated speed cost: " <<
    cost << std::endl;

  return cost * kSuboptimalSpeedCostWeight;
}

double CalculateStateMaintananceCost(const State& ego_state) {

  bool is_ego_preparing_for_maneuver = false;
    switch (ego_state) {
    case  State::PREP_LANE_CHANGE_LEFT:
    case  State::PREP_LANE_CHANGE_RIGHT:
      is_ego_preparing_for_maneuver = true;
      break;
    default:
      break;
  }
  double cost = kPreparationForManeuverCostWeight * is_ego_preparing_for_maneuver;
  std::cout << "  Cost of maintaining the maneuver preparation: " <<  cost << std::endl;
  return cost;
}

double CalculateDrvingStateCost(const std::shared_ptr<DrivingContext>& driving_context,
                               const Vehicle& ego_vehicle,
                               const vector<Vehicle>& predictions,
                               const DrivingState& target_state) {

  // Sum weighted cost functions to get total cost for trajectory.
  float total_cost = 0.0;
  total_cost+= CalculateSuboptimalSpeedCost(
    ego_vehicle, predictions, target_state.intended_lane_index, driving_context->max_speed);
  total_cost+= CalculateStateMaintananceCost(target_state.state);

 //TODO: add other fancy cost functions here
  return total_cost;
}

