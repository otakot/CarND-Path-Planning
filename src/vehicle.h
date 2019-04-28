#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cstdint>

#include "driving_context.h"
#include "driving_state.h"

class Vehicle {
 public:
  // Constructors

  Vehicle(const std::shared_ptr<DrivingContext> driving_context, uint8_t id, uint8_t lane_index, double x, double y,
    double velocity, double s, double d, double yaw=.0, double acceleration=.0) : driving_context_(driving_context), id_(id),
    lane_index_(lane_index), x_(x), y_(y), velocity_(velocity), s_(s),d_(d), yaw_(yaw), acceleration_(acceleration){
  }

  /**
   * @brief
   *
   * @param A predictions list. This is a list of prediceted vehilces. Trajectories are a vector of Vehicle
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing
   *    a vehicle trajectory, given a state and predictions. Note that
   *    trajectory vectors might have size 0 if no possible trajectory exists
   *    for the state.
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   */
  DrivingState CalculateNextOptimalDrivingState(const vector<Vehicle> &predictions);

  /**
   *   Provides the possible next states given the current state for the FSM
   *   discussed in the course, with the exception that lane changes happen
   *   instantaneously, so LCL and LCR can only transition back to KL.
   */
  vector<State> GetFeasibleSuccessorStates(const vector<Vehicle> &predictions);

  bool IsLaneChangeLeftPossible(const vector<Vehicle>& predictions);
  bool IsLaneChangeRightPossible(const vector<Vehicle>& predictions);

  DrivingState CreateDrivingState(const State& target_state, const vector<Vehicle>& predictions);

  /**
   *   Calculates vehilce kinematics for (position, velocity, acceleration)
   *   for a given lane index and time interval. Tries to choose the maximum velocity and acceleration,
   *   given other vehicle positions and accel/velocity constraints.
   */
  DrivingKinematics CalculateVehilceKinematics(
    const vector<Vehicle>& predictions, const uint8_t target_lane_index, const uint16_t time);

  DrivingState CreateConstantSpeedState();

  DrivingState CreateKeepLaneState(const vector<Vehicle>& predictions);

  DrivingState CreateLaneChangeState(const State& target_state, const vector<Vehicle> &predictions);

  /** Creates the  driving state parameters for Prepared Lane Change target state.
   * This function implements preparation for safe lane change:
   * - if velocity in target lane is slower than ego velocity then:
   *   - if there is no other vehicle behind ego
   *     - then smoothly slow down to match target lane speed to be able to switch lane without high deceleration
   *   - otherwise keep current speed and wait for gap big enough to change to target lane
   */
  DrivingState CreatePrepareLaneChangeState(const State& target_state, const vector<Vehicle> &predictions);

  /**
   * Predicts longitudinal position of the vehicle with current speed and acceleration
   * for given time interval (in seconds)
   *
   * @param time Time for prediction. In seconds
   * @return predicted s position of the vehicle
   */
  double PredictLongitudinalPosition(const double time) const;

  /**
   * Returns a true if a vehicle is found in ego lane and is located ahead/behind ego vehicle.
   * The passed reference rVehicle is updated if a vehicle is found.
   *
   * @param [in]lane_index Index of checked lane
   * @param [in] predictions List of predicted vehicles around the ego car
   * @param [in] ahead_of_ego Boolean flag indicating whether the detection should happen head or behind the ego car
   * @param [out] detected_vehicle Pointer to detected vehicle
   */
 bool GetClosestVehicleInLane(const uint8_t lane_index,
  const vector<Vehicle>& predictions, const bool is_ahead_of_ego, std::shared_ptr<Vehicle>& detected_vehicle);

private:
  bool IsLaneChangePossible(const uint8_t& target_lane_index, const vector<Vehicle>& predictions);

  //void ApplyTargetState(const DrivingState &target_state);

public:
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  const std::map<State, int> lane_index_change =
    {{State::CONSTANT_SPEED, 0},
    {State::KEEP_LANE, 0},
    {State::PREP_LANE_CHANGE_LEFT, 0},
    {State::LANE_CHANGE_LEFT, -1},
    {State::PREP_LANE_CHANGE_RIGHT, 0},
    {State::LANE_CHANGE_RIGHT, 1}};

  const int kPreferredBuffer = 6; // impacts "keep lane" behavior.

  int L = 1;

  uint8_t id_, lane_index_;

  double x_, y_, velocity_, s_, d_, yaw_, acceleration_;

  State state_= State::CONSTANT_SPEED;

  std::shared_ptr<DrivingContext> driving_context_;
};

#endif  // VEHICLE_H
