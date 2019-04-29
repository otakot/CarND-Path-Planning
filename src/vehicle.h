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
  /**
   * @brief Constructor
   */
  Vehicle(const std::shared_ptr<DrivingContext> driving_context, uint8_t id, uint8_t lane_index, double x,
    double y,double velocity, double s, double d, double yaw=.0, double acceleration=.0,
    State state= State::KEEP_LANE) : driving_context_(driving_context), id_(id), lane_index_(lane_index),
    x_(x), y_(y), velocity_(velocity), s_(s),d_(d), yaw_(yaw), acceleration_(acceleration), state_(state){
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
  const DrivingState CalculateNextOptimalDrivingState(const vector<Vehicle> &predictions) const;

  /**
   *   @brief Provides the possible next states given the current state for the FSM
   *   discussed in the course, with the exception that lane changes happen
   *   instantaneously, so LCL and LCR can only transition back to KL.
   */
  const vector<State> GetFeasibleSuccessorStates(const vector<Vehicle> &predictions) const;

  /**
   * @brief Generates the driving trajectory for reaching the given target driving state
   * Trajectory is generated using Spline interpolation. Also not covered waypoints of trajectory
   * generated in previous step are used as a start waypoints for new generated trajectory to smooth
   * the transition between previous and new trajectory in case of possible sharp curve
   *
   * @return calculated trajectory as list of {x,y} coordinates (in global map coordinates system)
   */
  const std::pair<vector<double>, vector<double>> CalculateDrivingTrajectory(
    const vector<std::pair<double, double>>& remaining_previous_trajectory,
    const double& remaining_previous_trajectory_end_s, const DrivingState& target_driving_state) const;

private:

  bool IsLaneChangeLeftPossible(const vector<Vehicle>& predictions) const;

  bool IsLaneChangeRightPossible(const vector<Vehicle>& predictions) const;

  const DrivingState CreateDrivingState(
    const State& target_state, const vector<Vehicle>& predictions) const;

  /**
   *  @brief Calculates vehilce kinematics for (position, velocity, acceleration)
   *  for a given lane index and time interval. Tries to choose the maximum velocity and acceleration,
   *  given other vehicle positions and accel/velocity constraints.
   */
  const DrivingKinematics CalculateVehilceKinematics(
    const vector<Vehicle>& predictions, const uint8_t target_lane_index, const double time_interval) const;

  const DrivingState CreateConstantSpeedState() const;

  const DrivingState CreateKeepLaneState(const vector<Vehicle>& predictions) const;

  const DrivingState CreateLaneChangeState(
    const State& target_state, const vector<Vehicle> &predictions) const;

  /**
   * @brief Creates the  driving state parameters for Prepared Lane Change target state.
   * This function implements preparation for safe lane change:
   * - if velocity in target lane is slower than ego velocity then:
   *   - if there is no other vehicle behind ego
   *     - then smoothly slow down to match target lane speed to be able to switch lane without high deceleration
   *   - otherwise keep current speed and wait for gap big enough to change to target lane
   */
  const DrivingState CreatePrepareLaneChangeState(
    const State& target_state, const vector<Vehicle> &predictions) const;

  /**
   * @brief Predicts longitudinal position of the vehicle with current speed and acceleration
   * for given time interval (in seconds)
   *
   * @param time Time for prediction. In seconds
   * @return predicted s position of the vehicle
   */
  double PredictLongitudinalPosition(const double time) const;

  /**
   * @brief Returns a true if a vehicle is found in ego lane and is located ahead/behind ego vehicle.
   * The passed reference rVehicle is updated if a vehicle is found.
   *
   * @param [in]lane_index Index of checked lane
   * @param [in] predictions List of predicted vehicles around the ego car
   * @param [in] ahead_of_ego Boolean flag indicating whether the detection should happen head or behind the ego car
   * @param [out] detected_vehicle Pointer to detected vehicle
   */
  bool GetClosestVehicleInLane(const uint8_t lane_index, const vector<Vehicle>& predictions,
    const bool is_ahead_of_ego, std::shared_ptr<Vehicle>& detected_vehicle) const;

  /**
   * @brief Checks whether the given is dangerously close too ego vehicle.
   * Currently the check is fairly simple safe_distance >=
   */
  bool IsVehicleTooClose(const Vehicle& vehicle) const ;

  bool IsLaneChangePossible(const uint8_t& target_lane_index, const vector<Vehicle>& predictions) const;

  //void ApplyTargetState(const DrivingState &target_state);

public:

  const uint8_t kTotalTrajectoryAnchorPoints = 5;
  const uint8_t kDistanceBetweenTrajectoryKeyPoints = 30; // in meters
  const uint8_t kTotalTrajectoryPoints = 50;

  const std::map<State, int> lane_index_change =
    {{State::CONSTANT_SPEED, 0},
    {State::KEEP_LANE, 0},
    {State::PREP_LANE_CHANGE_LEFT, 0},
    {State::LANE_CHANGE_LEFT, -1},
    {State::PREP_LANE_CHANGE_RIGHT, 0},
    {State::LANE_CHANGE_RIGHT, 1}};

  // const int kPreferredBuffer = 6; // safe distance between ego car and other car.

  double x_, y_, velocity_, s_, d_, yaw_, acceleration_;
  std::shared_ptr<DrivingContext> driving_context_;
  uint8_t id_, lane_index_;
  State state_;

};

#endif  // VEHICLE_H
