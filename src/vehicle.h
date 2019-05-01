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
   *
   * @param driving_context
   * @param id vehilce's unique ID
   * @param lane_index Index of the road lane (indices start from 0 for left most lane)
   * @param x X coordinate of the vehicle (in global coordinate system frame)
   * @param y Y coordinate of the vehicle (in global coordinate system frame)
   * @param velocity Velocity of the vehicle
   * @param s Longitudinal position of vehicle in relation to start point of the track in Frenet coordinate
   *          system frame
   * @param d Lateral shift from center of the road (yellow line )of vehicle (in Frenet coordinate system
   *          frame)
   * @param yaw Yaw of the vehicle in (in global coordinate system frame)
   * @param acceleration Acceleration of the vehicle (in m/s2)
   * @param state Current driving state of vehicle
   */
  Vehicle(const std::shared_ptr<DrivingContext> driving_context, const uint8_t id, const uint8_t lane_index,
    const double& x, const double& y,const double& velocity, const double& s, const double& d,
    const double& yaw=.0, const double& acceleration=.0, const State& state= State::KEEP_LANE) :
    driving_context_(driving_context), id_(id), lane_index_(lane_index), x_(x), y_(y), velocity_(velocity),
    s_(s),d_(d), yaw_(yaw), acceleration_(acceleration), state_(state){
  }

  /**
   *  @brief Calculates the optimal driving state of ego vehicle on the highway for moving cycle
   *  executed in simulated driving environment (duration of one cycle is 0.02 seconds)
   *
   *  @param predictions A list of detected other vehicles of the road.
   *
   *  @return Optimal set of driving parameters like state (e.g keep lane, chage to left lane, prepare for
   *  change to right lane), target lane index, velocity, acceleration for next cycle of drive of ego vehicle
   */
  const DrivingState CalculateNextOptimalDrivingState(const vector<Vehicle>& predictions) const;

  /**
   *  @brief Provides the possible next states given the current state for the FSM
   *  discussed in the course, with the exception that lane changes happen
   *  instantaneously, so LCL and LCR can only transition back to KL.
   *
   *  @param predictions A list of detected other vehicles of the road.
   */
  const vector<State> GetFeasibleSuccessorStates(const vector<Vehicle> &predictions) const;

  /**
   * @brief Generates the driving trajectory for reaching the given target driving state
   * Trajectory is generated using Spline interpolation. Also not covered waypoints of trajectory
   * generated in previous step are used as a start waypoints for new generated trajectory to smooth
   * the transition between previous and new trajectory in case of possible sharp curve
   *
   * @param remaining_previous_trajectory
   * @param remaining_previous_trajectory_end_s
   * @param target_driving_state
   *
   * @return calculated trajectory as list of {x,y} coordinates (in global map coordinates system)
   */
  const std::pair<vector<double>, vector<double>> CalculateDrivingTrajectory(
    const vector<std::pair<double, double>>& remaining_previous_trajectory,
    const double& remaining_previous_trajectory_end_s, const DrivingState& target_driving_state) const;

  double CalculateLaneSpeed(const vector<Vehicle>& predictions, const uint8_t lane_index) const;

  /**
   * Calculates the distance of detection of vehicles in front which is safe for executing the maneuver (eg.
   * lane change) by ego vehicle with current driving parameters
   *
   * Assumption: vehicle ahead is moving with constant speed
   *
   * @return calculated safe foresight distance
   */
  double GetSafeForesightDistance() const;

private:

  /**
   * Calculate driving parameters of ego vehicle for given target state taking into account given list
   * of other vehicles on the road
   *
   * @param target_state Target driving state of ego vehicle
   * @param predictions A list of detected other vehicles of the road.
   */
  const std::pair<DrivingState, float> EvaluateTargetState(
    const State& target_state, const vector<Vehicle>& predictions) const;

  const std::pair<DrivingState, float> EvaluateConstantSpeedState(const vector<Vehicle>& predictions) const;

  const std::pair<DrivingState, float> EvaluateKeepLaneState(const vector<Vehicle>& predictions) const;

  const std::pair<DrivingState, float> EvaluateLaneChangeState(
    const State& target_state, const vector<Vehicle> &predictions) const;

  /**
   * @brief Creates the  driving state parameters for Prepare to Lane Change target state.
   * Implements following behavior:
   * - if velocity in target lane is slower than ego velocity then:
   *   - if there is no other vehicle behind ego
   *     - smoothly slow down to match target lane speed to be able to switch lane without high deceleration
   *   - otherwise keep current speed and wait for gap big enough to change to target lane
   *
   *  @param target_state Desired driving state (either PrepareLaneChangeLeft or PrepareLaneChangeRight)
   *  @param predictions A list of detected other vehicles of the road.
   */
  const std::pair<DrivingState, float> EvaluatePrepareLaneChangeState(
    const State& target_state, const vector<Vehicle>& predictions) const;

  /**
   *  @brief Calculates vehilce kinematics for (position, velocity, acceleration)
   *  for a given lane index and time interval. Tries to choose the maximum velocity and acceleration,
   *  given other vehicle positions and accel/velocity constraints.
   *
   *  @param target_lane_index Index of target road lane for driving
   *  @param time_interval Time duration for calculating the driving kinematic parameters
   */
  const DrivingKinematics CalculateVehicleKinematics(
    const vector<Vehicle>& predictions, const uint8_t target_lane_index, const double& time_interval) const;

  /**
   * @brief Returns a true if a vehicle is found in ego lane and is located ahead/behind ego vehicle.
   * The passed reference rVehicle is updated if a vehicle is found.
   *
   * @param [in]lane_index Index of checked lane
   * @param [in] predictions A list of detected other vehicles of the road.
   * @param [in] ahead_of_ego Boolean flag indicating whether the detection should happen head or behind the ego car
   * @param [out] detected_vehicle Pointer to detected vehicle
   */
  bool GetClosestVehicleInLane(const uint8_t lane_index, const vector<Vehicle>& predictions,
    const bool is_ahead_of_ego, std::shared_ptr<Vehicle>& detected_vehicle) const;

  /**
   * @brief Checks whether the given is dangerously close too ego vehicle.
   *
   * @param vehicle Vehicle to check the distance to
   */
  bool IsVehicleTooClose(const Vehicle& vehicle) const ;

  /**
   * Checks whether a change to the Lane with given index is possible for ego vehicle at the moment
   *
   * @param target_lane_index Index of the road lane for lane change maneuver feasibility check
   * @param predictions A list of detected other vehicles of the road.
   */
  bool IsLaneChangePossible(const uint8_t& target_lane_index, const vector<Vehicle>& predictions) const;

public:
  double x_, y_, velocity_, s_, d_, yaw_, acceleration_;
  std::shared_ptr<DrivingContext> driving_context_;
  uint8_t id_, lane_index_;
  State state_;

private:

  const uint8_t kTotalTrajectoryAnchorPoints = 5;
  const uint8_t kDistanceBetweenTrajectoryKeyPoints = 30; // in meters
  const uint8_t kTotalTrajectoryPoints = 50;
  const double kMaxS = 6945.554; // The max s value before wrapping around the track back to 0

  const std::map<State, int> lane_index_change =
    {{State::KEEP_LANE, 0},
    {State::PREP_LANE_CHANGE_LEFT, 0},
    {State::LANE_CHANGE_LEFT, -1},
    {State::PREP_LANE_CHANGE_RIGHT, 0},
    {State::LANE_CHANGE_RIGHT, 1}};

};

#endif  // VEHICLE_H
