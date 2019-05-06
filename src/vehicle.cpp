#include "vehicle.h"
#include "cost.h"
#include "utilities.h"
#include "logger.h"
#include "spline.h"

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>
#include <limits>
#include <iostream>
#include <math.h>
#include "Eigen/Core"
#include "Eigen/QR"

using std::string;
using std::vector;
using std::pair;


const DrivingState Vehicle::CalculateNextOptimalDrivingState(const vector<Vehicle>& predictions) const {

  const vector<State> sucessor_states = GetFeasibleSuccessorStates(predictions);

  vector<float> costs;

  vector<DrivingState> feasible_next_driving_states;
  std::cout << "Calculating feasible driving states for transition..."  << std::endl <<
               "-----------------------------------------------------------------------------------" <<
               std::endl;
  for (const State state_id : sucessor_states) {
     // generate the driving parameters and calculate cost for feasable state
    const std::pair<DrivingState, float> evaluated_target_state = EvaluateTargetState(state_id, predictions);

    std::cout << LogDrivingState(evaluated_target_state.first) <<
      ", Cost: "  << evaluated_target_state.second << std::endl;
    std::cout << "-----------------------------------------------------------------------------------" <<
              std::endl;
    feasible_next_driving_states.push_back(evaluated_target_state.first);
    costs.push_back(evaluated_target_state.second);
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  std::uint16_t optimal_state_index = distance(begin(costs), best_cost);

  DrivingState optimal_next_driving_state = feasible_next_driving_states[optimal_state_index];
  std::cout << "Target state selected : "  << GetStateName(optimal_next_driving_state.state) <<
            std::endl << std::endl;
  return optimal_next_driving_state;
}

const vector<State> Vehicle::GetFeasibleSuccessorStates(const vector<Vehicle>& predictions) const {

  vector<State> states;
  states.push_back(State::KEEP_LANE); // we should always be able to stay in current lane
  switch (state_) {
    case  State::KEEP_LANE:
      if (lane_index_ > 0){
         states.push_back(State::PREP_LANE_CHANGE_LEFT);
      }
      if (lane_index_ < driving_context_->total_lanes - 1){
         states.push_back(State::PREP_LANE_CHANGE_RIGHT);
      }
      break;
    case State::PREP_LANE_CHANGE_LEFT:
      states.push_back(State::PREP_LANE_CHANGE_LEFT); // we should be able to keep preparing
      if ((lane_index_ > 0) && IsLaneChangePossible(lane_index_ - 1, predictions)){
          states.push_back(State::LANE_CHANGE_LEFT);
      }
      break;
    case State::PREP_LANE_CHANGE_RIGHT:
      states.push_back(State::PREP_LANE_CHANGE_RIGHT); // we should be able to keep preparing
      if ((lane_index_ < driving_context_->total_lanes - 1) && IsLaneChangePossible(lane_index_ + 1, predictions)){
          states.push_back(State::LANE_CHANGE_RIGHT);
      }
      break;
    default:
     break;
     // If state is "LANE_CHANGE_LEFT" or "LANGE_CHANGE_RIGHT", then just return "KEEP_LANE" for now.
  }
  return states;
}

const std::pair<DrivingState, float> Vehicle::EvaluateTargetState(
  const State& target_state, const vector<Vehicle>& predictions) const {
  // Given a possible next state, generate the appropriate trajectory to realize  the next state.
  switch (target_state) {
    case State::KEEP_LANE:
      return EvaluateKeepLaneState(predictions);
    case State::LANE_CHANGE_LEFT:
    case State::LANE_CHANGE_RIGHT:
      return EvaluateLaneChangeState(target_state, predictions);
    case State::PREP_LANE_CHANGE_LEFT:
    case State::PREP_LANE_CHANGE_RIGHT:
      return EvaluatePrepareLaneChangeState(target_state, predictions);
    default:
      throw std::runtime_error("Unknown target state");
  }
}

const std::pair<DrivingState, float>Vehicle::EvaluateKeepLaneState(
  const vector<Vehicle>& predictions) const{

  DrivingKinematics kinematics = CalculateVehicleKinematics(
    predictions, lane_index_, driving_context_->ego_postion_refresh_interval);

  DrivingState driving_state{lane_index_, lane_index_, kinematics, State::KEEP_LANE};

  // calculate the cost of the target driving state
  const float cost = CalculateDrvingStateCost(driving_context_, *this, predictions, driving_state);

  return std::make_pair(driving_state, cost);
}

const std::pair<DrivingState, float> Vehicle::EvaluatePrepareLaneChangeState(
  const State& target_state, const vector<Vehicle>& predictions) const{

  assert((target_state == State::PREP_LANE_CHANGE_LEFT && lane_index_ > 0 ) ||
          (target_state == State::PREP_LANE_CHANGE_RIGHT && lane_index_ < driving_context_->total_lanes));

  const uint8_t intended_lane_index = lane_index_ +
    ((target_state == State::PREP_LANE_CHANGE_LEFT) ? -1 : 1);

  const DrivingKinematics ego_lane_kinematics = CalculateVehicleKinematics(
    predictions, lane_index_, driving_context_->ego_postion_refresh_interval);

  // TODO: calculate target position data to smoothly match proper gap in target lane behind ego!!!
  const DrivingKinematics intended_lane_kinematics = CalculateVehicleKinematics(
      predictions, intended_lane_index, driving_context_->ego_postion_refresh_interval);


  DrivingKinematics optimal_kinematics;
  std::shared_ptr<Vehicle> detected_vehicle_behind = nullptr;

  // assumption: ego should not accelerate speed when switching lane

   bool tailgaiting_vehicle_behind_detected =
      GetClosestVehicleInLane(lane_index_, predictions, false, detected_vehicle_behind) &&
        IsVehicleTooClose(*detected_vehicle_behind);
   if (intended_lane_kinematics.velocity < ego_lane_kinematics.velocity &&
       !tailgaiting_vehicle_behind_detected) {
      // we slow down to the safe speed of target lane (to safely switch lane behind the car in front in the  target lane)
      optimal_kinematics = intended_lane_kinematics;
   } else  {
      // we keep driving with safe speed behind the front car or with max allowed speed if no car in front
      optimal_kinematics = ego_lane_kinematics;
   }

  DrivingState driving_state{lane_index_, intended_lane_index, optimal_kinematics, target_state};

  // calculate the cost of the target driving state
  const float cost = CalculateDrvingStateCost(driving_context_, *this, predictions, driving_state);

  // create instance of target driving state
  return std::make_pair(driving_state, cost);
}

const std::pair<DrivingState, float> Vehicle::EvaluateLaneChangeState(
  const State& target_state, const vector<Vehicle>& predictions) const {

  assert((target_state == State::LANE_CHANGE_LEFT && lane_index_ > 0 ) ||
          (target_state == State::LANE_CHANGE_RIGHT && lane_index_ < driving_context_->total_lanes));

  const uint8_t target_lane_index = lane_index_ + ((target_state == State::LANE_CHANGE_LEFT) ? -1 : 1);


  const DrivingKinematics target_lane_kinematics = CalculateVehicleKinematics(
      predictions, target_lane_index, driving_context_->ego_postion_refresh_interval);

  DrivingState driving_state{target_lane_index, target_lane_index, target_lane_kinematics, target_state};

    // calculate the cost of the target driving state
  const float cost = CalculateDrvingStateCost(driving_context_, *this, predictions, driving_state);

  return std::make_pair(driving_state, cost);
}

bool Vehicle::IsLaneChangePossible(const uint8_t& target_lane_index,
  const vector<Vehicle>& predictions) const{

  // check if another vehicle occupies the 'safe lane change' segment of target lane
  bool lane_chnage_possible = true;
  std::shared_ptr<Vehicle> vehicle_ahead = nullptr;
  std::shared_ptr<Vehicle> vehicle_behind = nullptr;

  std::cout << "Checking safety conditions for changing to lane " <<
            (int) target_lane_index << "..." << std::endl;
  if (GetClosestVehicleInLane(target_lane_index, predictions, true, vehicle_ahead) &&
      IsVehicleTooClose(*vehicle_ahead)) {
    std::cout << "Change to lane " << (int) target_lane_index <<
                 " is not possible! vehicle ahead is too close!" << std::endl;
    lane_chnage_possible = false;
  } else if (GetClosestVehicleInLane(target_lane_index, predictions, false, vehicle_behind)
             &&  this->s_- vehicle_behind->s_ < driving_context_->safe_distance_to_vehicle_behind ){
             // disabled dynamic check for close driving Vehicle behind ego and use the static constant value
             // TODO: improve IsVehicleTooClose() function
             // && IsVehicleTooClose(*vehicle_behind)) {
    std::cout << "Change to lane " << (int) target_lane_index <<
                 " is not possible! Vehicle behind is too close!" << std::endl;
    lane_chnage_possible = false;
  } else {
    std::cout << "Change to lane " << (int) target_lane_index << " is safe!"<< std::endl;
  }

  return lane_chnage_possible;
}

const DrivingKinematics Vehicle::CalculateVehicleKinematics(const vector<Vehicle>& predictions,
  const uint8_t target_lane_index, const double& time_interval) const {

  const double max_velocity_increment =
    std::min((velocity_ + driving_context_->max_accceleration*time_interval), driving_context_->max_speed);
  const double max_velocity_decrement =
    std::max(velocity_ - driving_context_->max_deceleration*time_interval, 0.0) ;

  double new_velocity;
  double new_acceleration;
  std::shared_ptr<Vehicle> vehicle_ahead = nullptr;
  std::shared_ptr<Vehicle> vehicle_behind = nullptr;

  //std::cout << "Calculating ego vehilce kinematics...";
  if (GetClosestVehicleInLane(target_lane_index, predictions, true, vehicle_ahead) &&
      IsVehicleTooClose(*vehicle_ahead)) {

      new_velocity = max_velocity_decrement;
    //}
  } else {
    new_velocity = max_velocity_increment;
  }
  // std::cout << "Calculated target ego vehicle speed: " << new_velocity * kMpsToMphRatio << "mph" << std::endl;

  new_acceleration = (new_velocity - velocity_)/time_interval; // Equation: (v_1 - v_0)/t = acceleration
  // std::cout << "Calculated target ego vehicle acceleration: " << new_acceleration << "m/s" << std::endl;


  return {new_velocity, new_acceleration};
}

bool Vehicle::GetClosestVehicleInLane(const uint8_t lane_index, const vector<Vehicle>& predictions,
  const bool is_ahead_of_ego, std::shared_ptr<Vehicle>& detected_vehicle) const {

  bool ego_lane_vehicle_detected = false;

  // WARNING!! because of reset of s in start/finish line to 0 there is a slight chance that
  // vehicle may not be detected being ahead/behind of ego car.
  // Need some smart handling of this corner case
  double closest_vehicle_s = is_ahead_of_ego ? std::numeric_limits<double>::max() : 0;

  for (Vehicle vehicle : predictions) {

    bool is_in_target_lane = (vehicle.lane_index_ == lane_index);
    bool is_longitudinal_position_correct = is_ahead_of_ego ?
      (vehicle.s_ >= this->s_) && (vehicle.s_ < closest_vehicle_s ) :
      (vehicle.s_ < this->s_) && (vehicle.s_ > closest_vehicle_s);


    if (is_in_target_lane && is_longitudinal_position_correct) {

      closest_vehicle_s = vehicle.s_;
      detected_vehicle = std::make_shared<Vehicle>(std::move(vehicle));
      ego_lane_vehicle_detected = true;
    }
  }

  return ego_lane_vehicle_detected;
}

double Vehicle::GetSafeForesightDistance() const {

  double moving_duration = driving_context_->safe_time_to_maneuver_start;
  double safe_distance =  this->velocity_ * moving_duration + 0.0 * moving_duration * moving_duration / 2;
  // NOTE! ego vehilce's real time acceleration is ignored for the moment, coz is causing big amplitude oscillations
  // of safe foresight distance when car is driving behind slow car (acceleration value jumping constantly from "-"
  // to "+" values)
  // TODO: use average ego vehicle's acceleration over several update cycles of 0.02sec (e.g 50)

  return safe_distance;
}

double Vehicle::CalculateLaneSpeed(const vector<Vehicle>& predictions, const uint8_t lane_index) const {
  // As a lane speed use the speed of closest vehicle in front , or a max allowed speed if the detected vehicle
  //exceeds one or there was no vehicle in front detected  within configured foresight buffer
  // Note! Detection of fast approaching vehicle behind should be done in IsLaneChangePossible function
  std::shared_ptr<Vehicle> detected_vehicle_ahead = nullptr;
  if (GetClosestVehicleInLane(lane_index, predictions, true, detected_vehicle_ahead) &&
      detected_vehicle_ahead->s_ - this->s_ < GetSafeForesightDistance()) {
      double lane_speed = detected_vehicle_ahead->velocity_;

    std::cout << "  Target Lane " << (int) lane_index << ": Distance to vehicle ahead: " <<
      detected_vehicle_ahead->s_ - this->s_ << " m, Speed of vehicle ahead: " <<
      detected_vehicle_ahead->velocity_ * kMpsToMphRatio << " mph, Calculated lane speed: " <<
      lane_speed * kMpsToMphRatio << " mph" << std:: endl;

      return lane_speed;
  }
  // Found no vehicle in the lane
  std::cout << "  Target Lane " << (int) lane_index << " info: Distance to vehicle ahead: UNKNOWN, " <<
    "Calculated lane speed: " << driving_context_->max_speed * kMpsToMphRatio << " mph (max allowed)" <<
    std:: endl;
  return driving_context_->max_speed;
}

bool Vehicle::IsVehicleTooClose(const Vehicle& vehicle) const {

  // TODO: this is very simple rule. better to use max allowed deceleration for calculating safe distance to the car
  return std::abs(vehicle.s_ - this->s_) < (vehicle.s_ > this->s_ ?
    this->velocity_ : (vehicle.velocity_)) * driving_context_->safe_ratio_distance_to_speed;
}

const pair<vector<double>, vector<double>> Vehicle::CalculateDrivingTrajectory(
  const vector<pair<double, double>>& remaining_previous_trajectory,
  const double& remaining_previous_trajectory_end_s, const DrivingState& target_driving_state) const{

  // build smooth trajectory using points from previous trajectory
  double ref_x = x_;
  double ref_y = y_;
  double ref_yaw = deg2rad(yaw_);

  // anchor points for generating new trajectory through
  vector<double> anchor_points_x;
  vector<double> anchor_points_y;

  const std::size_t previous_trajectory_size = remaining_previous_trajectory.size();
  if(previous_trajectory_size < 2) { //if previous path is almost empty use ccp as start point

    // calculate previous waypoints using current ego's yaw
    double previous_ego_position_x = x_ - cos(yaw_);
    double previous_ego_position_y =y_ - sin(yaw_);

    // use two last waypoints from previous trajectory for interpolation to make transition smoother
    anchor_points_x.push_back(previous_ego_position_x);
    anchor_points_x.push_back(x_);

    anchor_points_y.push_back(previous_ego_position_y);
    anchor_points_y.push_back(y_);

  } else { // otherwise use last point of previous trajectory as reference waypoint

    ref_x = remaining_previous_trajectory[previous_trajectory_size - 1].first;
    ref_y = remaining_previous_trajectory[previous_trajectory_size - 1].second;
    double ref_x_previous = remaining_previous_trajectory[previous_trajectory_size - 2].first;
    double ref_y_previous = remaining_previous_trajectory[previous_trajectory_size - 2].second;
    ref_yaw = atan2(ref_y - ref_y_previous, ref_x - ref_x_previous);

    // use two last  waypoints from previous path for interpolation to make transition smoother
    anchor_points_x.push_back(ref_x_previous);
    anchor_points_x.push_back(ref_x);

    anchor_points_y.push_back(ref_y_previous);
    anchor_points_y.push_back(ref_y);
  }

  // set the starting point of new trajectory generation to last point of previous trajectory
  // since all previous points are just inherited by new trajectory
  double new_trajectory_points_start_s = (previous_trajectory_size > 0) ?
    remaining_previous_trajectory_end_s : s_;

  // generate key points of new trajectory (to interpolate through them later)
  for (int i = 0; i < kTotalTrajectoryAnchorPoints; i++) {

    pair<double, double> next_key_point =
      getXY(new_trajectory_points_start_s + (i+1) * kDistanceBetweenTrajectoryKeyPoints,
         GetLaneCenterLineD(target_driving_state.target_lane_index, driving_context_->lane_width),
         driving_context_->map_waypoints_s,
         driving_context_->map_waypoints_x, driving_context_->map_waypoints_y);
    anchor_points_x.push_back(next_key_point.first);
    anchor_points_y.push_back(next_key_point.second);
  }

  // change reference angle to local coordinates system frame
  for (int i = 0; i < anchor_points_x.size(); i++) {
    double shift_x = anchor_points_x[i] - ref_x;
    double shift_y = anchor_points_y[i] - ref_y;

    anchor_points_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    anchor_points_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
  }


  vector<double> trajectory_x;
  vector<double> trajectory_y;

  // use remaining waypoints from previous trajectory as part of new trajectory
  for (int i = 0; i < previous_trajectory_size; i++){
    trajectory_x.push_back(remaining_previous_trajectory[i].first);
    trajectory_y.push_back(remaining_previous_trajectory[i].second);
  }

  // create spline for interpolation
  tk::spline trajectory_spline;
  trajectory_spline.set_points(anchor_points_x, anchor_points_y);

  // calculate trajectory points by breaking down the distance between anchor points into segments on the spline
  // in the way that ego velocity will not exceed speed limit
  double target_x = kDistanceBetweenTrajectoryKeyPoints;
  double target_y = trajectory_spline(target_x);
  const double target_dist = sqrt(target_x*target_x + target_y*target_y);
  const double total_segments = target_dist /
    (driving_context_->ego_postion_refresh_interval * target_driving_state.kinematics.velocity);


  //fill in the rest of trajectory points
  double updated_x = 0;
  for (int i = 0; i < (kTotalTrajectoryPoints - remaining_previous_trajectory.size()); i++) {
    double point_x = updated_x + target_dist / total_segments;
    double point_y = trajectory_spline(point_x);

    updated_x = point_x;

    double x_tmp = point_x;
    double y_tmp = point_y;

    // change reference angle back to global coordinates system frame
    point_x = x_tmp * cos(ref_yaw) - y_tmp * sin(ref_yaw);
    point_y = x_tmp * sin(ref_yaw) + y_tmp * cos(ref_yaw);

    point_x+= ref_x;
    point_y+= ref_y;


    trajectory_x.push_back(point_x);
    trajectory_y.push_back(point_y);

  }

  return std::make_pair(trajectory_x,trajectory_y);
}
