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

using std::string;
using std::vector;
using std::pair;


const DrivingState Vehicle::CalculateNextOptimalDrivingState(const vector<Vehicle>& predictions) const {

  const vector<State> sucessor_states = GetFeasibleSuccessorStates(predictions);

  vector<float> costs;

  vector<DrivingState> feasible_next_driving_states;
  std::cout << "Calculating feasable driving states for transition..."  << std::endl;
  for (const State state_id : sucessor_states) {
    // generate the driving parameters for feasable state
    const DrivingState possible_driving_state = CreateDrivingState(state_id, predictions);

    // calculate the cost of the sate
    float cost = CalculateTargetStateCost(driving_context_, *this, predictions, possible_driving_state);

    std::cout << LogDrivingState(possible_driving_state) << ", Cost: "  << cost << std::endl;

    costs.push_back(cost);
    feasible_next_driving_states.push_back(possible_driving_state);
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  std::uint16_t optimal_state_index = distance(begin(costs), best_cost);

  DrivingState optimal_next_driving_state = feasible_next_driving_states[optimal_state_index];
  std::cout << "Target state selected : "  << GetStateName(optimal_next_driving_state.state) << std::endl;
  return optimal_next_driving_state;
}

const vector<State> Vehicle::GetFeasibleSuccessorStates(const vector<Vehicle> &predictions) const {

  vector<State> states;
  states.push_back(State::KEEP_LANE);
  switch (state_) {
    case  State::KEEP_LANE:
      states.push_back(State::PREP_LANE_CHANGE_LEFT);
      states.push_back(State::PREP_LANE_CHANGE_RIGHT);
      break;
    case State::PREP_LANE_CHANGE_LEFT:
      if (lane_index_ != driving_context_->total_lanes_ - 1) {
        states.push_back(State::PREP_LANE_CHANGE_LEFT);
        if(IsLaneChangePossible(lane_index_ - 1, predictions)){
          states.push_back(State::LANE_CHANGE_LEFT);
        }
      }
      break;
    case State::PREP_LANE_CHANGE_RIGHT:
      if (lane_index_ != 0) {
        states.push_back(State::PREP_LANE_CHANGE_RIGHT);
        if(IsLaneChangePossible(lane_index_ + 1, predictions)){
          states.push_back(State::LANE_CHANGE_RIGHT);
        }
      }
      break;
    default:
     break;
     // If state is "LANE_CHANGE_LEFT" or "LANGE_CHANGE_RIGHT", then just return "KEEP_LANE" for now.
  }
  return states;
}

bool Vehicle::IsLaneChangePossible(const uint8_t& target_lane_index, const vector<Vehicle>& predictions) const{

  // check if another vehicle occupies the 'safe lane change' segment of target lane
  for (Vehicle vehicle : predictions) {
   bool is_in_tagret_lane = (vehicle.lane_index_ == target_lane_index);
   bool is_longitudinally_behind = (this->s_ > vehicle.s_);

   //TODO: make safe distance calculation more sofisticated. e.g. based on max comfort deceleration
   bool is_rear_vehile_too_close = (this->s_ - vehicle.s_)  <
      vehicle.velocity_ * driving_context_->safe_distance_to_speed_ratio_;
   bool is_front_vehile_too_close = (vehicle.s_ - this->s_)  <
      velocity_ * driving_context_->safe_distance_to_speed_ratio_;

    if (is_in_tagret_lane &&
        ((is_longitudinally_behind && is_rear_vehile_too_close) ||
         (!is_longitudinally_behind && is_front_vehile_too_close))) {
      // lane change is not safe
      return false;
    }
  }
  return true;
}

const DrivingState Vehicle::CreateDrivingState(
  const State& target_state, const vector<Vehicle>& predictions) const {
  // Given a possible next state, generate the appropriate trajectory to realize  the next state.
  switch (target_state) {
    case State::CONSTANT_SPEED:
      return CreateConstantSpeedState();
    case State::KEEP_LANE:
      return CreateKeepLaneState(predictions);
    case State::LANE_CHANGE_LEFT:
    case State::LANE_CHANGE_RIGHT:
      return CreateLaneChangeState(target_state, predictions);
    case State::PREP_LANE_CHANGE_LEFT:
    case State::PREP_LANE_CHANGE_RIGHT:
      return CreatePrepareLaneChangeState(target_state, predictions);
    default:
      throw std::runtime_error("Unknown target state");
  }
}

const DrivingState Vehicle::CreateConstantSpeedState() const{
  // Generate a constant speed trajectory.

  // TODO: calculate target position data for longer period of time (e.g 150 m drive) !!!
  double target_s = PredictLongitudinalPosition(1);  // 1sec for now. it's temp solution
  return DrivingState{lane_index_, {target_s, velocity_, 0}, state_};
}

const DrivingState Vehicle::CreateKeepLaneState(const vector<Vehicle>& predictions) const{

  // TODO:  maybe change time interval to bigger one?
  DrivingKinematics kinematics = CalculateVehilceKinematics(
    predictions, lane_index_, driving_context_->ego_postion_refresh_interval_);
  return DrivingState{lane_index_, kinematics, State::KEEP_LANE};
}

const DrivingState Vehicle::CreatePrepareLaneChangeState(
  const State& target_state, const vector<Vehicle>& predictions) const{

  assert((target_state == State::PREP_LANE_CHANGE_LEFT && lane_index_ > 0 ) ||
          (target_state == State::PREP_LANE_CHANGE_RIGHT && lane_index_ < driving_context_->total_lanes_));


  const uint8_t target_lane_index = lane_index_ + ((target_state == State::PREP_LANE_CHANGE_LEFT) ? -1 : 1);

  // TODO: calculate target position data for longer period of time (e.g 4 seconds) !!!
  const DrivingKinematics ego_lane_kinematics = CalculateVehilceKinematics(
    predictions, lane_index_, driving_context_->ego_postion_refresh_interval_);

  std::shared_ptr<Vehicle> detected_vehicle_behind = nullptr;
  if (GetClosestVehicleInLane(lane_index_, predictions, false, detected_vehicle_behind)) {
    // Keep current speed
    return DrivingState{lane_index_, ego_lane_kinematics, target_state};

  } else {

     // TODO: calculate target position data to smoothly match proper gap in target lane behind ego!!!
     // TODO: maybe change time interval to bigger one?
    DrivingKinematics target_lane_kinematics = CalculateVehilceKinematics(
      predictions, target_lane_index, driving_context_->ego_postion_refresh_interval_);

    // If lane speed of target lane is lower than ego speed -
    // then smoothly slow down to match it to prevent potential high deceleration during lane change
    if (target_lane_kinematics.velocity < ego_lane_kinematics.velocity) {
      return DrivingState{lane_index_, target_lane_kinematics, target_state};
    } else { // otherwise keep driving with current speed and wait for proper size gap in target lane
      return DrivingState{lane_index_, ego_lane_kinematics, target_state};
    }
  }
}

const DrivingState Vehicle::CreateLaneChangeState(
  const State& target_state, const vector<Vehicle> &predictions) const {

  assert((target_state == State::LANE_CHANGE_LEFT && lane_index_ > 0 ) ||
          (target_state == State::LANE_CHANGE_RIGHT && lane_index_ < driving_context_->total_lanes_));

  const uint8_t target_lane_index = lane_index_ + ((target_state == State::LANE_CHANGE_LEFT) ? -1 : 1);


  // TODO: calculate target position data for longer period of time (e.g 150 m drive) !!!
  const DrivingKinematics target_lane_kinematics = CalculateVehilceKinematics(
      predictions, target_lane_index, driving_context_->ego_postion_refresh_interval_);

  return DrivingState{target_lane_index, target_lane_kinematics, target_state};
}

const DrivingKinematics Vehicle::CalculateVehilceKinematics(
    const vector<Vehicle>& predictions, const uint8_t target_lane_index, const double time_interval) const {

  const double max_velocity_increment =
    std::min((velocity_ + driving_context_->max_accceleration_*time_interval), driving_context_->max_speed_);
  const double max_velocity_decrement =
    std::max(velocity_ - driving_context_->max_deceleration_*time_interval, 0.0) ;

  double predicted_position;
  double new_velocity;
  double new_acceleration;
  std::shared_ptr<Vehicle> vehicle_ahead = nullptr;
  std::shared_ptr<Vehicle> vehicle_behind = nullptr;

  //std::cout << "Calculating ego vehilce kinematics...";
  if (GetClosestVehicleInLane(target_lane_index, predictions, true, vehicle_ahead) &&
      IsVehicleTooClose(*vehicle_ahead)) {

    // std::cout << "Slow Vehicle ahead detected!" <<std::endl;
    if (GetClosestVehicleInLane(target_lane_index, predictions, false, vehicle_behind) &&
      IsVehicleTooClose(*vehicle_behind)) {
      //std::cout << "Vehicle behind in ego lane in " << (int)(vehicle_ahead->s_ - this->s_) <<"m" <<std::endl;
      // must travel at the speed of traffic, regardless of preferred buffer
      new_velocity = vehicle_ahead->velocity_;
    } else {

    // this is weird way of calculation of  target seed. but wil lleave it here for now
    //      double max_velocity_in_front = (vehicle_ahead->s_ - this->s_
    //                                  - kPreferredBuffer) + vehicle_ahead->velocity_
    //                                  - 0.5 * (acceleration_);
    //      std::cout << "Max velocity in front: " << max_velocity_in_front <<"m/s" <<std::endl;
    //      new_velocity = std::min(std::min(max_velocity_in_front,
    //                                       max_velocity_increment),
    //                                       driving_context_->max_speed_);
      new_velocity = max_velocity_decrement;
    }
  } else {
    new_velocity = max_velocity_increment;
  }
  // std::cout << "Calculated target ego vehicle speed: " << new_velocity * kMpsToMphRatio << "mph" << std::endl;

  new_acceleration = (new_velocity - velocity_)/time_interval; // Equation: (v_1 - v_0)/t = acceleration
  // std::cout << "Calculated target ego vehicle acceleration: " << new_acceleration << "m/s" << std::endl;

  predicted_position = s_ + new_velocity * time_interval + new_acceleration * time_interval*time_interval / 2.0;
  //std::cout << "Calculated target ego vehicle s poistion: " << predicted_position <<"m" << std::endl;

  return {predicted_position, new_velocity, new_acceleration};
}

double Vehicle::PredictLongitudinalPosition(const double time_interval) const{
  return s_ + velocity_*time_interval + acceleration_*time_interval*time_interval/2.0;
}

bool Vehicle::GetClosestVehicleInLane(const uint8_t lane_index, const vector<Vehicle>& predictions,
  const bool is_ahead_of_ego, std::shared_ptr<Vehicle>& detected_vehicle) const {

  bool ego_lane_vehicle_detected = false;

  // WARNING!! because of reset of s in start/finish line to 0 there is a slight chance that
  // vehicle may not be detected being ahead/behind of ego car.
  // Need some smart handling of this corner case
  double closest_vehicle_s = is_ahead_of_ego ? std::numeric_limits<double>::max() : 0;


  for (Vehicle vehicle : predictions) {


    bool is_in_ego_lane = (vehicle.lane_index_ == this->lane_index_);
    bool is_longitudinal_position_correct = is_ahead_of_ego ?
      (vehicle.s_ >= this->s_) && (vehicle.s_ < closest_vehicle_s ) :
      (vehicle.s_ < this->s_) && (vehicle.s_ > closest_vehicle_s);


    if (is_in_ego_lane && is_longitudinal_position_correct) {

      closest_vehicle_s = vehicle.s_;
      detected_vehicle = std::make_shared<Vehicle>(std::move(vehicle));
      ego_lane_vehicle_detected = true;
    }
  }

  return ego_lane_vehicle_detected;
}

bool Vehicle::IsVehicleTooClose(const Vehicle& vehicle) const {
  return std::abs(vehicle.s_ - this->s_) <
    (vehicle.s_ > this->s_ ? this->velocity_ : vehicle.velocity_) *
      driving_context_->safe_distance_to_speed_ratio_;
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
        GetLaneCenterLineD(target_driving_state.lane_index, driving_context_->lane_width_),
         driving_context_->map_waypoints_s_, driving_context_->map_waypoints_x_, driving_context_->map_waypoints_y_);
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

  // create spline for interpolation
  tk::spline trajectory_spline;
  trajectory_spline.set_points(anchor_points_x, anchor_points_y);


  vector<double> trajectory_x;
  vector<double> trajectory_y;

  // use remaining waypoints from previous trajectory as part of new trajectory
  for (int i = 0; i < previous_trajectory_size; i++){
    trajectory_x.push_back(remaining_previous_trajectory[i].first);
    trajectory_y.push_back(remaining_previous_trajectory[i].second);
  }

  // calculate trajectory points by breaking down the distance between anchor points into segments on the spline
  // in the way that ego velocity will not exceed speed limit
  double target_x = kDistanceBetweenTrajectoryKeyPoints;
  double target_y = trajectory_spline(target_x);
  const double target_dist = sqrt(target_x*target_x + target_y*target_y);
  const double total_segments = target_dist /
    (driving_context_->ego_postion_refresh_interval_ * target_driving_state.kinematics.velocity);


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

///**
// * Sets state and kinematics for ego vehicle.
// */
//void Vehicle::ApplyTargetState(const DrivingState &target_state) {
//
//  state_ = target_state.state;
//  lane_index_ = target_state.lane_index;
//  s_ = target_state.s;
//  velocity_ = target_state.velocity;
//  acceleration_ = target_state.acceleration;
//}
