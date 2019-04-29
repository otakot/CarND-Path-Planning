#include "vehicle.h"
#include "cost.h"
#include "utilities.h"
#include "logger.h"



#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <limits>
#include <iostream>
#include <math.h>

using std::string;
using std::vector;


DrivingState Vehicle::CalculateNextOptimalDrivingState(const vector<Vehicle>& predictions) {

  vector<State> sucessor_states = GetFeasibleSuccessorStates(predictions);

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

vector<State> Vehicle::GetFeasibleSuccessorStates(const vector<Vehicle> &predictions) {

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

bool Vehicle::IsLaneChangePossible(const uint8_t& target_lane_index, const vector<Vehicle>& predictions){
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

DrivingState Vehicle::CreateDrivingState(const State& target_state, const vector<Vehicle>& predictions) {
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

DrivingState Vehicle::CreateConstantSpeedState() {
  // Generate a constant speed trajectory.

  // TODO: calculate target position data for longer period of time (e.g 150 m drive) !!!
  double target_s = PredictLongitudinalPosition(1);  // 1sec for now. it's temp solution
  return DrivingState{lane_index_, {target_s, velocity_, 0}, state_};
}

DrivingState Vehicle::CreateKeepLaneState(const vector<Vehicle>& predictions) {

  // TODO: calculate target position data for longer period of time (e.g 150 m drive) !!!
  DrivingKinematics kinematics = CalculateVehilceKinematics(
    predictions, lane_index_, driving_context_->ego_postion_refresh_interval_);
  return DrivingState{lane_index_, kinematics, State::KEEP_LANE};
}

DrivingState Vehicle::CreatePrepareLaneChangeState(const State& target_state, const vector<Vehicle>& predictions) {

  assert((target_state == State::PREP_LANE_CHANGE_LEFT && lane_index_ > 0 ) ||
          (target_state == State::PREP_LANE_CHANGE_RIGHT && lane_index_ < driving_context_->total_lanes_));


  uint8_t target_lane_index = lane_index_ + ((target_state == State::PREP_LANE_CHANGE_LEFT) ? -1 : 1);

  // TODO: calculate target position data for longer period of time (e.g 4 seconds) !!!
  DrivingKinematics ego_lane_kinematics = CalculateVehilceKinematics(
    predictions, lane_index_, driving_context_->ego_postion_refresh_interval_);

  std::shared_ptr<Vehicle> detected_vehicle_behind = nullptr;
  if (GetClosestVehicleInLane(lane_index_, predictions, false, detected_vehicle_behind)) {
    // Keep current speed
    return DrivingState{lane_index_, ego_lane_kinematics, target_state};

  } else {

     // TODO: calculate target position data to smoothly match proper gap in target lane behind ego!!!
     // TODO: change time interval to bigger one!!
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

DrivingState Vehicle::CreateLaneChangeState(const State& target_state, const vector<Vehicle> &predictions) {

  // Generate a lane change trajectory.
  assert((target_state == State::LANE_CHANGE_LEFT && lane_index_ > 0 ) ||
          (target_state == State::LANE_CHANGE_RIGHT && lane_index_ < driving_context_->total_lanes_));

  uint8_t target_lane_index = lane_index_ + ((target_state == State::LANE_CHANGE_LEFT) ? -1 : 1);


  // TODO: calculate target position data for longer period of time (e.g 150 m drive) !!!
  DrivingKinematics target_lane_kinematics = CalculateVehilceKinematics(
      predictions, target_lane_index, driving_context_->ego_postion_refresh_interval_);

  return DrivingState{target_lane_index, target_lane_kinematics, target_state};
}

DrivingKinematics Vehicle::CalculateVehilceKinematics(
    const vector<Vehicle>& predictions, const uint8_t target_lane_index, const double time_interval) {

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

bool Vehicle::GetClosestVehicleInLane(const uint8_t lane_index,
  const vector<Vehicle>& predictions, const bool is_ahead_of_ego, std::shared_ptr<Vehicle>& detected_vehicle) {

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

bool Vehicle::IsVehicleTooClose(const Vehicle& vehicle) {
  return std::abs(vehicle.s_ - this->s_) <
    (vehicle.s_ > this->s_ ? this->velocity_ : vehicle.velocity_) *
      driving_context_->safe_distance_to_speed_ratio_;
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
