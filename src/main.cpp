#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#include <utility>
#include <math.h>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;


  const int kStartLane = 1; //start counting from 0 t(left most lane)
  static const double kReferenceVelocity = 49.5; //mph
  static const float kSafeDistanceToSpeedRatio = 0.5;
  static const double kMphToMpsRatio = 2.24;
  const int kTotalTrajectoryPoints = 50;
  const int kTotalTrajectoryAnchorPoints = 5;
  const int kDistanceBetweenTrajectoryKeyPoints = 30; // in meters
  const int kLaneWidth = 4; // in meters
  static const double kCarPositionRefreshTime = 0.02; // in seconds
  static const double kDecelerationConformLimit = 5; // in m/s2
  static const double kAccelerationConformLimit = 9; // in m/s2

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  int ego_lane = kStartLane;
  double target_ego_velocity = 0; // in m/s. we start smoothly to prevent max acceleration exceed

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &target_ego_velocity, &ego_lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          // DEFINE SAFE SPEED FOR CURRENT CAR MOVE

          const std::size_t previous_path_size = previous_path_x.size();
          if(previous_path_size > 0) {
            car_x = end_path_s;
          }
          bool slow_car_ahead = false;
          for (int i = 0; i < sensor_fusion.size(); i++){
            //check whether detected car is in our lane
            const float d = sensor_fusion[i][6];
            const int lane_center_line = ego_lane * kLaneWidth + kLaneWidth/2;
            if ((d > (lane_center_line - kLaneWidth/2)) && (d <(lane_center_line + kLaneWidth/2))){

              const double v_x = sensor_fusion[i][3];
              const double v_y = sensor_fusion[i][4];
              const double detected_car_velocity = sqrt(v_x*v_x + v_y*v_y);
              double detected_car_s = sensor_fusion[i][5];

              //project longitudinal position of detected car into the future:
              // where detected car would be if ego car position would evolve to the last point of previous path
              detected_car_s+=(double)previous_path_size * kCarPositionRefreshTime * detected_car_velocity;
              if((detected_car_s > car_s) && ((detected_car_s - car_s) <
                target_ego_velocity * kSafeDistanceToSpeedRatio * kMphToMpsRatio)) {
                // if detected car is in front of ego and is too close
                slow_car_ahead = true;
              }
            }
          }

          // adapt the ego speed smoothly to prevent max acceleration/deceleration exceed
          if(slow_car_ahead) {
            //  change lane if possible
            //if (LaneChangeFeasable()) {
              // ChangeLane();
            //} else { // or decrease the speed and wait for opportunity to change lane
              target_ego_velocity-= kCarPositionRefreshTime * kDecelerationConformLimit * kMphToMpsRatio;
              // PrepareForLaneChange();
            //}
          } else if (target_ego_velocity < kReferenceVelocity) {
            target_ego_velocity+= kCarPositionRefreshTime * kAccelerationConformLimit * kMphToMpsRatio;
          }


          // KEEP LANE WITH SAME SPEED

          // build smooth trajectory using points from previous path
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          vector<double> anchor_points_x;
          vector<double> anchor_points_y;

          if(previous_path_size < 2) { //if previous path is almost empty use ccp as start point

            // calculate previous waypoints using current car yaw
            double previous_car_x = car_x - cos(car_yaw);
            double previous_car_y = car_y - sin(car_yaw);

            // use two last  waypoints from previous path for interpolation to make transition smoother
            anchor_points_x.push_back(previous_car_x);
            anchor_points_x.push_back(car_x);

            anchor_points_y.push_back(previous_car_y);
            anchor_points_y.push_back(car_y);

          } else { // otherwise use last point of previous path as reference waypoint

            ref_x = previous_path_x[previous_path_size - 1];
            ref_y = previous_path_y[previous_path_size - 1];
            double ref_x_previous = previous_path_x[previous_path_size - 2];
            double ref_y_previous = previous_path_y[previous_path_size - 2];
            ref_yaw = atan2(ref_y - ref_y_previous, ref_x - ref_x_previous);

            // use two last  waypoints from previous path for interpolation to make transition smoother
            anchor_points_x.push_back(ref_x_previous);
            anchor_points_x.push_back(ref_x);

            anchor_points_y.push_back(ref_y_previous);
            anchor_points_y.push_back(ref_y);
          }

          // generate key points of new trajectory (to interpolate through them later)
          for (int i = 0; i < kTotalTrajectoryAnchorPoints; i++) {

            std::pair<double,double> next_key_point = getXY(car_s + (i+1) * kDistanceBetweenTrajectoryKeyPoints,
              kLaneWidth/2 + kLaneWidth * ego_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
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
          for (int i = 0; i < previous_path_size; i++){
            trajectory_x.push_back(previous_path_x[i]);
            trajectory_y.push_back(previous_path_y[i]);
          }

          // calculate trajectory points by breaking down the distance between anchor points into segments on the spline
          // in the way that ego velocity will not exceed speed limit
          double target_x = kDistanceBetweenTrajectoryKeyPoints;
          double target_y = trajectory_spline(target_x);
          const double target_dist = sqrt(target_x*target_x + target_y*target_y);
          const double total_segments = target_dist / (kCarPositionRefreshTime * target_ego_velocity / kMphToMpsRatio);


          //fill in the rest of trajectory points
          double updated_x = 0;
          for (int i = 0; i < (kTotalTrajectoryPoints - previous_path_size); i++) {
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

          msgJson["next_x"] = trajectory_x;
          msgJson["next_y"] = trajectory_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
