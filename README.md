# Path Planning project

This repository contains implementation of the Path Planner application that allows to control the car
which is driving on the highway type of road in the simulated environment. 

### Communication between Road Simulator and Path Planning application

The telemetry data which provided by the Simulator to the Path Planning application with a frequency of
50 updates per second has following structure:

#### Ego vehicle localization data

["x"] - X position of ego vehicle in map coordinate system
["y"] - Y position of ego vehicle in map coordinate system
["s"] - S position of ego vehicle in Frenet coordinates
["d"] - D position of ego vehicle in Frenet coordinates
["yaw"] -  yaw angle of ego vehicle in map coordinate system
["speed"] - Speed of ego vehicle

#### Previous driving trajectory data

The provided ego vehicle's previous trajectory will not contain the waypoints which ego vehicle was not
able to cover during the time frame between current and previous events of communication with the Road
Simulator.

["previous_path_x"] - The X points of previous ego vehicle trajectory (in map coordinate system frame)
["previous_path_y"] - The Y points of previous ego vehicle trajectory (in map coordinate system frame)

#### S and D values of previous driving trajectory

["end_path_s"] - The S value of the end of previous trajectory (in Frenet coordinates)
["end_path_d"] - The D value of the end of previous trajectory (in Frenet coordinates)

#### Sensor Fusion data 

During each communication event the simulator also provides the list of driving attributes of all other
vehicles detected on the same side of the road around the ego vehicle.

["sensor_fusion"] - A 2d vector of objects, each object represents detected vehicle on the road and contains
the following attributes:
  * unique ID of the vehicle
  * Y position (in map coordinates)
  * Y position  (in map coordinates)
  * velocity in X axis (in m/s)
  * velocity in Y axis (in m/s)
  * S position (in frenet coordinates)
  * D position (in frenet coordinates) 
  
#### Ego Vehilce control data wchich is sent back to Road Simulator

A list of waypoints represented by two separate arrays of X and Y coordinates is sent back by Path Planning application to Road Simulator periodically during every communication event. The waypoints represent the 
calculated optimal trajectory of ego vehicle. Vehilce controller used in Road Simulator every 0.02 will 
visit every next (x, y) waypoint from received list. The coordinates for (x,y) points are in global (map) 
coordinate system framere and are presented in meters, the distance between the points will determine the
speed of the ego car in Road Simulator. The vector going from a waypoint to the next waypoint in the list
also defines the heading of the car. Acceleration both in the tangential and normal directions is measured
along with the jerk (the amplitude of change of acceleration).


## Description of implemented Path Planning model

The path plannning model implemented in this application is fully based on handling the following 
requirements with respect to safety, comfort and efficiency of drive of ego vehicle:

### Implemented project requirements 

* Ego vehice avoids crossing the yellow lane that divides the driving directions of simulated highway.
 
* Ego vehicle always keep the lane center line of selected driving lane except of cases when ego
  vehicle is chaning the lane. The lane change duration does not exceed 5 seconds.
   
* Acceleration and jerk of ego vehice that follows the trajectory provided by Path Planner, does not
  exceed 10 m/s^2, and 50 m/s^3 accordingly.
   
* Velocity of ego vehicle is calculated to be as high as possible in the current road conditions (with 
  resepect to above specified requrements to maximum acceleration and jerk), but does not exceed 50 Miles
  per Hour.  
   
* The generated driving trajectory takes into account the position and speed of other vehicles detected 
  around the ego vehicle to minimize the risk of collision or violation of above mentioned requriements for
  maximum speed, acceleration and jerk of ego vehicle. 
    
* The generated driving trajectory aims the optimization of driving efficiency (permanent maximum proximity
  to biggest allowed driving speed) by overtaking the slower vehicles ahead. The overtake maneuvers are
  executed via changing to the 'faster; lane, if such lane change is possible in current driving conditions. 

### Application architecture description

High level execution workflow of Path Planning application is defined in main.cpp and can be logically
divided into following parts:

#### Preparation the data model of driving environment

This initial phase of application execution implementes the parsing the road model provided in text data 
file (highway_map.csv) (see lines 70-73 in main.cpp) and loading parsed road attributes (x, y, dx, dy, s) 
into corresponding data containers. Later on this road model wil be used for correct positioning of ego
vehicle on simulated road and generating of driving trajectory. Additionally a full list of driving 
parameters is loaded into Drivng Context class, which is used during whole application lifecycle.

#### Initialization of Ego Vehicle and start of driving

After creating the road model the instance of Ego vehicle is cretaed and initialized with default values
for such parameters as driving lane index, velocity, acceleration and initial state of the driving strategy
state maschine (see lines 81-84 in main.cpp). The ego vehicle is configured to start its movement from stand
still position in the middle highway lane (second out of three) and will slowly increaase its speed by
maininting the constant acceeration 9.5 m/s2 until the vehicle reaches the maximum allowed speed 50 miles
per hour. Afterwards vehicle will maintain this speed on a free road segments and decreaase it to in the
cases where maintaining such speed is not possible because of safety or comfort limitations.

#### Cyclic processing of telemetry data received from Drivng Simulator

The core part of the application is the funciton **Vehilce::ProcessTelemetryData()**
(see lines 37-66 in main.cpp) which is invoked every time when application receives a new portion of
telemetry data from running Road Simulator (see lines 87-115 in main.cpp). (the stucutre of telemetry data
package is descibed above). This function encapsulates the processing of received telemetry data and
generating of optimal driving trajectory for ego vehicle for the time frame sufficient for efficient safe 
and comfort drive until next telemetry data package is received.
The generated trajectory is highly dependent on which driving state of ego vehicle will be calculated as
optimal one for this time frame. The optimality of the driving state is always represented by the total 
value of all estimated costs of corresponing state (see **Vehicle::CalculateNextOptimalDrivingState()** 
function in vehicle.cpp). Details of state cost calculation are described below.

#### Driving Strartegy state maschine
The driving related decisions of ego vehicle (like keep driving in the current lane, decelerate before the
lane chane or lane change itself) are controlled by Driving Strategy state machine which is part of 
Vehicle class. During the drivr ego vehicle always stays in one of following driving states:

* **Keep Lane (initial state**). In this state ego vehicle will stay in current driving lane and maintain the 
speed as high as possible with respect to defined requirements for maximum speed. Also the closest vehicle
in front of ego car (if such was detected) is taken into account: if distance to such vehicle becomes 
dangerously small then ego vehicle will gradually decrease it's speed to restore the safe distance (see
function **Vehilce::CalculateVehicleKinematics()** in vehicle.cpp)

* **Prepare Lane Change Left/Right**. In this state vehicle will keep driving in the current lane, but will
look for possiblility and required conditions for changing to more  optimal adjacent lane. (see **Vehicle::EvaluatePrepareLaneChangeState()** function in vehicle.cpp). The optimality of each lane is 
calculated based on the speed of the vehicle which is drivining closest to ego vehicle in corresponding lane
in front of ego car whithin predefined foresight scanning distance (see **Vehilce::CalculateLaneSpeed()**
function in vehicle.cpp). 
Based on detected lane speed corresponding Speed cost value is assigned to corresponding lane (see detailed description of state cost calculation strategy below). The lane with lowest speed cost is considered as 
optimal for driving. Also the check of possibility to change to optimal driving lane is done. The lane 
change is considered possible (read 'safe'), if no vehicles are detected in the 'landing zone' in target
lane (vehicle free segment of lane ahead and behind of ego car) (see **Vehilce::IsLaneChangePossible()**
function in vehicle.cpp)

* **Lane Change Left/Right**. In this state the ego vehicle executes the lane change using the calculated 
trajectory that aims safe and comfortable lateral shift of ego vehicle from the center of current lane
to center line of target optimal lane.

#### Trajectory planning 

Thhe whole process of planning the driving trajectory for calculated optimal target lane and speed of ego
vehicle is encapsulated in **Vehicle::CalculateDrivingTrajectory()** function (vehicle.cpp).
Generated driving trajectory will consist from two parts: remaining waypoints uncovered by ego vehicle during
the time frame between current and previous event of communication with Road Simulator (see lines 381-388 in 
vehicle.cpp), and also newly generated waypoints to extend the trajectory to predefined length (see lines 
356-370 in vehicle.cpp). This merge of two trajectories is needed for making the transition from driving
trajectory generated using previous telemetry data package to the new trajectory (which may differ a lot in
case of lane change for example) as smooth as possible. The smoothing itself is achieved by using two last
waypoints of previous trajectory as start of newly geneerated trajectory (see lines 324-352 in vehicle.cpp).
The fixed number of constraint (anchor) points is calculated on the fixed disatnce from each other in order
to build the base (skeleton) of the 'new' part of resulting driving trajectory (see lines 356-369 in
vehicle.cpp). The calculated anchor points are then fed into the Spline library which connects those points
together into smooth line using Spline interpolation method (see lines 390-492 in vehicle.cpp). This line
is then split into the set of segments to be followed by ego vehicle one by one every 0.02 seconds. 
The length of these segments is calcuated accordingly in order to gradually match the caclulated optimal
speed in the end of generated trajectory and in the same time not to exceed the speed and 
acceleration/deceleration limitations on each of these trajectory segments. The (X,Y) coordinates of end
points of calculated segments are then added to remaining waypoints of previous trajectory and alltogether
they represent the resulting driving trajectory which is sent back to Road Simulator (see lines 394-425 in vehicle.cpp)

### Driving State Cost calculation

The calculated total cost of each Driving State of ego vehicle is always a sum of following costs:

* **Suboptimal speed cost**.  Simply saying, this cost represents the position of the speed value of the 
target lane (implied by corresponding target Driving State) on the scale between current ego vehicle speed 
and maximum posisble speed. The closer the target lane speed is to the maximum speed, the smaller cost is
assigned to corresponding target Driving State. (see lanes 20-33 in cost.cpp)

* **Driving State maintenace cost**. This type of cost was introduced in order to prevent the ego vehicle
stuck in 'transtition' state (e.g. prepare to lane change). Such states will have a 'maintenence' cost
higher than 'stable' driving sates (e.g. KeepLane or LaneChage<Left/Right>). In this way, if the same
total cost (based on other cost critieria) will be calculated for both LaneChange<X> and PrepareLaneChange<X>
states for the same target lane, the preference will be given to Lane Change state because of smaller cost 
value of this particular cost component. (see lanes 35-49 in cost.cpp)


## Further potential impromements of Path Planning application

Following features of Path Planner which are not yet implemented could potentially improve the overall level
of safety, comfort and efficiency for ego vehicle drive:

* **Reaction on "Cut Off" events**. At the moment ego vehicle is will not be able to gracefully handle the 
scenario, when other vehicce with much lower speed gets suddenly in front of ego vehicle. Such case may 
potentionally lead to collision. This may happen because while the ego vehicle will still try to slow down
and restore the safe distance to car in front, it will be executing the 'slow down' maneuver with resepect
to defined requirements to maximum deceleration and jerk. To avoid the collision into the 'cutting off' 
vehicle, the emergency braking or sharp lane change would have to be implemented, but this of couse would
lead to violation of above mantioned requirements to the comfort drive.

* **Dynamic calculation of safe distance to vehicle in front**. Current implementation for calculation of safe
distance to vehicle in front does not take into account the acceleration of ego vehicle and also ignores
the speed and acceleration of vehicle ahead. The formula used currenmtly defines the linear 
correlation between ego vehicle's speed and safe driving distance to the car in front. This potentialy can
lead to dangerious cases when there is big difference between speed values of ego vehicle and vehicle in 
front, or when vehicle in front start suddenly decelerate. Ideally the calculation of safe distance should
be based on Time To Collision formula, which takes into account all kinematic parameters of both vehicles



## Setup instructions

This project requires the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

## Application build dependencies

* cmake >= 3.5
  
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)
* uWebSockets
  
    * Scripts install-ubuntu.sh and install-mac.sh can be used to set up and install [uWebSocketIO]
    (https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems accordingly.
    For windows use either Docker, VMware, or [Windows 10 Bash on Ubuntu]
    (https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/)



## Build Instructions

Once installation of all dependencies is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
   
## Execution instructions

1. Start created Path Planning application from build folder:
  
   ./path_planning
   
   'Listening to port 4567' notification should appear.
   
2. Start downloaded Term 3 Simulator application in terminal. Select requried resolution and press Play button.
   Press 'Select' button under 'Project 1: Path Planning' sign. This will start the simulation and establish
   the connection with started Path Planning application. All the information about current driving parameters
   and driving decisions of ego vehicle will be displayed in terminal window of started Path Planning application
   in real time  
