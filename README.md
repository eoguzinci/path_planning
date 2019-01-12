# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Introduction
In this project, the main goal is to safely navigate around a virtual highway provided by Udacity with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The localization information and the sensor fusion data is provided by the simulator itself. There is also a sparse map list of waypoints around the highway in the file `data/highway_map.csv`. The  The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The efficiency of the ride is also wished for. Therefore, the car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3. This is kind of comfortable driving requirement.

## Basic Build Instructions & Setting up the Environment

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

### Simulator.
To run this project, we need to download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Highway Data
The map of the highway is in data/highway_map. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Localization and Sensor Fusion Data

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

3. The code starts with perception of the cars around the ego vehicle. The lane, speed, distance to our ego vehicle of the cars surrounding is first captured. From this information, the action of the vehicle in the next step will be determined. As we think the ego vehicle, we still use the local FreNet coordinates with respect to our ego vehicle. To capture the vicinity, speed and lane, we loop over the sensor fusion data.
 
    - We first check if the car is on the same lane ahead to implement an algortihm with lane keeping and adaptive cruise control. If the car is ahead and in 50m close in the same lane, we check if we are in the safe distance according to the physics. In this case, we assume that the car ahead might suddenly stop and we are able stop in 1.5 s. We define the safe distance as follows:

    <img src="https://latex.codecogs.com/svg.latex?\Large&space; x_{safe} = v_{ego}t - a_{max}t^2/2" title="\Large x=\frac{-b\pm\sqrt{b^2-4ac}}{2a}" /> 

    - If the the distance with the car ahead is less than this safe value, the car starts to decelerate to keep the safe distance. The car accelerates with maximum acceleration until it reaches the speed limit if there is no car ahead. The maximum longitudinal acceleration is set as 5 m/s^2 where as the acceleration limit is set to 10 m/s^2. This is due to the safety factor wwe put to avoid lateral + longitudinal combined acceleration is less than the limit. 

    - The lane change dynamics are a bit different than keep lane algorithm. We check if there is a car ahead blocking the lane or in the vicinity on left and right lanes. If there is one we skip the lane change to this lane for this timestep. So, the lane is label as NO_GO for this timestep. The vehicles coming from behind with a higher speed than our ego vehicle are also considered to avoid rear collision during lane change.

4. After perception and restrictions we put, the moment comes to take action. The action options vary upon which lane are we in. In the middle lane, we have 3 options to take: (i) keep the lane, (ii) shift to left lane or (iii) shift to right lane. However, in the side lanes you have only two options, as we try to keep inside the road. The vehicle is set to keep the current lane with a speed close to speed limit (50 mph) if there is no vehicle in front. If a vehicle was detected within the safe distance ahead, our vehicle tend to decelerate until it reaches the velocity of the car in front. Due to the efficieny concerns, we put patience time limit to keep behind a slower vehicle front. The clock starts to tick when we first detect the slow vehicle in front and when it exceed our patience time limit, we start find safe opportunity to faster lane. Therefore, we also keep track of the speed of each lane. The speed of each lane is restricted with the slowest vehicle in 50m ahead of our vehicle in FreNet coordinates. If there is no vehicle restricting our lane change and the traffic flows faster than the current lane, the vehicle changes the lane. After we change the lane the vehicle tends to accelerate back to the speed limit if there is no car in front. If there is one the patience clock starts over until the vehicle got bored and finds a better lane to proceed. This provides us that the vehicle always drives between lane lines and does not always search for a gap to lane change between lanes.

5. After the determination of the action, a trajectory is drawn to be followed by the vehicle. To avoid abrupt changes in acceleration and jerk, our trajectory must be at least C1-continuous. For this reason, we use splines to create the waypoints that the car follows. We first add the 3 new waypoints with at the update lanes in 30m, 60m, and 90m ahead of the vehicle in FreNet coordinates. To ensure smoothness, we use the previous waypoints to create the trajectory spline. If there is no previous waypoints, we add two artificial points assuming the yaw of the vehicle was constant to ensure the C1-continuity. This is especially important to avoid high jerk in the lane changes. The distance between the waypoints are determine by the speed of the car. After obtaining the trajectory spline, we spread our next waypoints regarding where the vehicle will be in the next timesteps. This trajectory building operation is done in local FreNet coordinates, but at the end the waypoints are transformed back to the global coordinates.

## Tips & Rubrics

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single header file is really easy to use.

### Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

### Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


### Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

### How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

