# Term 3 Path Planning / Behavior Project 

## Objectives, Performance Summary and Improvements

In accordance with the Project Rubric, the vehicle must be capable of traveling 4.32 miles without incident. 
Incidents include collisions, maximum Jerk events, exceeding the speed limit and traveling within the confines of the selected lane. The vehicle must also not take longer than 3 seconds when executing lane changes.

With the code reflected in the repository, the vehicle has consistent performance when the vehicles in the vicinity maintain a consistent path. Currently when running the simulator, I have consistently achieved distances of over 20 miles. This could be improved with the addition of further prediction methods, however due to work travel and the program timeline, I was not able to execute. In it's current form, the system will execute the majority of the track without issue. 

I would like to be able develop a cost function that also relates to minimum distance between vehicles but have yet to do that. A further improvement would be to implement evasive actions based on the time to collision functions and to also use that value in lane cost functions.  A function that monitors conditions and aborts a lane change would also be a way to ensure better stability. 

## Code Description

### Externally Obtained Includes

Per the Udacity policy, I take no credit for these libraries as I did not write them. Here, I want to breifly describe why I chose to include them.  

#### tic_toc.h

This function is analogous to the tic and toc functions in Matlab for reporting program execution time. I used this to capture the time between incoming data frames.  The code was copied from  http://programming-tips-and-tricks.blogspot.com/2017/05/tic-and-toc-functions-in-c-millisecond.html and modified for my needs to put out a double. 

#### spline.h 

After implementing the code initially from the Q&A video and reading various comments on the slack channel related to the path generation using the spline library, I chose to execute the path planning with that tool as it appears to create the necessary constraints required for curve generation. 

In the github repo (https://github.com/ttk592/spline) the README states "It generates a piecewise polynomial function of degree 3 and is twice continuously differentiable everywhere. Boundary conditions default to zero-curvature at the end points."
While this is not identical to the Jerk Minimizing Trajectory explained in the lesson, this allows for smooth transitions from spline to spline due to the endpoints of each spline generated produce zero curvature So long as the constraint of dt = 0.02 is respected when feeding successive points to the simulator.

#### Track Class from Udacity Alumni Eric Lavigne 

The Slack Workspace for the Udacity projects are a great way to avoid program setup and pitfalls. Due to traveling with a chromebook during a large section of project time, I leaned on this resource to write as much code as I could before being able to debug on the simulator. 
During that time, I saw multiple Udacity students mentioning that the getXY functions producing erroneous data that would generate higher jerk trajectories. 
I found a post from Eric Lavigne (https://carnd.slack.com/archives/C5ZS5SBA8/p1526189941000047) stating that he had eliminated that problem with a new Track class. This class utilized the spline library seen in the Q&A. Not wanting to get bogged down in errors from the original helpers, I chose to include this based on feedback in the channel.

### Helper Functions

I moved most of the helper functions from the provided main.cpp to another file called helpers.cpp. After pruning functions I no longer intended to use, I added other functions listed here.

#### s_relative

Since the track is a loop, the s value reported by the simulator resets at an endpoint. In order to calculate relative longitudinal distances in Frenet Coordinates, I created a function that generates accurate relative distances between two s values, even if one of the two vehicles is straddling the reset point.

#### getLane

This is a simple function to generate a lane integer from the Frenet coordinate d value. 

#### getMapYaw

I intended this function to be a means of verifying if a particular vehicle was diverging from a clean lane path along the S axis. In the end, I never added this level of prediction but after submission, but I intend to continue with this as an exercise.

### Vehicle Data Handling

I created 3 classes that would serve as the containers for both ego vehicle and external vehicles coming from sensor fusion.  Since I would likely be comparing quanities between the ego vehicle and the external vehicle, it made sense to me to create a data type that was uniform between them so I would be indexing into the same member.  

#### Vehicle Frame
This represents an single frame of incoming data for a specific vehicle, including the ego vehicle. In addition to the regular data, the time since the last frame is included as well as the vehicle lane so downstream calculation is not required.

#### Vehicle

This class and its member functions maintain a buffer of vehicle frames for the individual vehicles and maintain estimated values for the following quanitites as each vehicle frame is added to the buffer. 

- s distance relative to ego Vehicle
- s axis velocity
- s axis acceleration
- d axis velocity
- d axis acceleration

Besides maintaining and handling the incoming data for each Vehicle Instance, functions of this class are also used to provide a predicted Vehicle state based on the estimated values.

#### Vehicle Field 

This purpose of this class is to catalog and maintain information on the vehicles surrounding the ego vehicle. A std::map creates a new id, Vehicle pair for each vehicle that enters within a specified distance ahead or behind the ego vehicle. The entry for the vehicle is also reset when the vehicle exceeds that boundary so the next vehicle assigned that ID starts without any incorrect prior data. 

Member functions in this class allow quick interrogation of the Vehicle state information for the vehicle ahead in a specified lane. By calling the member functions of the individual Vehicles, this class can also produce a 'fast forwarded' version of itself for prediction of the entire field of cars. 

### Behavior Class

The functions that execute the decision making policy for lane changes and setting the vehicle speed target are contained within the Behavior class. When constructing the Behavior class, a pointer to the Vehicle Field and Ego Vehicle is passed for easy access to that data. 

##### Speed Member Functions

keepLane and setLaneChangeSpeed are functions that take the lane information and determine the appropriate vehicle speed after polling the speed of the vehicles directly ahead of the ego Vehicle. They differ in that the setLaneChangeSpeed will allow for a shorter following distance for the vehicle in the departure lane.

#### Lane Evaluation Functions

In order to decide if a lane change should be executed a cost function is used to evaluate the state of vehicles in each lane. The cost functions are weighted according to importance.

- Clear Adjacency: This is the highest weighted cost as the car should not change into a lane that is occupied at any time. 
- Lane Speed: Cost increases linearly based on the difference between the speed of the car ahead and the speed limit.
- Clear Lane Distance: Cost increases as the gap between the ego car and the car ahead decreases. 
- Lane Change Execution Cost: There is a fixed cost associated with any lane that is not the current lane. This is to add some hysteresis to the decision making and prevent 'hunting' and successive lane changes unless the conditions truly do change significantly.  
- "Bridge Lane" Cost: A direct lane change of two lanes is not allowed. If the lowest cost lane is two lane changes away, a "bridge cost" is added to the current lane to help incentivize the system to execute the change. 
- Static Lane Costs: I am added a static cost function focusing on the center lane for a few reasons. Crusing in the center lane allows us more options for passing without creating complex multi-step lane maneuvers. From a legality and driver courtesty standpoint, it is frowned upon to pass on the right but it does happen. This means we should not linger in the left lane as it should be reserved for faster moving traffic. There is also a defect in the simulator at one point that will indicate a "Out of Lane" incorrectly. This occurs in the right lane only. I chose to use this cost to bias my car out of that lane unless passing. It is marginally successful depending on traffic conditions. 

#### Prediction of Lane Cost

Before executing a lane change, the system re-evaluates the lane costs with all of the vehicles, including the ego Vehicle, predicted forward by 1.5 seconds. This is intended to produce the vehicle positions and states after the lane change has occurred. If the targeted lane is not the most adventageous lane given these new conditions, the lane change is cancelled before it begins.

With more time to work on the project, I would execute this check several more times in smaller increments. 

### Path Planning

The path planning approach taken is very closely based on the methodology outlined in the Q&A video. To demonstrate my knowledge of that process (vs a blind copy & paste), I will explain it in detail to the best of my ability.

#### Prior Path Points && Path Initialization

The simulator returns the previous, unused path waypoints that were sent in the previous transmission. The final two points from these are re-used to assure tangency and reduce the amount of points that need to be recalculated. 

When the simulator starts up, there is no path provided, so in order to create a vector of smooth path points, an artificial starting point is created. This is done by evaluating the vehicles current position in cartesian coordinates and using the provided yaw information to provide another point behind it. 

#### Adding Spline Knots

Using the lane chosen by the behavior object, 3 way points are calculated in Frenet coordinates using the ego vehicle s position as a starting point. These points are 30 meters apart for maintaining a lane position but are expanded to 50 meters apart when changing lanes. This changes is done to produce a path with acceptable lateral acceleration and jerk. The d position is calculated simply by multiplying the desired lane integer by 4 and adding 2. 

The cartesian coordinates are then converted to the vehicle reference frame and a check is done to add only pairs of points that provide a monotonically increasing x value. These two vectors of x and y points are used to define a new path spline from which to select waypoints.

#### Speed Considerations and Simulator Timing. 


## UDACITY ORIGINAL CONTENT BELOW THIS LINE

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

