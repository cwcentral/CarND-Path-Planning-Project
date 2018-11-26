# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
In this project, we configure and execute a path planner to navigate a highway with multiple vehicles.   
   
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.
   
### Implementation

The ego car uses a provided highway_map.csv to provide a set of waypoints on how to drive around the track. 
The software looks at extending the current car's path by 50 points every 0.02 seconds via frenet space points:

<img src="output/freenet.png" width="480" alt="Combined Image" />

Step 1 (lines 315-425) I needed to decide on whether the car needs to change lanes or not. I implemented a logic function to express a cost in changing lanes:
   1. Strong bias to switch to the farest most left lane, lane 0
	
   2. if there is zero cars detected in the next lane, move to it, it maybe left or right if I;m in lane 1.
	
   3. only allow one lane changes per lane switch/time step
   
   By developing a database of what cars are in "what" lane, I was able to deteremine what was the nopen lane I should switch to IF I was coming upto a car in my lane within 30meters.
   
   
Step 2 (lines 426-440) I needed to decide if I should slow down or speed up. If I detected a car in front of me and decided to see if I can change lanes, this would trigger logic to slow down. Otherwise I would always speed upto the max speed limit.

Step 3 (lines 445-512) I need to create a smooth path to the next endpoint of my waypoint set. In order to create a smooth transition of points during lane changes and curves in the road, I extracted anchor points at the last position of the previous trajectory, the current position and 30, 60, and 90 meters in front of the ego car.

Step 4 (lines 517-570) I convert the anchor points to local frame (pose of the car is (0,0), run a spline generator to create new points, then convert back to global frame with the current position of the car as 0,0. 

The result was a path planner that changed changes smoothly, maintain speed and avoid collisions.

[![Output](output/spline.png)](https://youtu.be/HMGEo_nn_6c "Click to Play Video")

   
### Reflection

The software implemented uses a function to limit acceleration in both Frenet S and D dimensions. It also uses the recommended Spline class to generate smooth transistion when lane switching. In the project lessons, it was recommended that only moving to lane 0 was the only option. And that could create a scenario that the car would take 10 mins or more to complete the track (got caught in a traffic jamb on lane zero. 

This provide successful and I was able to consistency navigate the track in under 6 minutes.
<img src="output/running_example.png" width="480" alt="Combined Image" />

To check for robustness, I was able to run for an extended period of time:
<img src="output/running_long.png" width="480" alt="Combined Image" />

##### What are issues with this implementation?

a. If you're in a traffic jam, you are constantly deaccelerating and accelerating. Though jerk is acceptable and within limits, it would be better to either match the speed of the traffic OR find a way to navigate through the traffic by multiple lane switches.

b. The car lane database I created ONLY looks at a rectangle-area of collision avoidance. It need to incorporate the velocities of the cars close by to ensure 100% collision free trajectories. Currently it is sufficient for the vehicle dynamics of the current model, BUT there are times I would cut off another vehicle.

c. The model needs to account for sudden lane changes from OTHER vehicles. A couple of times I switched lanes to go around a vehicle, then suddenly that same vehicle switches to my new lane, in essence CUTTING ME OFF and attempting to "block me". That would require a emergency stop, but likely exceeding the acceleration limits define by the requirements. I currently use a braking multipler looking at the distance of the cars and appears to be sufficient, but likely not robust in all conditions. It also shows the current vehicle dyanmics model provided (of the other cars) is incomplete. 

d. Lastly a better cost function can be used. For example, what can improve lane switching is looking at multiple cars in front of you in the same lane to assess traffic conditions/density. This would require a more sophisticated cost function than the rules I have used. Of course in reality, LIDAR data would shadow any cars in front of the immediate car I detected, thus need more technology to identify all cars/objects on the road.
   
   
### Rubric Summary

(OK) The code compiles correctly.
(OK) The car is able to drive at least 4.32 miles without incident.

*Provided in the video link above*

(OK) The car drives according to the speed limit (50mph)

*See software, main.cpp, lines 441*

(OK) Max Acceleration and Jerk are not Exceeded (+- 10m/s^2).

*See software, main.cpp, lines 441*

(OK) Car does not have collisions

*Provided in the video link above*

*There maybe conditions where computer vehicles decides to cut you off and block you intentionally, thus needing a emergency stop that will exceed jerk requirements*

(OK) The car stays in its lane, except for the time between changing lanes
(OK) The car is able to change lanes

===============================================================================================================

   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

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

