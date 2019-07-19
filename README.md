# Overview 
This project navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit in C++. The car's localization, sensor fusion data, and a sparse map list of waypoints around the highway is available. The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car tries to avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. 
![](video/pathplan.gif)
# Implementing the Path Planner 
The implementation is completed in the telemetry section in the file [main.cpp](./src/main.cpp) between lines 109 and 259. The path planner consist  of 3 main parts: Prediction,  Behaviour Planning, and Trajectory Generation.

#### 1. Prediction  
The prediction part of the code is between lines 109 and 145. 

The prediction part use data from sensor fusion to generate predictions about likely behavior of moving objects. The prediction checks if the neighboring car is ahead, to the left or to the right of the ego vehicle at the end of the planned trajectory. If any of the car's location is less than 30 meters in front of the ego vehicle, the presence of the neighboring car is considered for planning. 

#### 2. Behaviour Planning
The behavior planning part of the code is between lines 146 and 173. 

Based on the prediction, this behaivor planning decides if there should be lane change, or to stay on the current lane for safety reasons. If the vehicle is not on the middle lane, it plans to change back to the middle lane when safe. This part of the code also checks for the ego vehicle's speed and determine whether to increase or decrease the speed. 


#### 3. Trajectory Generation
The trajectory generation part of the code is between lines 175 and 259.

For creating a smooth trajectory spline function is used. The starting points of the spline calculation is where the ego vehicle is if the previous path is empty. If the previous path is not empty, the starting reference is at the end point from the previous path. The next three points are projected at 30, 60, 90 meters away by taking account for the target lane. Using the points, spline is generated. 

For a continuous trjectory generation, previously generated paths are used as the starting reference for new trajectory. To minimize jerk and for smooth highway driving, the spline points are broken up by considering the desired velocity.    





#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.


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




