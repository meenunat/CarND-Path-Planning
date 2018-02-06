# Udacidy CarND Path Planning Project

## Objective: 

The goal of the project is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic that is driving +-10 MPH of the 50 MPH speed limit. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

The car's localization and sensor fusion data along with sparse map list of waypoints around the highway are provided. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

The map of the highway is in data/highway_map.txt. Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions:

This project is done in a simulator and Term 3 Simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Buiding instructions as follow:
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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
* main.cpp handles websocket events, and contains all of the relevant code.

* spline.h is a convenient open source spline library used for finding trajectories.

## Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

## Previous path data given to the Planner
//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

## Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

## Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Flow Details 

1. The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.
      
## Path Planning Implementation

The simulator returns instantaneous telemetry data for the ego vehicle, but it also returns the list of points from previously generated path. The sensor fusion data received from the simulator in each iteration is parsed and trajectories for each of the other cars on the road are generated. 

A FSM to change Right Lane, change Left Lane, remain in same Lane and slow down if accelerating was created.They  are used in conjunction with a set of cost functions (based on Feasibility, Collision, Danger, and Efficiency - i.e by using constant multiplier as cost) to determine a best trajectory for the ego car. Each state has assocaited cost and the decision is made comparing the cost. The trajectories match the duration and interval of the ego car's trajectories generated for each available state. 

     for(it = cost_Map.begin(); it != cost_Map.end(); it++)
      {
           //Left lane
           if (it->first == 0)
       {
            cost_Map[0] += feasibility_cost(lane-1);
            if (cost_Map[0] == 0)
        {
             cost_Map[0] += collision_cost(SensorVector, lane-1, prev_size, car_s);
             cost_Map[0] += left_lane_change_cost();
             cost_Map[0] += guard_cost(SensorVector, lane-1, prev_size, car_s);
            }
           }
           //Right Lane
           else if (it->first == 1){
            cost_Map[1] += feasibility_cost(lane+1);
            if (cost_Map[1] == 0)
        {
             cost_Map[1] += collision_cost(SensorVector, lane+1, prev_size, car_s);
             cost_Map[1] += right_lane_change_cost();
             cost_Map[1] += guard_cost(SensorVector, lane+1, prev_size, car_s);
            }
           }
           //Keep Lane
           if (it->first == 2)
       {
            cost_Map[2] += feasibility_cost(lane);
            if (cost_Map[2] == 0)
        {
             cost_Map[2] += collision_cost(SensorVector, lane, prev_size, car_s);
             cost_Map[2] += keep_lane_cost();
             cost_Map[2] += guard_cost(SensorVector, lane, prev_size, car_s);
            }
           }

          }


###Collision cost: penalizes a trajectory that collides with any predicted traffic trajectories with distance less than 35.
   double collision = pow(10,4); 
 
   double collision_cost(vector<vector<double>> SensorVector, int lane, int prev_size, double car_s){
    double cost=0;
    for (int i=0; i < SensorVector.size(); ++i)
    {
       float d = SensorVector[i][6];
       if (d<(2+4*lane+2) && d >(2+4*lane-2))
              {
       double vx = SensorVector[i][3];
       double vy = SensorVector[i][4];
       double check_speed = sqrt(vx*vx+vy*vy);
       double check_car_s = SensorVector[i][5];
       check_car_s += ((double)prev_size * SampleTime *check_speed);

       if ((check_car_s > car_s) &&((check_car_s-car_s)<35))
       {
        cost+=10*collision;
       }
       }
     }
    return cost;
   }

###Guard cost: penalizes a trajectory that comes within a certain distance of another traffic vehicle trajectory with distance less than 55.
   double guard_cost(vector<vector<double>> SensorVector, int lane, int prev_size, double car_s){
    double cost=0;
    for (int i=0; i < SensorVector.size(); ++i)
    {
       float d = SensorVector[i][6];
       if (d<(2+4*lane+2) && d >(2+4*lane-2)){
      double vx = SensorVector[i][3];
      double vy = SensorVector[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = SensorVector[i][5];
      check_car_s += ((double)prev_size * SampleTime *check_speed);
      if ((check_car_s > car_s) &&((check_car_s-car_s)<55)){
       cost+=10*danger;
      }
     }
    }
    return cost;
   }


###Efficiency cost: penalizes trajectories with lower target velocity.
   double efficiency = pow(10,2); 

###Feasibility cost: penalizes driving in any lane not feasible by the ego car.
   double feasibility = pow(10,5); 

## Results: 
The car was able to run successfully around the highway without any incident. Due to git size limitation: short clipping of the drive is uploaded in the repository (out-4.ogv)

## Discussions
Sometimes the car might experience maximum acceleration depending on the spline. Hence I had to fine tune the points to be less aggressive to maintain within maximum acceleration. Sometime car can get stuck behind a slow car if the next lane car relatively same speed and following little behind. In such a case, the car could slow down and move to the very last right lane.  
