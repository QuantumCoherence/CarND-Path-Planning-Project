# Path Planning Project
## GitHub Repo
[CarND-Path-Planning-Project](https://github.com/QuantumCoherence/CarND-Path-Planning-Project)


This project was completed on Ubuntu 16.04.

The build sub-folder contains all the binaries and the makefile for compiling the project at the prompt with no formal IDE. 

**Compiling on your system: CMakelists.txt**

Use the CMakeLists.txt in the repo root folder to create the make file and compile on your system.See the README file.


## Path Planning Project Notes

### Fundamentals

The project implements path planning for a freeway traffic environment, using behavior planning to control driving state transitions by associating a cost value to each possible trajectory and state transition. The behavior planning loop simply selects the least expensive trajectory among all possible outcomes.
    Although collisions are to be avoided, the expectation is that in this simulated environemnt no circumstance will arrise that would make a collision unavoidable without axceeding the max acceleration constratint, so no special provision is conisdered to hanadle emergency situations.
In this particular project localization and lane detection are simply simulated by the freeway simulator, which returns the actual positions of the Ego vehicle and of all of the traffic vehicles at all time.
The task at hand is to simply analyze the traffic on the simulated freeway and to return to the simulator a trajectory for the ego vehicle that will avoid collisions, drive the ego within dynamic constraints and maintain its speed as close as possible to the target speed, changing lane whenever possible to avoid being slowed down by traffic.
	The selected trajectory is sent to the simulator in the form of waypoints the ego vehicle will then drive on. Waypoints are processed by the simulator at a 50 Hz frequency, hence waypoints define not only position, but also speed and acceleration.
    
	Dynamic Constraints
	- Speed limit 		49.88 	[mph]
	- Max Acceleration	10		[m/s2]
    - Max Jerk 		   30	    [m/s2]


### Processing loop

In a nutshell, this is what the path planning software does at every simulation loop cycle
1. Update current Ego and traffic positions as returned by the simulator
2. Predict the future positions for a given time interval of all traffic vehicles that are in range of the ego (in_range_distance is a tunable parameter)
3. Estimate all possible ego trajectories during the same time interval
4. Estimate the cost of each trajectory by comparing all of the estimated trajectories with all future traffic vehicle positions
5. Select the minimal cost trajectory and calculate the waypoints to be sent to the simulator so that the ego will execute such trajectory, while keeping max acceleration, speed limit and max jerk within the required constraints.

### Waypoints

The waypoints are calculated such to keep the speed, acceleration and jerks within the required constraints, by using a spline function, which connects the current ego position  with the calculated final position at the end of the desired trajectory.

The trajectory estimation is calculated by measuring the actual time elapsed between simulation loops. On the linux system used for development, the typical simulation loop time was in the range of 0.26 seconds at a 600x800 pixel simulator image size

### Cost Functions

Each trajectory at each simulation loop will be associated with an estimated cost (nominally every 0.02 seconds).
The cost is the sum of all cost functions, that are being calculated to minimize the impact of various aspect of the traffic.

The current implementation estimate the following costs:
- Distance to Goal Cost
	This cost penalizes trajectories with longer distance
- Collision Cost
	This cost penalizes trajectories that would collide with other vehicles
- Inefficiency Cost
	This cost penalizes trajectories that are slower, in prticular this cost tend to favor a lane with faster traffic over one that has lower traffic, even if closer to the furthest vehicle on any lane.

### Driving State FSM

There are three possible driving state: 
- Keep-Lane KL
- Prepare_Lane_Change PLCL PLCR
- Change_Lane LCL LCR

The KL trajectory is biased to prefer the central lane when no vehicles are present so the ego will transist back to the central lane even if there are no vehicles on its current lane.

	Transitions
    - KL PLCL PLCR
    - PLCL LCL
    - PLCR LCR
    - PLCL PLCL (if current lane is Right)
    - PLCR PLCR (if current lane is Left)

In meta language, the above transition means the folllowing:
Try KL, PCLC and PLCR
If KL is the lowest cost, try the same transition again
If eiher PLCL or PLCR are lower cost, then respectivley try 
PLCL LCL, or
PLCR, LCR

This FSM is very simple and does not observe lane transisitons beyond one single lane. 
This can lead to the ego potentially getting stuck on one side of the freeway, behind a slow vehicle, if another slow vehicle occupies the middle lane close to the ego vehicle, inspite of an open lane available on the other side of the freeway.
Typically the ego comes along the middle lane. If a vehicle is present in the middle lane, ego will change to another lane following these priorities: 
1. Empty Lane
2. If all lane are occupied, faster lane first

At this point, without the double PLC* transistion, the ego would remain behind the vehicle on the side lane, until the middle lane was to become free, regardless of what happened to the opposite side lane. The added transition enables the ego to transition two lanes off and take advantage of an open lane that was otherwise no longer accessible. It howevr requires the ego to back off enough for it to be able to transition. This is why the ego will  slow down slightly more than the ahead vehicle, therefore increasing so slighlty the distance to the next vehicle over time. This will create the opportunity to transittion to the other side in one single pass, even though technically is two transition in rapid succession. This will take place only if the other lane is empty.
	
This approach is however not very robust when vehicles are traveling nearly at the same speed in all three lanes. In such case the ego would tend to transition back and forth trying to keep up with the fastest lane. Being the vehicles driving at virtually the same speed, this is is likley to cause repeated changes of lane that would bring the ego no further than w/o lane changes.

A more robust approach is to extend the FSM and cost function so to handle efficiently more complex traffic cases.

**Vides**

[Path Planning Video](https://github.com/QuantumCoherence/CarND-Path-Planning-Project/blob/master/vokoscreen-2018-10-22_03-09-30.mkv)

**Images**
The following images show some outoput screen captures from a successful run. 
Notice how the Ego vehicle "decided" to wait behind a specific traffic vehicle despite it appearing to be the furthest back of the three vehicle impeding a full speed run. As it turns out, the selected lane is the fastest one, which will lead to the fastest resolution of the bottle neck. By using a combination of cost functions and driving state transtion logi, the ego vehicle achieves the shortest path in time through a traffic jam and can then resume its normal full speed course soon after the way is free of slower traffic.

| ![Heading Into Traffic Jam](https://github.com/QuantumCoherence/CarND-Path-Planning-Project/blob/master/vlcsnap-2018-10-23-22h36m29s016.png?raw=true) | 
|:--:| 
| **Heading Into Traffic Jam** |


| ![Best Lane it's behind this vehicle on the left](https://github.com/QuantumCoherence/CarND-Path-Planning-Project/blob/master/vlcsnap-2018-10-23-22h37m03s275.png?raw=true) | 
|:--:| 
| **Best Lane it's behind this vehicle on the left** |

| ![Keep Distance](https://github.com/QuantumCoherence/CarND-Path-Planning-Project/blob/master/vlcsnap-2018-10-23-22h37m14s133.png?raw=true) | 
|:--:| 
| **Keep Distance** |

| ![Slowly following through traffic jam](https://github.com/QuantumCoherence/CarND-Path-Planning-Project/blob/master/vlcsnap-2018-10-23-22h37m49s652.png?raw=true) | 
|:--:| 
| **Slowly following through traffic jam** |

| ![STeadyg](https://github.com/QuantumCoherence/CarND-Path-Planning-Project/blob/master/vlcsnap-2018-10-23-22h38m00s584.png?raw=true) | 
|:--:|
| **Steady as it goes** |

| ![Almost Through](https://github.com/QuantumCoherence/CarND-Path-Planning-Project/blob/master/vlcsnap-2018-10-23-22h38m15s054.png?raw=true) | 
|:--:| 
| **Almost Through** |

| ![Clearing Traffic on Fastest Path](https://github.com/QuantumCoherence/CarND-Path-Planning-Project/blob/master/vlcsnap-2018-10-23-22h38m28s479.png?raw=true) | 
|:--:| 
| **Clearing Traffic on Fastest Path** |

| ![Back To Full Speed](https://github.com/QuantumCoherence/CarND-Path-Planning-Project/blob/master/vlcsnap-2018-10-23-22h38m59s983.png?raw=true) | 
|:--:| 
| **Back to full Speed** |





