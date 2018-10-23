# Path Planning Project
## GitHub Repo
[CarND-Path-Planning-Project](https://github.com/QuantumCoherence/CarND-Path-Planning-Project)


This project was completed on Ubuntu 16.04.

The build sub-folder contains all the binaries and the makefile for compiling the project at the prompt with no formal IDE. 

**Compiling on your system: CMakelists.txt**

Use the CMakeLists.txt in the repo root folder to create the make file and compile on your system.


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

Each trajectory at each simulation loop will be associate with an estimated cost (nominally every 0.02 seconds).
The cost is the sum of all cost functions, that are being calculated to minimize the impact of various aspect of the traffic.

The current implementation estimate the following costs:
- Distance to Goal Cost
	This cost penalizses trajectory with longer distance
- Collision Cost
	This cost penalizes trajectories that would collide with other vehicles
- Inefficiency Cost
	This cost penalizes trajectories that are slower, in prticular this cost tend to favor a lane with faster traffic over one that has lower traffic.

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
This can lead to the ego potentially getting stuck on one side of the freeway, behind a slow vehicle, if another slow vehicle occupies the middle lane close to the ego vehicle, inspite of an open lane avaibale on the other side of the freeway.
Typically the ego comes along the middle lane. If a vehicle is present in the middle lane, ego will change to another lane following this priorities: 
1. Empty Lane
2. If all lane are occupied, faster lane first

At this point, without the double PLC* transistion, the ego would remain behind the vehicle on the side lane, until the middle lane was to become free, regardless of what happened to the opposite side. The added transition enable the ego to slowdown and transition two lanes off and take advnatage of an open lane that was otherwise no longer accessible.


**Vides**
Download the videoscreen capture of an entire loop around the track in ziped form, gunzip and play.
This is the outcome using following paramters 

```
Kp  0.061751
Ki  0.004
Kd  -0.662543
Max_Speed 35
```
These are set by default, so just type 
``./pid ``

[PID Loop Video Download from here](https://github.com/QuantumCoherence/CarND-PID-Control-Project/blob/master/vokoscreen-2018-06-01_21-37-30.mkv.gz)

**Images**
The following images show two outoput screen captures of the Tweedle state machine process (see under Parameters Tuning and Coding Notes for details).
The word "Reset" at the beginning of the line indicates that the car left the paved road and so the error estiamtion process was stopped, the simulator reset and the next step of the Tweedle algorhitmn processed.
The word "Loop" instead indicates that the whole track was completed.
Each time a new lowest error was found, the best_error was updated and the next step of the Tweedle algorithm processed. In the end the progress is visible, even though very slow, given one loop requires aproximatley 600 simulation steps.

The images below show just a subset of the whole process.




![Tweedle Process Output sample 1](https://github.com/QuantumCoherence/CarND-PID-Control-Project/blob/master/Tweedle%20Output.jpg?raw=true)


![Tweedle Process Output sample 2](https://github.com/QuantumCoherence/CarND-PID-Control-Project/blob/master/Tweedle%20Output2.jpg?raw=true)

### Speed and Paramter Tuning
A fully functional Tweedle state machine was implemented that can perform the tweedle algorithm to estimate the optimal PID paramter values. The state machine automatically resets the simulator, whenever the vehicle goes off the paved road or simply hits the top of the curb on either side of the track.

The optional input "-tweedle" to the "pid" binary, will start the tuning process with the starting values 0.1, 0.1 , 0.1 resp. for P, I and D. However, this process is exceedingly long, because at the outstart most parameter values combianiton will lead to crashes and many resets are necessary to move forward with the estimation.

With a bit of heuristic, trial and error it is however possible to find a rough guess numerically quite close to a good starting point for all the parameters, such that the vehicle will remain on the track most of the time and the algorithm will find an optimal configuration in a much shorter number of iterations.

The manual process is quite simple:

```
1. Keep the speed low (see below)
2. Set I and D to zero and make a guess for P
2. If the car flies of the track, reduce P until the car tends to remain on the track on straight or slightly turning trajectory.
3. Add a negative value to D such that the vehicle can now stay on the track even for higher P values, that is for tighter turns, all while achieving some level of stability
4. Once the car remains on the track indefinitely, even if with visible instability, pass the guesstimated parameters as starting to point to the the "pid" binary using the -tweedle option on.
```


Using these paramters the Tweedle state machine can then relativley quickly find optimal P, I and D paramter values, in aproximately several 100s steps. Yes, it remains quite long even after a rough intial manual estiamtion, but comperatively much much shorter,than without using manually estimated initial values.
The images above show a sample of the process output while it was seeking the best optimal PID param setting.

The sample show a starting PID setting that was estimated manually:
P = 0.19
I = 0
D = -0.8

The final values (not shown in the images) that were then coded as default settings for the compliled binary, were:

```
Kp  0.061751
Ki  0.004
Kd  -0.662543
Max_Speed 35
```

*Estimation and control of the speed*

To control the speed and emulate a car driver behavior of breaking to slow down and avoid driving off the track as much as possible, a simple throttle algorithm was used that can be summarized as following:

throttle = (0.7-fabs(cte -prev_cte))*fabs(1-(speed/max_speed));

What this means is the follwing: 

1. when the ratio between speed and the predefined max_speed value, is close to one, the throttle is set to zero.
2. when the rate of increase of the cte error is bigger than 0.7, the car slows down.
3. When the car slows down, the cte error rate goes to zero, hence accleration is the applied normally.




### Coding and usage Notes
No throttle PID was implemented as a simple algorithm was sufficient to control the speed effectively.
A Tweedle State Machine was coded in the pid class, that fully implements the whole tweedle algorithm.
The pid binary accepts following optional parameters

``./pid -tweedle" `` 

triggeres the tweedle process with starting values 0.1, 0.1, 0.1, It stops when tol < 0.00001

``./pid -tweedle -sp <float> -si <float> -sd <float> -mv <float>`` 

start the pid controller with the specified p, i, d and max_speed -mv value

``./pid ``

triggers the pid controller with default parameters value as described above 


Comments in the code should be self explanatory . 

Please note despite the Tweedle algorithm being simple, the state machine implementing it instead isn't. As the algorithm gets spread out among states, reading and following the tweedle logic in the code, becomes counterintuitive. To make things more complicated,this particular state machine was implemented using transtional logic rather than topology only as state transition criteria. This makes the number of used states smaller at the expanse of code readability.  
The reason a state machine was used is because of the need to transfer control between the tweedle algorithm logic and the simulator. State transition logic is a natural representaton of such problems and has the advantage of retaining the latest state even when the connection with the simulator was to be lost half way during the parameters estimation process.   
Being able to restart the process without problems is a signficant advantge and therefore the state machine was implemented. 
As the tweedle state machine code is however outside the scope of the project, no detailed comments or documentation are made available.





