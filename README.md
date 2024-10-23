# Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario
**Project Description**
This is a project that **simulates the work of multiple robots in a warehouse scenario** and is developed in three steps
1. Control a differential drive robot to navigate in a warehouse
2. Control a robot moving in three locations in the warehouse
3. Control multiple robots move in a warehouse
**Development Environment:**
+ MATLAB Version: R2024a
+ Modeling Tools: Simulink, Robotics System Toolbox
+ Robot Model: Differential Drive Model

**Path planning algorithm:**
+ Probabilistic Road Map(PRM)

![111](https://se.mathworks.com/help/releases/R2024a/examples/robotics/win64/ControlAndSimulateMultipleWarehouseRobotsExample_13.png)

# 1. Control a differential drive robot to navigate in a warehouse

> execute an obstacle-free path planning between two locations on a given map in Simulink. 
>
> The path is generated using a probabilistic road map(PRM) planning algorithm [mobileRobotPRM]
>
> Control commands for navigating this path are generated using the **Pure Pursuit** controller block.
>
> A differential drive kinematic motion model simulates the robot motion

## 1.1 Load the Map and Simulink Model

Load the occupancy map, which defines the map limits and obstacles within the map

```matlab
load [name of your map].mat      %load the data into base workspace
```

Specify a start and end location within the map

````matlab
startLoc = [5 5];
goalLoc = [20 20];
````

## 1.2 Model Overview

open the simulink model

````matlab
open_system('pathPlanningSimulinkModel.slx')
````

The model is composed of 3 primary parts

+ Planning
+ Control
+ Plant Model

![image-20240928111640929](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/1.2_1.png)

### 1.2.1 Planning

<img src="https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/1.2.1_1.png" alt="image-20240928111608692" style="zoom:35%;" />

+ The planner function block uses the ***mobileRobotPRM*** path planner 

+ **Inputs**:

  1. start location
  2. goal location
  3. map

+ **Outputs**:

  1. an array of **waypoints**.

+ **Process**:

  The robot follows these waypoints to reach the goal position.

  The planned waypoints are used downstream by the **Pure Pursuit controller block**.

### 1.2.2 Control

<img src="https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/1.2.2_1.png" alt="img" style="zoom:35%;" />

#### 1.2.2.1 Pure Pursuit

+ **Inputs**:
  1. Pose of the robot [from the robot state]
  2. waypoints from the planner
+ **Outputs**:
  1. Linear velocity commands
  2. Angular velocity commands

#### 1.2.2.2 Check Distance to Goal

the subsystem calculates the current distance to the goal and if it is within a threshold, the simulation stops

+ **Inputs**:
  1. robotGoal [the same as the goal location]
  2. currentPose [from the robot state]
+ **Ouputs**:
  1. atGoal [ bool digit]

#### 1.2.2.3 Zero-Velocity at goal

to check if it is within the goal

the final velocity = is_atgoal * velocity

### 1.2.3 Plant Model

<img src="https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/1.2.3_1.png" style="zoom:35%;" />

The **Differential Drive Kinematic Model** block creates a vehicle model to simulate simplified vehicle kinematics.

+ **Inputs**:
  1. linear velocity $v$ from Pure Pursuit controller block
  2. angular velocity $\omega$ from Pure Pursuit controller block
+ **Outputs**:
  1. current position
  2. current velocity states

## 1.3 Run the Model

````matlab
simulation = sim('pathPlanningSimulinkModel.slx');
````

## 1.4 Visualize the Motion of Robot

````matlab
map = binaryOccupancyMap([the name of you map]);
robotPose = simulation.Pose;
thetaIdx = 3;

% Translation
xyz = robotPose;
xyz(:, thetaIdx) = 0;

% Rotation in XYZ euler angles
theta = robotPose(:,thetaIdx);
thetaEuler = zeros(size(robotPose, 1), 3 * size(theta, 2));
thetaEuler(:, end) = theta;

% Plot the robot poses at every 10th step.
for k = 1:10:size(xyz, 1) 
    show(map)
    hold on;
    
    % Plot the start location.
    plotTransforms([startLoc, 0], eul2quat([0, 0, 0]))
    text(startLoc(1), startLoc(2), 2, 'Start');
    
    % Plot the goal location.
    plotTransforms([goalLoc, 0], eul2quat([0, 0, 0]))
    text(goalLoc(1), goalLoc(2), 2, 'Goal');
    
    % Plot the xy-locations.
    plot(robotPose(:, 1), robotPose(:, 2), '-b')
    
    % Plot the robot pose as it traverses the path.
    quat = eul2quat(thetaEuler(k, :), 'xyz');
    plotTransforms(xyz(k,:), quat, 'MeshFilePath',...
        'groundvehicle.stl');
    light;
    drawnow;
    hold off;
end
````

# 2. Control the robot move in three locations

> + This example demonstrates how to execute an obstacle-free path planning for a mobile robot between three locations on a given map. 
>   + The robot is expected to visit the three locations in a warehouse: a charging station, loading station, and unloading location.
>
> + The sequence in which these locations are visited is dictated by a scheduler.
>
> + The scheduler gives each robot a goal pose to navigate to. 
>
> + The robot plans a path and uses a Pure Pursuit controller to follow the waypoints based on the current pose of the robot.
>
> + The **Differential Drive Kinematic Model** block models the simplified kinematics, which takes the linear and angular velocities from the Pure Pursuit Controller.



## 2.1 Warehouse Map

### 2.1.1 Load the map

````matlab
load warehouseMaps.mat logicalMap
map = binaryOccupancyMap(logicalMap);
show(map)
````

### 2.1.2 Assign charging station, sorting (loading) station, and the unloading location

````matlab
chargingStn = [5,5];
loadingStn = [52,15];
unloadingStn = [15,42];
````

Show the various locations on the map.

```matlab
hold on;

text(chargingStn(1), chargingStn(2), 1, 'Charging');
plotTransforms([chargingStn, 0], [1 0 0 0])

text(loadingStn(1), loadingStn(2), 1, 'Sorting Station');
plotTransforms([loadingStn, 0], [1 0 0 0])

text(unloadingStn(1), unloadingStn(2), 1, 'Unloading Station');
plotTransforms([unloadingStn, 0], [1 0 0 0])

hold off;
```

## 2.2 Model Overview

A Simulink® model is provided that models all aspects of the system for scheduling, planning, controlling, and modelling the robot behavior.

```matlab
open_system('warehouseTasksRobotSimulationModel.slx')
```

![image-20240928125402215](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/2.2_1.png)

### 2.2.1 Planning, Control and Plant Model

![image-20240928125620845](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/2.2.1_1.png)

The model uses a planning, control, and plant model similar to the [Plan Path for a Differential Drive Robot in Simulink](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/1.2_1.png) example.

#### 2.2.1.1 Planner

+ **inputs**:
  1. start location from the scheduler
  2. goal location from the scheduler
  3. given map
+ **Outputs**:
  1. waypoints

#### 2.2.1.2 Controller

#### 2.2.1.3 Plant Model

### 2.2.2 Robot Scheduler

The Scheduler block assigns start and goal locations to the robot. 

The current pose of the robot is used as a starting location and the end location is determined by a sequence of tasks specified inside the scheduler. 

**Sequence:**

1. Starts from the charging location, and goes to the loading location.

2. Pauses as the loading station to load the package and plans a path to the unloading location.

3. Navigates to the unloading station to unload the package. Replans a path to the charging station.

4. Stops at the charging station.

   

![img](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/2.2.2_1.png)

### 2.2.3 Simulate the Robot

```matlab
simulation = sim('warehouseTasksRobotSimulationModel.slx');
```

### 2.2.4 Visualize Robot Trajectories

A custom visualization tool is given to mimic a distributed camera system and get more detailed views of the robot trajectory at certain locations in the map.

Open the **Visualization Helper** block and use the **Preset Views** drop-down to select different perspectives. The `Sample time` of the visualization has no effect on the simulation of the robot.

# 3. Control and Simulate Multiple Warehouse Robots

> + This example shows how to control and simulate multiple robots working in a warehouse facility or distribution center. 
>
> + The robots drive around the facility picking up packages and delivering them to stations for storing or processing. 
>   + A **Central Scheduler** sends commands to robots to pick up packages from the *loading station* and deliver them to a specific *unloading station*.
>   + The **Robot Controller** plans the trajectory based on the locations of the loading and unloading stations, and generates velocity commands for the robot. 
>   + These commands are fed to the **Plant**, which contains a differential-drive robot model for executing the velocity commands and returning ground-truth poses of the robot. 
>   + The poses are fed back to the scheduler and controller for tracking the robot status. This workflow is done for a group of 5 robots, which are all scheduled, tracked, and modeled simultaneously.
>
> + This package-sorting scenario can be modeled in Simulink® using Stateflow charts and Robotics System Toolbox™ algorithm blocks.

<img src="https://se.mathworks.com/help/examples/robotics/win64/ControlAndSimulateMultipleWarehouseRobotsExample_01.png" alt="img" style="zoom:80%;" />

## 3.1 Model Overview

![image-20240928142648288](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.1_1.png)

### 3.1.1 Central Scheduler

<img src="https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.1.1_1.png" style="zoom: 67%;" />

#### 3.1.1.1 For Each Robot and Package State

> This subsystem is a For-Each Subsystem which processes an array of buses for tracking the robot and package states as **RobotPackageStatus** bus object
>
> This makes it easy to update this model for varying number of robots.

+ **Inputs**:

  1. robotpackage

  > Each robot can carry one package at a time 
  >
  > And is instructed to go from the loading to an unloading station based on the required location for each package

+ **Outputs**:

  1. PlanningState
  2. hasPackage

#### 3.1.1.2 Scheduler

+ **Inputs**:
  1. currentPoses: tracks the status of the robots
  2. packages dispenser: tracks the status of the packages
  3. PlanningState
  4. hasPackage
+ **Outputs**:
  1. STOP commands: based on robot poses when detects an imminent collision
  2. delivery
  3. track
  4. metricinfo

![img](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.1.1.2_1.png)

### 3.1.2 Roboto Controller

The Robot Controller uses a For-Each Subsystem to generate an array of robot controllers for 5 robots.

<img src="https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.1.2_1.png" style="zoom:67%;" />

Each robot controller has the following inputs and outputs.

![img](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.1.2_2.png)

![img](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.1.2_3.png)

The controller takes delivery commands, which contains the package information, and plans a path for delivering it someone in the warehouse using mobileRobotPRM. 

![image-20240928150553774](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.1.2_4.png)

The **Pure Pursuit** block takes this path and generates velocity commands for visiting each waypoint. Also, the status of the robot and packages get updated when the robot reaches its goal.

Each robot also has its own internal **scheduler** that tells them the location of unloading stations based on the package information, and sends them back to the loading station when they drop off a package.

The robot controller model uses the same model, `warehouseTasksRobotSimulationModel`, shown in Execute Tasks for a Warehouse Robot.

![image-20240928150525858](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.1.2_5.png)

### 3.1.3 Plant

The **Plant** subsystem uses a **Differential Drive Kinematic Model** block to model the motion of the robots.

<img src="[https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.1.3_1.png" style="zoom:67%;" />

<img src="https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.1.3_2.png" style="zoom: 67%;" />

## 3.2 Model Setup

### 3.2.1 Defining the Warehouse Environment

````matlab
load multiRobotWarehouseMap.mat logicalMap loadingStation unloadingStations chargingStations
warehouseFig = figure('Name', 'Warehouse Setting', 'Units',"normalized", 'OuterPosition',[0 0 1 1]);
visualizeWarehouse(warehouseFig, logicalMap, chargingStations, unloadingStations, loadingStation);
````

### 3.2.2 Checking occupancy at stations

Ensure that the stations are not occupied in the map.

````matlab
map = binaryOccupancyMap(logicalMap);
if(any(checkOccupancy(map, [chargingStations; loadingStation; unloadingStations])))
    error("At least one of the station locations is occupied in the map.")
end
````

### 3.2.3 Central Scheduler

The Central Scheduler requires the knowledge of the packages needed to be delivered in order to send the delivery commands to the robot controllers.

### 3.2.4 Defining Packages

Packages are given as an array of index numbers of the various unloading stations that the packages are supposed to be delivered to.

Because this example has three unloading stations, a valid package can take a value of 1, 2, or 3.

````matlab
load packages.mat packages
% packages
````

### 3.2.5 Number of Robots

The number of robots is used to determine the sizes of the various signals in the initialization of the **Scheduler** Stateflow chart

````matlab
numRobots = size(chargingStations, 1); % Each robot has its own charging station; the number of the chargingstations equal to the number of robots
````

### 3.2.6 Collision Detection and Goal-Reached Threshold

The **Central Scheduler** and the **Robot Controller** use certain thresholds for collision detection, `collisionThresh`, and a goal-reached condition, `awayFromGoalThresh`.

+ **Rules**:

1. Collision detection ensures that for any pair of robots within a certain distance-threshold,  the robot with a lower index should be allowed to move while the other robot should stop[zero-velocity command]

2. The still moving robot should be able to avoid local static obstacles in their path.

This could be achieved with another low-level controller like the Vector Field Histogram (Navigation Toolbox) block

The goal-reached condition occurs if the robot is within a distance threshold, awayFromGoalThresh, from the goal location.

````matlab
load exampleMultiRobotParams.mat awayFromGoalThresh collisionThresh
````

### 3.2.7 Bus Objects

The **RobotDeliverCommand** and **RobotPackageStatus**[in robot controller] bus objects are used to pass robot-package allocations between the **Central Scheduler** and the **Robot Controller**.

````matlab
load warehouseRobotBusObjects.mat RobotDeliverCommand RobotPackageStatus
````

## 3.3 Simulation

````matlab
open_system("multiRobotExampleModel.slx")
sim('multiRobotExampleModel');
````

### 3.3.1 Metrics and Status Dashboard

For each of the packages, the dashboard in the model shows if the package is "InProgress", "Unassigned", or "Delivered". Robot Status displays the distance travelled, package location, and a package ID.

![img](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.3.1_1.png)

## 3.4 Extending the Model

This model is setup to handle modifying the number of robots in the warehouse based on availability. 

### 3.4.1 Adding more robots requires defining additional charging stations.

````matlab
chargingStations(6, :) = [10, 15]; % Charging Station for the additional 6th robot
chargingStations(7, :) = [10, 17];  % Charging Station for the additional 7th robot
````

### 3.4.2 Add more unloading stations and assign packages to it

````matlab
unloadingStations(4, :) = [30, 50];
packages = [packages, 4, 4];
````

### 3.4.3 Additional Differential Kinematic Model blocks are also required to match the number of robots

The exampleHelperReplacePlantSubsystem adds these by updating numRobots.

````matlab
numRobots = size(chargingStations, 1) % As before, each robot has its own charging station 
exampleHelperReplacePlantSubsystem('multiRobotExampleModel/Robots', numRobots);
````

### 3.4.4 Redefine any existing locations

````matlab
loadingStation = [35, 20];
````

## 3.5 Visualization

![img](https://github.com/Aynlai/Control-and-Simulate-Multiple-Robots-in-a-Warehouse-Scenario/blob/main/imgs/3.5_1.png)







