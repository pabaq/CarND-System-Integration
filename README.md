This is the final project of Udacity's [Self-Driving-Car Nanodegree][Course]. The project
description and build instructions can be found [here][Project], the required simulator [here][Simulator].

The goal of this project is the integration of the ***perception***, ***planning***, and 
***control*** subsystems of an autonomous vehicle using the Robot Operating System ([ROS][ROS])
to guide a self-driving car on a simulated highway track while taking into account upcoming 
traffic signals.

![][Final]

## System Architecture
![][Architecture]

The system architecture depicted above shows all the ROS nodes and topics involved in the 
project. The main focus lies on the implementation and integration of the following three nodes:

**Traffic light detection**: This node is part of the *perception* subsystem. It is responsible 
for the detection and classification of the traffic lights. If a red signal is detected, the node 
will publish the position of the traffic light's stop line to the `/traffic_waypoint` topic.

**Waypoint updater**: This node is part of the *planning* subsystem. It is responsible for the 
generation of a suitable trajectory to be followed by the vehicle. In general, the vehicle will 
follow the reference trajectory provided by the `/base_waypoints` topic, and should stop only if a 
red traffic light was detected. In these cases, the node will update the trajectory in such a way 
that vehicle comes to a halt at the location specified by the `/traffic_wapoint` topic. The updated
trajectory will be published to the `/final_waypoints` topic.

**Drive by wire controller**: This node is part of the *control* subsystem. It is responsible 
for the longitudinal and lateral motion control of the vehicle. PID and Stanley controllers are 
implemented to provide the appropriate throttle, brake, and steering commands that make the vehicle 
follow the trajectory proposed by planning subsystem above.

In the following, we will take a closer look at the implementation details of each subsystem.

## Perception subsystem

### Traffic Light Detection
The traffic light detection node is part of the [`/ros/src/tl_detector/`][DetectorPackage] package 
and it is implemented in [`tl_detector.py`][Detector].

![][PerceptionNode]

As mentioned above, the node is in charge of detecting the incoming traffic lights and classifying 
their signal states. In order to do so, it subscribes to the following topics: 

- `/base_waypoints` - A list of waypoints representing the reference trajectory on the map. 
Published by the [Waypoint Loader](#waypoint-loader) node.
- `/image_color` - The current RGB image received from the vehicle camera. Published by the simulator.
- `/current_pose` - The current position of the vehicle. Published by the simulator.
- `/vehicle/traffic_lights` - The positions of all the traffic lights on the map. Published by the 
simulator.

The node will search for the next upcoming traffic light within a look ahead distance of 
[NUM_LOOKAHEAD][lookahead] waypoints in the front of the vehicle's current pose. If a traffic 
light is detected, the node will classify its signal state using the images from the incoming 
camera stream. If the traffic light signal is red, the node will communicate the position of the 
corresponding stop line. This is done by searching the reference waypoint that is closest to the 
stop line in the `/base_waypoints` list, and publishing its index to the `/traffic_waypoint` topic.
This topic will then be used by the [Waypoint Updater](#waypoint-updater) node in the planning 
subsystem to update the trajectory such that the vehicle comes to a halt at the stop line.

### Traffic Light Classification
The traffic light classifier is implemented in [`tl_detector/light_classifier/tl_classifier.py`][Classifier]. 
Up to now, the classification is solved by using color thresholding in the hsv color space. Three 
masks are defined to isolate the red, yellow and green areas in the image. The summation of the 
active pixels in each mask represent the intensities of each color. Finally, the color with the 
highest intensity indicates the color of the current traffic light. In the future, we may come 
back to this and replace this classification approach with a more robust one that uses Tensorflow's
Object Detection API. 

### Obstacle Detection
The obstacle detection node is not required in this project.


## Planning subsystem
The planning subsystem is responsible for the generation of a suitable trajectory to be followed by 
the vehicle. Two nodes are implemented to achieve this.

### Waypoint Loader
The waypoint loader node is part of the [`/ros/src/waypoint_loader/`][WaypointLoaderPackage] package 
and it is implemented in [`waypoint_loader.py`][WaypointLoader]. Its only purpose is to load the 
reference trajectory to be followed by the vehicle and to publish it to the `/base_waypoints` topic. 

![][LoaderNode]

The reference trajectory is represented by a list of waypoints, of which each holds the information 
about a future target [`Pose`][Pose] (location/orientation) and [`Twist`][Twist] (linear/angular 
velocity) of the vehicle. The reference trajecory simply guides the vehicle along a predetermined 
path on the highway while maintaining a certain speed limit specified in the corresponding 
[launch][LoaderLaunch] file. However, the reference waypoints are static and do not yet allow the 
vehicle to stop at red lights. To achieve this, they need to updated, and this is performed by the 
waypoint updater node.

### Waypoint Updater
The waypoint updater node is part of the [`/ros/src/waypoint_updater/`][WaypointUpdaterPackage] 
package and it is implemented in [`waypoint_updater.py`][WaypointUpdater].

![][UpdaterNode]

The node is responsible to update the reference waypoints taking into account the traffic lights in 
the vicinity. To accomplish this, it subscribes to the following topics:

- `/base_waypoints` - A list of waypoints representing the reference trajectory on the map.
Published by the [Waypoint Loader](#waypoint-loader) node.
- `/current_pose` - The current position of the vehicle. Published by the simulator.
- `/traffic_waypoint` - Index to the reference waypoint in the `/base_waypoints` list that is closest 
to the stop line of the upcoming red traffic light. Published by the 
[Traffic Light Detection](#traffic-light-detection) node.
- `/obstacle_waypoints` - Not required for this project.

The node will update the next [NUM_LOOKAHEAD][lookahead] reference waypoints in front of the 
vehicle. If a red traffic light is detected (indicated by the `/traffic_waypoint` topic), the speed 
of each waypoint between the current position of the vehicle and the stop line will be reduced using 
a smooth decelerating curve. Otherwise, the original reference waypoints are kept unchanged and the 
vehicle will continue to follow its designated trajectory. The list of updated waypoints are 
published to the `/final_waypoints` topic that will be used by the control subsystem.

## Control subsystem
The control subsystem is responsible for the longitudinal and lateral motion control of the vehicle. 
Two nodes are implemented to achieve this.

### Waypoint Follower
The waypoint follower node is part of the [`/ros/src/waypoint_follower/`][WaypointFollowerPackage] 
package and it is provided for this project using code from [Autoware][Autoware]. 

![][FollowerNode]

The node is responsible to generate the linear and angular target velocities that the vehicle shall
adopt. To do this, it uses the updated trajectory that is published to the `/final_waypoints` topic 
by the [Waypoint Updater](#waypoint-updater) node . The determined target velocities are then 
published themselves to the `/twist_cmd` topic in form of [Twist][Twist] messages. The target 
velocities will be used by the [Drive by Wire](#drive-by-wire) node to generate approriate control 
commands.

### Drive by Wire
The Drive by Wire node is part of the [`/ros/src/twist_controller/`][ControlPackage] package and it 
is implemented in [`dbw_node.py`][DBWNode].

![][ControlNode]

The node is responsible for the generation of appropriate throttle, brake, and steering commands 
that make the vehicle follow the updated trajectory proposed by planning subsystem. To achieve this, 
it subscribes to the following topics:

- `/base_waypoints` - A list of waypoints representing the reference trajectory on the map. 
Published by the [Waypoint Loader](#waypoint-loader) node.
- `/current_pose` - The current position of the vehicle. Published by the simulator.
- `/current_velocity` - The current linear and angular velocities of the vehicle. Published by the 
simulator.
- `/twist_cmd` - The target linear and angular velocities for the vehicle. Published by the 
[Waypoint Follower](#waypoint-follower) node.
- `/vehicle/dbw_enabled` - The drive by wire status (true/false). Published by the simulator.

The node uses the `Controller` class implemented in [`twist_controller.py`][Controller] to generate 
the throttle, brake and steering commands and publishes them to the `/vehicle/throttle_cmd`,
`/vehicle/brake_cmd`, and `/vehicle/steering_cmd` topics. These are then used as input for the
simulator.

#### Lateral Control
A Stanley controller is implemented in [`stanley.py`][StanleyController] to take care of the lateral 
motion control. The Stanley method is the path tracking approach used by Stanford University’s Darpa 
Grand Challenge team. The theoretical details of this controller can be found in [[1]](#references). 

![][StanleyImg]

The Stanley method uses the front axle as its reference point and simultaneously corrects the 
heading error &psi; and the cross-track error *e*, while obeying the maximum steering angle 
bounds &delta;<sub>min</sub> and &delta;<sub>max</sub> . It requires the two gains *k* and 
*k<sub>s</sub>*, that typically need to be determined experimentally. After some tuning in the 
simulator the gains were chosen as follows:

> ``K = 0.1``  
> ``Ks = 1.0``

#### Longitudinal Control
Two seperate PID controllers are implemented for the longitudinal motion control, one of which 
generates the throttle commands and the other the brake commands. The speed error between the target 
and current velocity of the vehicle determines which of both controllers is active. For instance, if
the speed error is positive, i.e. the vehicle is too slow, the throttle controller is activated and 
vice versa. Throttle and brake commands cannot be applied simultaneously. A [low pass][LowPassFilter] 
filter is implemented to remove the high frequency noise from the incoming vehicle velocity. 

The simulator requires the throttle commands to be normalized to [0, 1], wheras the brake commands 
must be provided as a physical torque value. The torque can be computed as ``T = m * a * r ``, 
with the vehicle mass *m*, the deceleration *a*, and the wheel radius *r*, whereby the control value
returned by the brake controller is used as deceleration *a*.

After some tuning in the simulator the gains were chosen as follows:

> ``# PID gains for the throttle controller``  
> ``Kp = 0.3``  
> ``Ki = 0.1``  
> ``Kd = 0.0``  

> ``# PID gains for the brake controller``  
> `Kp = 0.2`  
> `Ki = 0.0`  
> `Kd = 0.1`  

The integral gain of the throttle controller ensures a higher accuracy to maintain the target speed 
while driving. However, no integral gain was defined for the brake controller, since no residual brake demand is required when the speed target changes, e.g. due to a traffic light switching from 
red to green.

## References
[[1]][StanleySource] Hoffmann, Gabriel M., Claire J. Tomlin, Michael Montemerlo, and Sebastian Thrun.
Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental
Validation and Racing. *American Control Conference*, 2007, pp. 2296–2301


[Course]: https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
[Project]: https://github.com/udacity/CarND-Capstone
[Simulator]: https://github.com/udacity/CarND-Capstone/releases
[ROS]: https://www.ros.org/
[Autoware]: https://github.com/CPFL/Autoware

[Final]: https://github.com/pabaq/CarND-System-Integration/raw/main/videos/Final_8x_480.gif 
[Architecture]: https://github.com/pabaq/CarND-System-Integration/raw/main/images/Architecture.png
[PerceptionNode]: https://github.com/pabaq/CarND-System-Integration/raw/main/images/TrafficLightDetectionNode.png
[LoaderNode]: https://github.com/pabaq/CarND-System-Integration/raw/main/images/WaypointLoaderNode.png
[FollowerNode]: https://github.com/pabaq/CarND-System-Integration/raw/main/images/WaypointFollowerNode.png
[UpdaterNode]: https://github.com/pabaq/CarND-System-Integration/raw/main/images/WaypointUpdaterNode.png
[ControlNode]: https://github.com/pabaq/CarND-System-Integration/raw/main/images/DBWNode.png
[StanleyImg]: https://github.com/pabaq/CarND-System-Integration/raw/main/images/Stanley.png
[StanleyEq]: https://github.com/pabaq/CarND-System-Integration/raw/main/images/StanleyEquation.png

[DetectorPackage]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/tl_detector/
[Detector]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/tl_detector/tl_detector.py
[Classifier]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/tl_detector/light_classification/tl_classifier.py
[WaypointLoaderPackage]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/waypoint_loader/
[WaypointLoader]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/waypoint_loader/waypoint_loader.py
[Pose]: https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html
[Twist]: https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
[LoaderLaunch]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/waypoint_loader/launch/waypoint_loader.launch
[WaypointUpdaterPackage]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/waypoint_updater/
[WaypointUpdater]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/waypoint_updater/waypoint_updater.py
[WaypointFollowerPackage]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/waypoint_follower/
[ControlPackage]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/twist_controller/
[DBWNode]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/twist_controller/dbw_node.py
[Controller]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/twist_controller/twist_controller.py
[StanleyController]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/twist_controller/stanley.py
[LowPassFilter]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/twist_controller/lowpass.py
[lookahead]: https://github.com/pabaq/CarND-System-Integration/tree/main/ros/src/utilities/src/utilities/__init__.py#L4

[StanleySource]: https://ieeexplore.ieee.org/document/4282788