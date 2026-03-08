# ME495 Sensing, Navigation, and Machine Learning for Robotics
* Conor Hayes
* Winter 2025

# Setup
## System Requirements
This repo has been tested with the following system configuration:
- ROS 2 Kilted Kaiju
- g++-14 compiler
- Ubuntu 24.04

It likely works with other versions of the above, but this is untested.

## One-time setup
In order to build & use this package, first perform the following steps:
```bash
# clone the repo into a clean ROS workspace
SLAM_WS_NAME="slam-ws"   # use whatever workspace name you want
mkdir -p $SLAM_WS_NAME/src
cd $SLAM_WS_NAME/src
git clone git@github.com:ME495-Navigation/slam-cwoodhayes.git

# run one-time setup for dependency installation
./slam-cwoodhayes/install_dependencies.sh

# build package from source
cd ..
colcon build
```

# Package List
This repository consists of several ROS packages:
- `nuslam` - SLAM + odometry estimation package for turtlebot3
- `turtle_control` - Control functionality for the turtlebot3. Supports odometry + control of the robot in simulation and in hardware.
- `nuturtle_description` - contains models, configs, and visualization files for the `turtlebot3` burger, adapted from the official [turtlebot3_description](https://index.ros.org/p/turtlebot3_description/) package.
- `nusim` - a custom turtlebot arena simulator based on RViz which supports our SLAM algorithm development
- `turtlelib` - pure C++ library that implements helper code for the packages above (geometry, kinematics, EKF + SLAM abstract implementations)

# nuslam Description
Implements EKF-SLAM for the turtlebot3.

https://github.com/user-attachments/assets/98b15d36-1264-4295-b9a9-0fb8c69cb47d

> Above: The result of running `ros2 launch nuslam slam.launch.xml cmd_src:=teleop robot:=nusim use_rviz:=true`, then driving the robot around. Red obstacles, walls, and robot are ground-truth. Yellow obstacles are noisy landmark measurements provided by `nusim` and consumed by `nuslam`. Blue robot is an odometry-only pose estimate. Green robot + obstacles are the `nuslam` SLAM pose + map estimate, with robot pose covariance as an ellipse + angular field of view (purple & yellow, respectively). **Note that when the ground truth collides with an obstacle, the odometry estimate keeps driving, but the SLAM estimate correctly stays with ground truth.** See [here](images/slam_result.png) for a still image of the final poses.

Node summary:
- `slam_node` (`nuslam_node` in launch): computes wheel-odometry and EKF-SLAM pose estimates, publishes `odom` (pure odometry-based estimate, like the `odometry` node below) and `/slam/pose` (odometry + LiDAR-based combined estimate using EKF SLAM), and broadcasts TF `map->odom->[robot]`.
    - EKF parameters, simulated noise configuration, and more are available as parameters (see `nuslam/config/*`)

## Launch File Details
The `slam.launch.xml` launchfile starts robot control (sim or hardware), RViz visualization, the SLAM node, and supporting TF/static transforms.

* `ros2 launch nuslam slam.launch.xml --show-arguments`

```
Arguments (pass arguments as '<name>:=<value>'):

  'cmd_src':
    Source of cmd_vel commands. Valid choices are: ['circle', 'teleop', 'none']
    (default: 'circle')

  'robot':
    what robot to control (sim vs hardware etc). Valid choices are: ['nusim', 'localhost', 'none']
    (default: 'nusim')
```

# turtle_control Description
Controls the turtlebot3 in simulation or hardware and provides odometry. Node summary:
- `turtle_control` node: converts `cmd_vel` into wheel commands, publishes wheel joint states using IK/FK implementation in `turtlelib::DiffDrive`
- `odometry` node: computes and publishes `nav_msgs/Odometry`, broadcasts TF, and provides `set_initial_pose` service.
- `circle` node: publishes circular `cmd_vel` and offers `circle_control`, `reverse`, and `stop` services.

The `start_robot.launch.xml` launch file runs these nodes for motion in `nusim` or hardware, with configurable source of `cmd_vel` (a simple circular path, teleop, or none)

https://github.com/user-attachments/assets/bab2ae6e-a6a7-44c5-8ed0-ffb4ad08dbb6

> Above: Running `ros2 launch turtle_control start_robot.launch.xml robot:=localhost use_rviz:=false` on the turtlebot3, together with the rviz output on PC from `ros2 launch src/slam-cwoodhayes/turtle_control/launch/start_robot.launch.xml cmd_src:=none robot:=none use_rviz:=true`. The turtlebot reverses direction in response to manual calls of the `/reverse` service.

In the video above, according to `/odom` the turtlebot stops at:
```
  pose:
    position:
      x: 0.1207919554875541
      y: 0.03904022617800196
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.2537207775181715
      w: 0.9672775026100703
```

## Launch File Details
* `ros2 launch turtle_control start_robot.launch.xml --show-arguments`

```
Arguments (pass arguments as '<name>:=<value>'):

    'cmd_src':
        Source of cmd_vel commands. Valid choices are: ['circle', 'teleop', 'none']
        (default: 'circle')

    'robot':
        what robot to control (sim vs hardware etc). Valid choices are: ['nusim', 'localhost', 'none']
        (default: 'nusim')

    'use_rviz':
        if true, view robot behavior in RViz
        (default: 'true')
```
Note that the above command will display 
additional arguments from included launchfiles (not shown above) due to a known bug in the launch framework.

# Nuturtle Description
URDF files for Nuturtle 
* `ros2 launch nuturtle_description load_one.launch.xml` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.
![](images/rviz.png)
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![](images/rqt_graph.svg)

## Launch File Details
* `ros2 launch nuturtle_description load_one.launch.xml --show-arguments`
```
  Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        Launch RViz for visualization
        (default: 'true')

    'use_jsp':
        Launch joint_state_publisher for default joint states
        (default: 'true')

    'color':
        Determines namespace for nodes launched by this file, and colors the URDF model. Valid choices are: ['red', 'green', 'blue', 'purple']
        (default: 'purple')
```
* `ros2 launch nuturtle_description load_all.launch.xml --show-arguments`

The `load_all` launchfile accepts no arguments; however, the above command will display 
arguments from the `load_one` launchfile as shown above due to a known bug in `launch`. 

# turtlelib Description
Implements geometric primitives and operations upon them.
- `angle.hpp` - helper functions for working with angles in degrees and radians
- `geometry2d.hpp` - two-dimensional geometric primitives (Points, Vectors) and operations upon them
- `se2d.hpp` - two-dimensional SE2 transformations + twists, that can operate on points + vectors
- `svg.hpp` - visualization functions for the above using SVG files as output.
- `diff_drive.hpp` - handles inverse and forward kinematics for an arbitrary diff-drive robot. For derivations of the math used, see [doc/Kinematics.pdf](doc/Kinematics.pdf)

# nusim Description
A custom turtlebot arena simulator based on rviz. 

## Simulator Setting Parameters
The following parameters are available on the `nusimulator` node provided
by this package to control the simulation:

- `rate` - Simulation rate in Hz (default: 100.0)
- `x0` - Initial ground-truth x position of the turtlebot (default: 0.0)
- `y0` - Initial ground-truth y position of the turtlebot (default: 0.0)
- `theta0` - Initial ground-truth orientation of the turtlebot (default: 0.0)
- `arena_x_length` - Length of the arena in the world X direction (default: 10.0)
- `arena_y_length` - Length of the arena in the world Y direction (default: 10.0)
- `obstacles.x` - X coordinates of obstacles (default: empty)
- `obstacles.y` - Y coordinates of obstacles (default: empty)
- `obstacles.r` - Radius of obstacles (default: 0.0)


## Launch File Details
This package has one launch file (`nusim.launch.xml`) which spawns a single
robot in an rviz view, as well as arena walls and cylindrical obstacles.
The robot's position can be reset to its spawn point with the `/reset` service.

* `ros2 launch nusim nusim.launch.xml --show-arguments`:
```
Arguments (pass arguments as '<name>:=<value>'):

    'config_file':
        YAML file to configure the simulator.
        (default: 'config/basic_world.yaml')
```

![rviz screenshot from nusim](nusim/images/nusim1.png)