# ME495 Sensing, Navicagion, and Machine Learning for Robotics
* Conor Hayes
* Winter 2025

# Package List
This repository consists of several ROS packages:
- `nuturtle_description` - contains models, configs, and visualization files for the `turtlebot3` burger, adapted from the official [turtlebot3_description](https://index.ros.org/p/turtlebot3_description/) package.

# Nuturtle Description
URDF files for Nuturtle 
* `ros2 launch nuturtle_description load_one.launch.xml` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.
![](images/rviz.png)
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![](images/rqt_graph.svg)

# Launch File Details
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
This launchfile accepts no arguments; however, the above command will display 
arguments from the `load_one` launchfile as above due to a known bug in `launch`. 