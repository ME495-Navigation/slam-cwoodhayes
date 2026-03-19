# NUSLAM

## Demonstration in Nusim

https://github.com/user-attachments/assets/8f95d7c8-a8a6-441a-91ff-cd5723f86deb

> Driving the robot around a closed path, observing the environment only using (simulated) lidar. The robot builds a map online (green) using EKF-SLAM, with circle detection + clustering to detect landmarks (blue) and data association using Mahalanobis distance. The map is visualized in RViz, and the ground truth map (red) + odometry-based estimate (blue robot) is shown for comparison. Note that when the robot collides with an obstacle, the odometry-based estimate diverges significantly, while the SLAM estimate remains accurate by leveraging the landmarks in the environment.

After this closed loop is complete, the ground truth, odometry, and SLAM poses are as follows:

- Ground Truth (red)
	- (x, y, z) = -0.0038688; 0.029482; 0
	- (qx, qy, qz, qw) = 0; 0; -0.099636; 0.99502
- SLAM estimate (green)
	- (x, y, z) = 0.016987; 0.01215; 0
	- (qx, qy, qz, qw) = 0; 0; -0.094745; 0.9955
- Odometry-only estimate (blue)
	- (x,y,z) = -1.1721; -0.84761; 0
	- (qx, qy, qz, qw) = 0; 0; -0.056191; 0.99842


**Final SLAM position error**: 0.027m
**Final Odometry-only position error**: 1.46m