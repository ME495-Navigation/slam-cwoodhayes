# NUSLAM

## Demonstration in Nusim

https://github.com/user-attachments/assets/ea1a8a6e-004b-4681-86aa-999a3f587f48

> Driving the robot around a closed path, observing the environment only using (simulated) lidar. The robot builds a map online (green) using EKF-SLAM, with circle detection + clustering to detect landmarks (white) and data association using Mahalanobis distance. The map is visualized in RViz, and the ground truth map (red) + odometry-based estimate (blue robot) is shown for comparison. Note that when the robot collides with an obstacle, the odometry-based estimate diverges significantly, while the SLAM estimate remains accurate by leveraging the landmarks in the environment.

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


## Demonstration in Real World

https://github.com/user-attachments/assets/1179359c-8443-42d2-a3f8-9bd3460544c9

> Above: Running `nuslam`'s SLAM algorithm in real-time on the  turtlebot3. Key: Green/multicolored dots: LIDAR scan points. Green obstacles+robot+path = SLAM estimate. Blue robot+path = odometry-only estimate. White obstacles=landmarks detected by the `landmarks` node (see below), which feed into SLAM. SLAM's robot pose covariance is shown as the purple ellipse and yellow cone. **Note that the odometry-only estimate gradually diverges from the ground truth as shown in the real-world video on the left, while the SLAM estimate correctly stays with ground truth.** See [here](images/slam_irl_result.png) for a still image of the robot poses at the end of the video demonstrating this.

After this closed loop is complete, with the robot parked roughly at the real-world starting point, the odometry and SLAM poses are as follows:

- Ground Truth (top left video -- rough estimate)
	- (x, y, z) = (0, 0, 0)
- SLAM estimate (green)
	- (x, y, z) = 0.040364; 0.030199; 0.0
	- (qx, qy, qz, qw) = 0; 0; 0.81067; 0.5855
- Odometry-only estimate (blue)
	- (x,y,z) = 0.010667; 0.29087; 0.0
	- (qx, qy, qz, qw) = 0; 0; 0.72989; 0.68356

**Final SLAM position error (estimated)**: 0.05m
**Final Odometry-only position error (estimated)**: 0.29m

**SLAM Pipeline**

- **Files:** [src/slam-cwoodhayes/nuslam/config/slam_config.yaml](src/slam-cwoodhayes/nuslam/config/slam_config.yaml) and [src/slam-cwoodhayes/nuslam/launch/slam.launch.xml](src/slam-cwoodhayes/nuslam/launch/slam.launch.xml)

- **Overview:** Sensors (LIDAR / range) -> Detector (cluster + geometric tests) -> Data association (Mahalanobis gating) -> DDSLAM node (EKF predict/update, landmark management) -> Map & TF -> RViz visualization. Odometry feeds the motion-prediction step.

```mermaid
flowchart TD
	subgraph Robot
		A[Sensors: Lidar / Rangefinder]
		B[Wheel odometry]
	end

	A -->|observations| Detect[Detector: cluster & feature tests]
	B -->|pose predict| Prop[Motion Model / EKF predict]

	Detect -->|measurements| Assoc[Data association (Mahalanobis)]
	Prop --> Assoc

	Assoc --> DDSLAM[DDSLAM SLAM Node]
	DDSLAM --> Map[Landmark Management & Map]
	DDSLAM --> TF[pose estimate -> TF]
	Map -->|visualize| RViz[RViz2]

	style DDSLAM fill:#f9f,stroke:#333,stroke-width:2px
	style Detect fill:#ff9,stroke:#333
	style Assoc fill:#9ff,stroke:#333
	style Prop fill:#9f9,stroke:#333

	%% Annotations from config/launch
	subgraph Config
		C1[slam_config.yaml: slam_Q, slam_R,
			mahalanobis thresholds,
			detector params]
		C2[slam.launch.xml: launches nuslam_node,
			rviz2, static transform]
	end
	C1 -.-> DDSLAM
	C2 -.-> DDSLAM
	C2 -.-> RViz
```

- **Key configuration parameters (from `slam_config.yaml`):**
	- **`slam_Q` / `slam_R`:** process and measurement covariances for EKF predict/update.
	- **`slam_n_max_landmarks`:** maximum number of landmarks tracked.
	- **`slam_new_landmark_variance`:** initial variance for newly created landmarks.
	- **`slam_mahalanobis_association_threshold`:** gating threshold for association.
	- **`slam_mahalanobis_provisional_observation_count`:** observations required before confirming a landmark.
	- **`detector` params:** distance and RMSE thresholds, inscribed-angle limits, minimum cluster size, concavity threshold (controls landmark detection quality).

- **Launch behavior (from `slam.launch.xml`):**
	- Launches `nuslam` node with parameters loaded from the config files and remaps `joint_states` for the simulated robot.
	- Starts `rviz2` with `nuslam` RViz configuration when `use_rviz=true`.
	- Publishes a static transform between `nusim/world` and `map` for visualization alignment.
