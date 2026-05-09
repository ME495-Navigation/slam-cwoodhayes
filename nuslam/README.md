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

## SLAM Pipeline Summary

As seen in the demo videos (both in simulation and IRL), this pipeline:
1. detects cylindrical obstacles
2. associates them to a consistent internal ID 
3. places them on an internal map (which is used to drive the association in step 2), and then
4. localizes the robot in this map.

It took a fair amount of good old fashioned fiddling, adding steps & tuning parameters, but as seen in the demo videos, the final algorithm performs very well in both simulation & deployment. In particular:
1. very rare for invalid landmarks to make it into the SLAM state
2. very rare for the landmark detector to miss real landmarks
3. EKF is tuned well enough to the respective landmark + odometry uncertainties to fuse the two into a very good estimate.

Here is how the pipeline works at a high level (diagram & text below):

```mermaid
flowchart TD
  LIDAR["LIDAR ranges"]
  Odom["Wheel odometry"]
  Discard([Discard])
  Map["Map / Pose estimate"]

  subgraph Detection["Landmark Detection (CylinderDetector)"]
    Cluster["1 - Cluster by proximity<br/>(distance_threshold)"]
    SizeFilter["2 - Min cluster size filter<br/>(min_cluster_size)"]
    CircleFit["3 - Algebraic circle fit<br/>(SVD + eigendecomposition)"]
    GeoFilter["4 - Geometric filters<br/>(radius range, rmse_threshold)"]
	ConcavityCheck["5 - Concavity check<br/>(concavity_threshold)"]
    AngleCheck["6 - Inscribed angle check<br/>(angle mean and stddev thresholds)"]
  end

  subgraph EKF["EKF-SLAM (DDSLAMMahalanobis)"]
    EKFPredict["EKF predict (slam_Q)"]
    Mahal["6 - Mahalanobis association<br/>(association_threshold, n_max_landmarks)"]
    Provisional["7 - Provisional gate<br/>(provisional_observation_count)"]
    EKFUpdate["8 - EKF update (slam_R)"]
  end

  LIDAR --> Cluster
  Cluster --> SizeFilter
  SizeFilter -->|too small| Discard
  SizeFilter --> CircleFit
  CircleFit --> GeoFilter
  GeoFilter -->|bad fit| Discard
  GeoFilter --> ConcavityCheck
  ConcavityCheck --> |convex| AngleCheck
  ConcavityCheck -->|concave| Discard
  AngleCheck -->|too far from 90deg| Discard
  AngleCheck -->|detected landmark| Mahal

  Odom --> EKFPredict
  EKFPredict --> Mahal
  Mahal --> Provisional
  Provisional -->|count not reached| Discard
  Provisional -->|confirmed| EKFUpdate
  EKFUpdate --> Map
```

1. **Cluster raw LIDAR points** — convert range readings to Cartesian, group consecutive points within `distance_threshold` (0.1m). A wrap-around check merges the last cluster with the first if they are close.
2. **Minimum size filter** — clusters with fewer than `min_cluster_size` (8) points are discarded.
3. **Algebraic circle fit** — fits a circle to each cluster via SVD + eigendecomposition (Taubin method). Produces a center, radius, and RMSE.
4. **Geometric filters** — rejects fits whose RMSE exceeds `rmse_threshold`, or whose radius falls outside the realistic cylinder range for the obstacles we're using (this second check is a bit of a cheat; the algorithm still performs pretty well without it, but it'd be nice if this worked for _any_ obstacles. However, I wanted to squeeze the best performance I could out of the real-world demo and so I put this in.)
5. **Inscribed angle check** — This includes 2 sub-checks, based on the property that inscribed angles of a perfect circle are always 90deg: (a) mean inscribed angle must fall within `inscribed_angle_mean_range_deg` (70, 140); (c) std dev of inscribed angles must be below `inscribed_angle_stddev_threshold_deg` (20).
6. **Concavity Check** -- I added this in response to the circle detector often detecting the (concave) corners of the arena wall as circles. Checks that interior points must be closer to the sensor than the endpoints of each clustered arc, controlled by `concavity_threshold`; 
7. **Mahalanobis data association** — for each surviving detected circle, compute Mahalanobis distance to every known landmark in the EKF state. This is a distance metric for distributions that's like a euclidean distance between means, but weighted by uncertainty. Best match below `slam_mahalanobis_association_threshold` associates with that landmark; otherwise a new landmark is created (up to `slam_n_max_landmarks`).
8. **Provisional gate** — new landmarks must accumulate `slam_mahalanobis_provisional_observation_count` observations before their measurements feed into the EKF, preventing brief mistaken detections from corrupting the map.
9. **EKF update** — confirmed measurements update the EKF state using measurement covariance `slam_R`.
