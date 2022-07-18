# SCC Atlantis (ROS1 Noetic)
In this version of ROS1 we need to connect the SCC one robot at a time.

## 1. Communication
The robot and the SCC need to share the same network.

The SCC is the **client**, so we need to set the MASTER_URI to a specific robot's IP (the robot is the **server**). 

In the terminal, run:
1. `export ROS_MASTER_URI=http://ROBOT_IP:11311`
2. `export ROS_HOSTNAME=IP_OF_SCC`
3. `export ROS_IP=IP_OF_SCC`

## 2. Launch
To launch, we run the following: `roslaunch scc_atlantis_ros1 run_scc.launch`

This program launches: 

* Visualizers for both ASVs and UAVs so we can visualize robot information in real-time.
* UAV custom services to arm, land, change mode and takeoff:
  * `scc/raven/arm`
  * `scc/raven/set_mode`
  * `scc/raven/land`
  * `scc/raven/takeoff`
* ASV custom services to define a GoTo waypoint:
  * `scc/sense/skill_goto/start`



