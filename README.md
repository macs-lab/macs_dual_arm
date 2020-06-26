# MACS Dual-Arm Robots ROS Driver

This driver enables controlling the dual-arm robots in a ROS environment and in Gazebo. 
- [x] Control single arm using Rviz with MoveIt! plugin
- [x] Control both arm using Rviz with MoveIt! plugin
- [X] Dual-arm visulization and simulation in Gazebo with robot cell included
- [ ] Control the Hand-E grippers on both arms
- [ ] Enable velocity control of both arms
- [ ] Enable force control of both arms

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development.

### Prerequisites

Install Universal_Robots_ROS_Driver. Follow instructions provided [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).
You only need to build the driver packages. The UR robots in the lab already have remote control URCap installed. 
The calibration files are stored inside the ```/etc``` folder of this repo.

You must have your computer connected to both control boxes of the robots. 
By default, we have configered the IP of the left arm to be ```192.168.0.2``` and the right arm to be ```192.168.0.3```.
The IP of your computer must be configered as ```192.168.0.xxx``` to communicate with the robots.
The submask of you IP configuration should be ```255.255.255.0```.
To test if you have built connection, try to ping the robot ip:
```
$ ping 192.168.0.2
```
If the connection is correct, you should somthing like
```
PING 192.168.0.2 (192.168.0.2) 56(84) bytes of data.
64 bytes from 192.168.0.2: icmp_seq=1 ttl=64 time=0.085 ms
64 bytes from 192.168.0.2: icmp_seq=2 ttl=64 time=0.166 ms
```
Try to ping the other robot ip to make sure both robot are connected.

Finally, clone and build this repo.

### Control single arm using Rviz with MoveIt! plugin (left arm as example)
- Power up the robot. Open and run the program ```external_control.urp``` on the robot.
- Run the ur_robot_driver
```
$ roslaunch macs_dual_arm left_arm_bringup.launch
```
- Run the moveit ros move group
```
$ roslaunch macs_dual_arm single_arm_moveit_planning_execution.launch
```
- Start Rviz
```
roslaunch macs_dual_arm single_arm_moveit_rviz.launch
```

### Control both arms using Rviz with MoveIt! plugin
- Power up the robot. Open and run the program ```external_control.urp``` on both robots.
- Run the ur_robot_driver
```
$ roslaunch macs_dual_arm both_arm_bringup.launch
```
- Run the moveit ros move group
```
$ roslaunch macs_dual_arm both_arm_moveit_planning_execution.launch
```
- Start Rviz
```
roslaunch macs_dual_arm both_arm_moveit_rviz.launch
```

### Visualize both arms with robot cell in Gazebo

- Clone this repository in your catkin workspace
- Change the address to the mesh files in `macs_dual_arm/sdf/robot_cell.sdf`
- Execute `catkin_make` in your catkin workspace
- Run `roslaunch macs_dual_arm ur5e_macs_new.launch limited:=true` to bring up the two UR5e arms attached to the robot cell in Gazebo

## Todo

- [ ] Add `moveIt!` groups for each manipulator so that they can be controlled separately in `RViz`.

## Authors

- **Hui Xiao** - *Initial work, driver setup* - [xiaohuits](https://github.com/xiaohuits)
- **Nishant Elkunchwar** - *Add both arms, design and add robot cell visual and collision geometries to gazebo world* - [thecountoftuscany](https://github.com/thecountoftuscany)

## License

This repository is only for internal usage for now.

## Acknowledgments


