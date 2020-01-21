# MACS Dual-Arm Robots ROS Driver

This driver enables controlling the dual-arm robots in an ROS environment. 
- [x] Control single arm using Rviz with MoveIt! plugin
- [ ] Control both arm using Rviz with MoveIt! plugin
- [ ] Dual-arm visulization and simulation with robot cell included
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

## Authors

* **Hui Xiao** - *Initial work, driver setup* - [xiaohuits](https://github.com/xiaohuits)

## License

This driver is only for internal usage for now.

## Acknowledgments


