# fantastic-ROS-Packages
汇总了一些自己经常使用的ROS Package

## [ROS device drivers](https://github.com/ros-drivers)
>包含了许多ros的驱动

- []()

## 驱动类

- [camera_umd](https://github.com/ros-drivers/camera_umd)
>包含了 uvc_camera包, a Video4Linux-based webcam driver for ROS，已经被弃用了，被libuvc_camera取代了
- [libuvc_camera](https://github.com/ros-drivers/libuvc_ros)
>libuvc_camera is a ROS driver that supports webcams and other UVC-standards-compliant video devices. It's a cross-platform replacement for uvc_camera, a Linux-only webcam driver. 使用参考 http://wiki.ros.org/libuvc_camera


- [virtual keyboard joystick](https://github.com/ethz-asl/rotors_simulator/wiki/Setup-virtual-keyboard-joystick)
使用键盘模拟 遥控的软件包


- [ps3joy 摇杆手柄驱动](http://wiki.ros.org/ps3joy)
>Playstation 3 SIXAXIS or DUAL SHOCK 3 joystick driver. Driver for the Sony PlayStation 3 SIXAXIS or DUAL SHOCK 3 joysticks. In its current state, this driver is not compatible with the use of other Bluetooth HID devices. The driver listens for a connection on the HID ports, starts the joystick streaming data, and passes the data to the Linux uinput device so that it shows up as a normal joystick.




## 通信类


- [mavros_offboard_control](https://github.com/raaslab/mavros_offboard_control)
>基于mavros包的一个上层应用接口，可以直接给它设定航点，也可以 使用上下左右前后的命令控制无人机

- [MAVLink Router](https://github.com/intel/mavlink-router)
>Route mavlink packets between endpoints. 

>The usual configuration is to have one "master" endpoint that is the flight stack (either on UART or UDP) and other components that can be on UDP or TCP endpoints. This is not strictly required and other configurations are possible: mavlink-router mainly routes mavlink packets from on endpoint without differentiating what they are.

## 算法类

- [ros gmapping](https://github.com/ros-perception/slam_gmapping)
>This package contains a ROS wrapper for OpenSlam's Gmapping. The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.

```bash
sudo apt-get install ros-kinetic-slam-gmapping
roscore
rosparam set use_sim_time true   ##启动包里的模拟时间
wget http://pr.willowgarage.com/data/gmapping/basic_localization_stage.bag ##测试的激光雷达数据集
rosrun gmapping slam_gmapping scan:=base_scan  ## 监听base_scan 话题
rosbag play --clock basic_localization_stage.bag ##发布话题
rosrun rviz rviz   ## 订阅map  /map话题

```


## SDK与ROS

- [PX4 Gazebo Simulation](http://dev.px4.io/en/simulation/gazebo.html)
>Pixhawk飞控在gazebo中的软件仿真和硬件仿真

- [RotorS a MAV gazebo simulator ](https://github.com/ethz-asl/rotors_simulator)
>There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the **VI-Sensor**, which can be mounted on the multirotor.

- [turtlebot_simulator](https://github.com/turtlebot/turtlebot_simulator)
>Launchers for Gazebo simulation of the TurtleBot

- [gazebo 模型下载](https://bitbucket.org/osrf/gazebo_models/src/9533d55593096e7ebdfb539e99d2bf9cb1bff347?at=default)



- DJI Onboard SDK (OSDK) && ROS SDK

>[DJI Onboard SDK (OSDK)](https://github.com/dji-sdk/Onboard-SDK)
>[DJI Onboard SDK ROS](https://github.com/dji-sdk/Onboard-SDK-ROS)
[DJI Onboard SDK Development](https://developer.dji.com/onboard-sdk/documentation/sample-doc/sample-setup.html#ros-oes)
>[DJI Onboard SDK ROS Development](https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/ROS/README.html)
>The DJI Onboard SDK allows you to connect your own Onboard Computer to a supported DJI vehicle or flight controller using a serial port (TTL UART). (Onboard Computer的形式可以时TX2/STM32 ……)




- [ROS Navigation ](https://github.com/ros-planning/navigation)
> A 2D navigation stack that takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base.

## 优秀的算法package

- [px4 Avoidance  ](https://github.com/PX4/avoidance)
>based-on px4 UAV Obstacle Detection and Avoidance


- [Autoware Open-Source To Self-Driving](https://github.com/CPFL/Autoware)
>Autoware is the world's first "all-in-one" open-source software for self-driving vehicles. The capabilities of Autoware are primarily well-suited for urban cities, but highways, freeways, mesomountaineous regions, and geofenced areas can be also covered. The code base of Autoware is protected by the BSD License. Please use it at your own discretion. For safe use, we provide a ROSBAG-based simulation environment for those who do not own real autonomous vehicles. If you plan to use Autoware with real autonomous vehicles, please formulate safety measures and assessment of risk before field testing
> https://www.autoware.ai/
> https://blog.csdn.net/zhangrelay/article/details/71750863



-[hector_slam ](http://wiki.ros.org/hector_slam)
>hector_mapping The SLAM node.
>hector_geotiff Saving of map and robot trajectory to geotiff images files.
>hector_trajectory_server Saving of tf based trajectories.



## book

- http://rosrobots.com/
> Here are the few latest technologies that are discussed on this book
> Self Driving Car, VR, DeepLearning, Autonomous Mobile Robots, 3D Object recognition 

## ROS2.0

- [eProsima Fast RTPS Documentation](https://eprosima-fast-rtps.readthedocs.io/en/latest/)
>RTPS as a tool to implement DDS 