# -IROL-Moveit_UR

`moveit_servo` is copied from [moveit_servo](https://github.com/ros-planning/moveit/tree/master/moveit_ros/moveit_servo "moveit_servo")
`ur5e_moveit_config` is copied from [ur5e_moveit_config](https://github.com/fmauch/universal_robot/tree/calibration_devel/ur5e_moveit_config "ur5e_moveit_config")
`ur_robot_driver` is copied from [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_robot_driver "ur_robot_driver")

## Used execution(launch) files for pose tracking

#### 1. [pose_tracking_example.launch](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/moveit_servo/launch/pose_tracking_example.launch "pose_tracking_example.launch") in `moveit_servo/launch/`

- This launch file runs `pose_tracking_example` node in `moveit_servo`.
- Corresponding executable file is [pose_tracking_example.cpp(original)](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/moveit_servo/src/cpp_interface_example/pose_tracking_example.cpp "pose_tracking_example.cpp").
- pose_tracking_example.cpp creates pose tracker([#103](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/cpp_interface_example/pose_tracking_example.cpp#L103)) and publishes target pose([#106](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/cpp_interface_example/pose_tracking_example.cpp#L106), [#149](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/cpp_interface_example/pose_tracking_example.cpp#L149)).
- Created `tracker` instance of `PoseTracking` class creates `servo_` instance of `Servo` class([#67](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/pose_tracking.cpp#L67)), subscribes target pose([#71](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/pose_tracking.cpp#L71),[#239](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/pose_tracking.cpp#L239)) and publishes stamped target twist([#75](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/pose_tracking.cpp#L75)).
- And `servo_` instance creates `servo_calcs_` instance of `ServoCalcs` class([#79](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/servo.cpp#L79))
- `servo_calcs_` instance subscribes stamped target twist published by `tracker`([#101](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/servo_calcs.cpp#L101))
- Finally, calculated command published in `servo_calcs_` instance([#127](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/servo_calcs.cpp#L127)) and will be subscribed by ur_robot_driver.
(std_msgs/Float64MultiArray message type for ros_control JointGroupVelocityController)



#### 2. [ur5e_moveit_planning_execution.launch](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/ur5e_moveit_config/launch/ur5e_moveit_planning_execution.launch "ur5e_moveit_planning_execution.launch") in `ur5e_moveit_config/launch/`
- [ur5e_moveit_config](https://github.com/dhlee04/-IROL-Moveit_UR/tree/main/ur5e_moveit_config "ur5e_moveit_config") package contains SRDF([ur5e.srdf](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/ur5e_moveit_config/config/ur5e.srdf "ur5e.srdf")), configuration files([config](https://github.com/dhlee04/-IROL-Moveit_UR/tree/main/ur5e_moveit_config/config "config")), launch files([launch](https://github.com/dhlee04/-IROL-Moveit_UR/tree/main/ur5e_moveit_config/launch "launch")) generated from robot's URDF model.
- This launch file loads config files and robot's srdf model.
- These files are needed for interface with `MoveIt`(move_group)


#### 3. [ur5e_bringup.launch](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/ur_robot_driver/launch/ur5e_bringup.launch "ur5e_bringup.launch") in `ur_robot_driver/launch/`
- This launch file establishes connection with robot's control box




