
# moveit-ur

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
- Finally, calculated command will be published in `servo_calcs_` instance([#127](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/servo_calcs.cpp#L127)) and subscribed by ur_robot_driver.
(std_msgs/Float64MultiArray message type for ros_control JointGroupVelocityController)



#### 2. [ur5e_moveit_planning_execution.launch](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/ur5e_moveit_config/launch/ur5e_moveit_planning_execution.launch "ur5e_moveit_planning_execution.launch") in `ur5e_moveit_config/launch/`
- [ur5e_moveit_config](https://github.com/dhlee04/-IROL-Moveit_UR/tree/main/ur5e_moveit_config "ur5e_moveit_config") package contains SRDF([ur5e.srdf](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/ur5e_moveit_config/config/ur5e.srdf "ur5e.srdf")), configuration files([config](https://github.com/dhlee04/-IROL-Moveit_UR/tree/main/ur5e_moveit_config/config "config")), launch files([launch](https://github.com/dhlee04/-IROL-Moveit_UR/tree/main/ur5e_moveit_config/launch "launch")) generated from robot's URDF model.
- This launch file loads config files and robot's srdf model.
- These files are needed for interface with `MoveIt`(move_group)


#### 3. [ur5e_bringup.launch](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/ur_robot_driver/launch/ur5e_bringup.launch "ur5e_bringup.launch") in `ur_robot_driver/launch/`
- This launch file establishes connection with robot's control box

## Target frame(e.g. EEF)??? ?????? Twist ????????? ???????????? Joint angular velocity ??????
- ?????? Pose tracking ????????? ?????????, `moveit_servo`???????????????  `servo_calcs_` ??????????????? ?????????.
- ?????? ??????, `ServoCalcs` ???????????? `start` ???????????? ??????([#305](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/servo.cpp#L305))??????, ????????? ????????? ?????? ???????????? `mainCalcLoop` ???????????? ??????([#203](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L203))?????? ????????? ?????? while loop([#232](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L232))??? main loop?????? ?????????.
- ?????? ????????? `calculateSingleIteration` ?????????([#248](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L248), [#268](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L268))?????? ???????????? `cartesianServoCalcs` ?????????([#337](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L337), [#432](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L432)) ????????? ?????????.
- [#471](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L471)~#498 ?????? `planning_frame`(e.g. base_link)??? twist command ??? reference frame??? `robot_link_command_frame`(e.g. base_link, tool0)?????? TF??? ???????????? twist command??? reference frame??? `planning_frame` ??? ??????????????? ?????????. ???, commanding frame??? base frame??? ????????????.
- Eigen??? x-dimensional vector variable(VectorXd)??? `delta_x`??? ??? ????????? ?????? ????????? twist command ??????.
- MatrixXd??? `jacobian`??? moveit_core??? `getJacobian` ?????????([#1269](https://github.com/ros-planning/moveit/blob/a63580edd05b01d9480c333645036e5b2b222da9/moveit_core/robot_state/src/robot_state.cpp#L1269))??? ?????? jacobian matrix ??????.

- Eigen ????????? ??????(SVD) ????????? `JacobiSVD` ???????????? ??????  thin $U$??? thin ${V}$(reduced SVD) matrix, singular value?????? ?????????.[(#527)](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L527)
- ?????? singular value?????? `asDiagonal` ???????????? ?????? diagonal matrix??? $S$??? ????????????.[(#529)](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L529)
- $U,S,V$ ????????? ???????????????, ?????? ?????? ?????? ????????? ?????? pseudo inverse??? ?????? jacobian inverse??? ?????????.[(#530)](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L530)
 $$A = US{V^T},A^+ =  VS^+U^T$$

-  ?????? `delta_x`??? ????????? ??? ????????? ?????? ?????? jacobian inverse??? `pseudo_inverse`??? ?????? ????????? joint??? angular velocity??? ?????? ?????? `delta_theta_`??? ????????????.[(#532)](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L532)


## ????????????
[MoveIt Servo Inverse Kinematics Improvements, 22/07/22](https://moveit.ros.org/moveit/ros/2022/07/22/MoveIt-Servo-Inverse-Kinematics.html)
- ?????? jacobian pseudo inverse??? ????????? ????????? ??????????????? singularity??? ????????? ?????? ????????? ?????? ??????
- A primary focus of this research is _redundancy resolution_, in which robots with redundant degrees of freedom may utilize alternative joint configurations to achieve the same end-effector pose. By doing so, robots may be enabled to better avoid singularities, avoid collisions, and lower power consumption, to name a few benefits.
- MoveIT2 ????????? ?????? ??????
