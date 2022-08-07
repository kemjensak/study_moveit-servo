
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
- Finally, calculated command will be published in `servo_calcs_` instance([#127](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/servo_calcs.cpp#L127)) and subscribed by ur_robot_driver.
(std_msgs/Float64MultiArray message type for ros_control JointGroupVelocityController)



#### 2. [ur5e_moveit_planning_execution.launch](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/ur5e_moveit_config/launch/ur5e_moveit_planning_execution.launch "ur5e_moveit_planning_execution.launch") in `ur5e_moveit_config/launch/`
- [ur5e_moveit_config](https://github.com/dhlee04/-IROL-Moveit_UR/tree/main/ur5e_moveit_config "ur5e_moveit_config") package contains SRDF([ur5e.srdf](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/ur5e_moveit_config/config/ur5e.srdf "ur5e.srdf")), configuration files([config](https://github.com/dhlee04/-IROL-Moveit_UR/tree/main/ur5e_moveit_config/config "config")), launch files([launch](https://github.com/dhlee04/-IROL-Moveit_UR/tree/main/ur5e_moveit_config/launch "launch")) generated from robot's URDF model.
- This launch file loads config files and robot's srdf model.
- These files are needed for interface with `MoveIt`(move_group)


#### 3. [ur5e_bringup.launch](https://github.com/dhlee04/-IROL-Moveit_UR/blob/main/ur_robot_driver/launch/ur5e_bringup.launch "ur5e_bringup.launch") in `ur_robot_driver/launch/`
- This launch file establishes connection with robot's control box

## Target frame(e.g. EEF)에 대한 Twist 명령에 대응하는 Joint angular velocity 계산
- 상기 Pose tracking 설명에 따르면, `moveit_servo`최종적으로  `servo_calcs_` 인스턴스가 생성됨.
- 생성 이후, `ServoCalcs` 클래스의 `start` 메소드가 실행([#305](https://github.com/dhlee04/-IROL-Moveit_UR/blob/9c5bedec394015011f0bbec9fa5adcd47bba6e96/moveit_servo/src/servo.cpp#L305))되고, 메소드 내에서 동일 클래스의 `mainCalcLoop` 메소드를 호출([#203](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L203))하여 메소드 내의 while loop([#232](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L232))가 main loop로써 동작함.
- 실제 계산은 `calculateSingleIteration` 메소드([#248](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L248), [#268](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L268))에서 호출하는 `cartesianServoCalcs` 메소드([#337](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L337), [#432](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L432)) 내에서 이뤄짐.
- [#471](https://github.com/dhlee04/-IROL-Moveit_UR/blob/a53577f8032243a7d577fd3d02e264fd6b0b3265/moveit_servo/src/servo_calcs.cpp#L471)~#498 에서 `planning_frame`(e.g. base_link)과 twist command 의 reference frame인 `robot_link_command_frame`(e.g. base_link, tool0)간의 TF를 이용하여 twist command의 reference frame을 `planning_frame` 과 일치하도록 변환함. 즉, commanding frame과 base frame을 일치시킴.
- Eigen의 x-dimensional vector variable(VectorXd)인 `delta_x`에 위 과정을 통해 변환된 twist command 대입.
- MatrixXd인 `jacobian`에 moveit_core의 `getJacobian` 메소드([#1269](https://github.com/ros-planning/moveit/blob/a63580edd05b01d9480c333645036e5b2b222da9/moveit_core/robot_state/src/robot_state.cpp#L1269))를 통해 jacobian matrix 대입.

- Eigen 특이값 분해(SVD) 모듈의 `JacobiSVD` 클래스를 통해  thin $U$와 thin ${V}$(reduced SVD) matrix, singular value들을 구한다.
- 구한 singular value들은 `asDiagonal` 메소드를 통해 diagonal matrix인 $S$로 변환된다.
- $U,S,V$ 모두를 구했으므로, 아래 식과 같은 계산을 통해 pseudo inverse를 통한 jacobian inverse를 구한다.
 $$A = US{V^T},A^+ =  VS^+U^T$$

-  이제 `delta_x`의 좌측에 위 과정을 통해 얻은 jacobian inverse인 `pseudo_inverse`를 곱한 결과가 joint의 angular velocity가 되며 이를 `delta_theta_`에 대입한다.


## 참고자료
[MoveIt Servo Inverse Kinematics Improvements, 22/07/22](https://moveit.ros.org/moveit/ros/2022/07/22/MoveIt-Servo-Inverse-Kinematics.html)
- 상기 jacobian pseudo inverse를 이용한 방법의 문제점으로 singularity에 근접한 경우 멈추는 문제 지적
- A primary focus of this research is _redundancy resolution_, in which robots with redundant degrees of freedom may utilize alternative joint configurations to achieve the same end-effector pose. By doing so, robots may be enabled to better avoid singularities, avoid collisions, and lower power consumption, to name a few benefits.
