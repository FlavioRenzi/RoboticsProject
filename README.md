# ROS ~ Robotics course's first project
Politecnico di Milano - Accademic Year 2021-2022

>### Team
>* [__Elia Maggioni__](https://github.com/Eliaxie)
>* [__Flavio Renzi__](https://github.com/FlavioRenzi)
>* [__Jaskaran Sing__]()

The whole project was implemented utilizing `ROS Noetic` on a `Linux Ubuntu 20.04` machine.

## Goals
- Compute odometry using appropriate kinematics:
    - Compute robot linear and angular velocities v, ⍵ from wheel encoders.
    - Compute odometry using both Euler and Runge-Kutta integration.
    - ROS parameter for initial pose.
    - Calibrate (fine-tune) robot parameters to match ground truth.
- Compute wheel control speeds from v, ⍵.
- Add a service to reset the odometry to a specified pose (x,y,θ).
- Use dynamic reconfigure to select between integration method.

##
> To see the complete requirements -> [Project presentation](Project1.pdf)


## ToDo
- [ ] create launch file with parameter
- [ ] compute real wheels speed from wheels position
    - [ ] custom message to pubblish the speed `WheelSpeed`  on topic `real_wheel_rpm`
- [ ] compute kinematic to obtain the speed of the robot
    - [ ] publish speed on topic `cmd_vel` with a message of type `geometry_msgs/TwistStamped`
- [ ] compute integration to obtain the position of the robot
    - [ ] enumeration for the 2 integration methods
    - [ ] add ROS parameter for initial position
    - [ ] publish speed on topic `odom` with a message of type `nav_msgs/Odometry`
    - [ ] Broadcast TF `odom->base_link`
- [ ] compute inverse kineamtic from the speed of the robot to obtain the speed of the wheels
    - [ ] custom message to publish the speed `WheelSpeed` on topic `wheels_rpm` 
- [ ] tune parameter
- [ ] create a service to reset the position to a given one


