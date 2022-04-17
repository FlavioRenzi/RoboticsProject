# ROS ~ Robotics course's first project
Politecnico di Milano - Accademic Year 2021-2022

>### Team
>* [__Elia Maggioni__](https://github.com/Eliaxie)
>* [__Flavio Renzi__](https://github.com/FlavioRenzi)
>* [__Jaskaran Singh__](https://github.com/zJaska)

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


## Diagrams
![BlockDiagram](./BlockDiagram.drawio.svg)

## ToDo
- [x] create launch file with parameter
- [x] compute real wheels speed from wheels position
    - [x] custom message to pubblish the speed `WheelSpeed`  on topic `real_wheel_rpm`
- [x] compute kinematic to obtain the speed of the robot
    - [x] publish speed on topic `cmd_vel` with a message of type `geometry_msgs/TwistStamped`
- [x] compute integration to obtain the position of the robot
    - [x] enumeration for the 2 integration methods
    - [x] add ROS parameter for initial position
    - [x] publish speed on topic `odom` with a message of type `nav_msgs/Odometry`
    - [x] Broadcast TF `odom->base_link`
- [ ] compute inverse kineamtic from the speed of the robot to obtain the speed of the wheels
    - [x] custom message to publish the speed `WheelSpeed` on topic `wheels_rpm` 
- [ ] tune parameter
- [x] create a service to reset the position to a given one


