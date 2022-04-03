# ROS ~ Robotics course's first project
Politecnico di Milano - Accademic Year 2021-2022

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
