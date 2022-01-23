# Robosim

3D robot arm simulator made with C++ and OpenGL. Can simulate any robot arm with rotational joints when given its Denavit-Hartenberg parameters. The robot is rendered using programmatically generated meshes that adapt their dimensions to the robot's link lengths. Lighting is implemented using Phong shading.

Features:
* Live control of joint angles
* Live editing of the Denavit-Hartenberg parameters of the robot
* Shows TCP world position using forward kinematics
* Shows linear and angular velocity of the TCP using the Jacobian matrix
* Numerical inverse kinematics to any point given by right click of the mouse


![demo](https://github.com/samilauronen/robosim/blob/main/doc/demo.gif)

Libraries used:
* OpenGL for 3D rendering
* GLFW for window management
* ImGUI for user interface
* Eigen for linear algebra
