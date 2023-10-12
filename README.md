# robot_localization
Project by Dexter Friis-Hecht & Ben Tarr. Source code written by Paul Ruvolo

# Overview

The project is focused on an implementation of a particle-filter algorithm, which is used to localize a robot within a known environment. This is done in a couple of steps:
- A hypothetical position for the robot is initialized within the mapped area.
- "Particles" are distributed around this position. Each particle has its own pose, similarily to the robot.
- As the robot navigates the environment, its odometry data is used to update each particle's position.
- After each position update, laser scan readings from the robot are transfered onto each particle, and measured distances from obstacles are compared between the particles and robot.
- Particles with laser readings close to the robot readings are assigned a weight, with a greater weight the closer the particles are.
- Particles with a low weight are redistributed on the high weight particles.
- These new distributes are used to contionusly update the robot's hypothetical position.
- Over time, as the robot moves about the environment more, the particles converge on the robot's true position, resulting in the robots position within the environment becomming known.

This report will go over each of these steps, detailing approach, actions the code performs, and limitations.

## Position and Particle Initialization


## Particle Position Update

Particle positions are updated with the **update_particles_with_odom():** function.
The function first samples the current pose of the robot. A pose contains x and y position, as well as rotation in radians (represented as theta). Both the current robot pose, as well as the
previous robot pose, are then stored in transformation matrices with the following format:

$$
\begin{bmatrix}
cos(\theta_0) & -sin(\theta_0) & x_0\\
sin(\theta_0) & cos(\theta_0) & y_0\\
0 & 0 & 1
\end{bmatrix}
$$

$$
\begin{bmatrix}
cos(\theta_1) & -sin(\theta_1) & x_1\\
sin(\theta_1) & cos(\theta_1) & y_1\\
0 & 0 & 1
\end{bmatrix}
$$

where the subscript 0 represents the previous position, and subscript 1 represents the new position. We can then find the difference, or delta, between these two transformation matrices with the following equation:

$$
\begin{bmatrix}
cos(\theta_0) & -sin(\theta_0) & x_0\\
sin(\theta_0) & cos(\theta_0) & y_0\\
0 & 0 & 1
\end{bmatrix}^- * 
\begin{bmatrix}
cos(\theta_1) & -sin(\theta_1) & x_1\\
sin(\theta_1) & cos(\theta_1) & y_1\\
0 & 0 & 1
\end{bmatrix}
$$

This delta matrix can be used in conjunction with each particle's own transformation matrix to update each particle's position relative to updates in the robot's odometry position.
In the equation below $\theta_p$, $x_p$, and $y_p$ represent the particles pose:

$$
\begin{bmatrix}
cos(\theta_p) & -sin(\theta_p) & x_p\\
sin(\theta_p) & cos(\theta_p) & y_p\\
0 & 0 & 1
\end{bmatrix} *
\begin{bmatrix}
cos(\theta_0) & -sin(\theta_0) & x_0\\
sin(\theta_0) & cos(\theta_0) & y_0\\
0 & 0 & 1
\end{bmatrix}^- * 
\begin{bmatrix}
cos(\theta_1) & -sin(\theta_1) & x_1\\
sin(\theta_1) & cos(\theta_1) & y_1\\
0 & 0 & 1
\end{bmatrix} *
\begin{bmatrix}
cos(\theta_p) & -sin(\theta_p) & x_p\\
sin(\theta_p) & cos(\theta_p) & y_p\\
0 & 0 & 1
\end{bmatrix}^- *
\begin{bmatrix}
x_p\\
y_p\\
1
\end{bmatrix}
$$

The updated particle position vector is then applied to each particles x and y class variable.
