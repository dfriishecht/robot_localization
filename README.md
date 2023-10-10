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

