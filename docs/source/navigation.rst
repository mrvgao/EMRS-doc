Autonomous Navigation
======================

The **Patrolling Trash-Collecting Robot** utilizes advanced autonomous navigation capabilities enabled by ROS2 Nav2 to systematically patrol designated areas while effectively avoiding obstacles and dynamically adapting to changes in its environment.


Navigation Stack (Nav2)
------------------------

The navigation subsystem is implemented using ROS2's Navigation 2 (Nav2) stack, providing the robot with autonomous navigation capabilities through robust path planning, obstacle avoidance, and dynamic replanning.

Key components of Nav2 used in this project include:

- **Global Planner**: Determines optimal paths from the robot's current location to its patrol waypoints.
- **Local Planner (DWB Controller)**: Handles real-time obstacle avoidance and local trajectory adjustments.
- **Behavior Tree-Based Control**: Enables robust decision-making and fault recovery in various navigation scenarios.


Patrolling Strategy
--------------------

The robot patrols predefined areas by:

1. Establishing patrol waypoints within a mapped environment.
2. Utilizing global planning for optimal path determination to cover all specified waypoints efficiently.
3. Employing local planning to dynamically react to obstacles detected by onboard sensors, ensuring smooth, collision-free motion.


Obstacle Avoidance
-------------------

Real-time obstacle avoidance is achieved using:

- **LiDAR Sensor Data**: Continuously monitors surroundings for static and dynamic obstacles.
- **Costmaps**: Maintains and updates an occupancy grid map representing obstacles and free space.
- **Dynamic Window Approach (DWA)**: Calculates velocity commands to maneuver around detected obstacles safely.


Localization
-------------

Accurate localization is maintained through:

- **RTAB-Map**: Generates and maintains a detailed 3D map for precise robot localization.
- **Adaptive Monte Carlo Localization (AMCL)**: Assists in localization refinement during navigation tasks.

This integrated navigation solution ensures the robot can autonomously patrol complex environments effectively and reliably.

Executive Packages
------------