Deliverables
============

This section lists the **completed deliverables** of the Patrolling Trash-Collecting Robot project, organized by functionality. For system overview and environment setup, see the **Project Introduction**.

Simulated Functionality (`sim` branch)
----------------------------------------

**Robot Description & Visualization**

.. image:: /images/placeholder.png
   :alt: Robot Description
   :align: center
   :scale: 50%

- **Description**: Unified URDF model combining TurtleBot4 base, OpenManipulator-X arm, 2D LiDAR, and Intel RealSense camera.
- **Launch Command**::

   .. code-block:: bash

      ros2 launch turtlebot4_manipulator_description \
        t4_manipulator_description.launch.py rviz:=true

- **Key Nodes**:
  - `robot_state_publisher`: publishes TF tree for robot links
  - `joint_state_publisher_gui`: interactive joint position control
  - `rviz2`: 3D visualization of robot and sensors

**Ignition Gazebo Simulation**

.. image:: /images/gazebo_gif.gif
   :alt: Gazebo Simulation
   :align: center
   :scale: 50%

- **Description**: Physics-based simulation in custom apartment and Wyman lab worlds.
- **Launch Command**::

   .. code-block:: bash

      ros2 launch turtlebot4_manipulator_ignition \
        t4_manipulator_ignition.launch.py localization:=true nav2:=true slam:=false

- **Key Nodes**:
  - `ign_gazebo`: simulation server and physics engine
  - `ros_ign_bridge`: bridges ROS2 topics/services to Ignition
  - Nav2 nodes (`controller_server`, `planner_server`, `lifecycle_manager_navigation`)

**Autonomous Patrolling**

.. image:: /images/simulation.gif
   :alt: Patrolling Behavior
   :align: center
   :scale: 50%

- **Description**: Waypoint-based patrol using Nav2 behavior tree.
- **Parameters**: Defined in `config/patrol_waypoints.yaml`.
- **Launch Command**::

   .. code-block:: bash

      ros2 launch turtlebot4_manipulator_navigation patrol.launch.py

- **Key Nodes**:
  - `patrol_robot_node`: reads waypoints and publishes goals
  - `bt_navigator`: executes behavior tree for navigation
  - `waypoint_follower`: follows sequential goals

**Hand-Eye Calibration**

.. image:: /images/eye-calibration.png
   :alt: Hand-Eye Calibration
   :align: center
   :scale: 50%

- **Description**: Calibrates end-effector to camera transform using MoveIt calibration pipeline and ArUco board.
- **Procedure**:
  1. Launch calibration pipeline via MoveIt.
  2. Capture marker poses and compute transform.
  3. Publish static TF with `static_transform_publisher`.

Real-World Deployment (`real` branch)
-------------------------------------

**Hardware Setup & Drivers**

.. image:: /images/physical-robot-whole.jpeg
   :alt: Physical Robot
   :align: center
   :scale: 50%

- **Description**: Integration on TurtleBot4 with OpenManipulator-X, LiDAR, RealSense.
- **Driver Nodes**:
  - `turtlebot4_node`: ROS2 driver for Create3 base
  - `open_manipulator_node`: controller for arm joints
  - Sensor drivers for LiDAR and RealSense

**Navigation & Patrolling**

.. image:: /images/real_t4.gif
   :alt: Real Patrol
   :align: center
   :scale: 50%

- **Description**: Executes identical patrol routine on hardware.
- **Launch Command**::

   .. code-block:: bash

      ros2 launch turtlebot4_manipulator_navigation patrol.launch.py

- **Key Nodes**:
  - Full Nav2 stack (`planner_server`, `controller_server`, `lifecycle_manager_navigation`)
  - `patrol_robot_node`
  - Hardware sensors publishing to ROS2 topics

**Calibration & Verification**

.. image:: /images/real_world_a.gif
   :alt: Calibration Verification
   :align: center
   :scale: 50%

- **Description**: Verifies hand-eye and mapping accuracy in physical environment.
- **Procedure**:
  - Run mapping in real environment (RTAB-Map integration).
  - Test grasp poses in MoveIt2 RViz for accuracy.

Common Setup
-------------

**Workspace Initialization**

.. image:: /images/placeholder.png
   :alt: Workspace Initialization
   :align: center
   :scale: 50%

.. code-block:: bash

   mkdir -p ~/rsp_ws/src && cd ~/rsp_ws/src
   git clone https://github.com/jhu-rsp/rsp-project-team-emrs
   cd ~/rsp_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install && source install/setup.bash

**Branch Selection**

.. image:: /images/placeholder.png
   :alt: Branch Selection
   :align: center
   :scale: 50%

.. code-block:: bash

   cd ~/rsp_ws/src/rsp-proj
   git checkout sim   # for simulation
   git checkout real  # for hardware deployment

All source packages and full launch configurations are maintained in the [`rsp-proj`](https://github.com/jhu-rsp/rsp-project-team-emrs) repository under the `sim` and `real` branches respectively. Refer to individual chapters (Mapping, Navigation, Manipulation) for deeper details on each subsystem.
