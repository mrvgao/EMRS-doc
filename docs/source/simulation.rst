Simulation Deliverables
=======================

This section lists the **simulation deliverables** of the Patrolling Trash-Collecting Robot project, organized by functionality. For system overview and environment setup, see the **Project Introduction**.


Simulated Functionality (`sim` branch)
----------------------------------------

Trash Collection in Simulation
___________________________________

.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/uyoueyiZ7CE?autoplay=1&mute=1" title="SLAM" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


- **Description**: Patrol, find the trash, and pick it up.
- **Launch Command**::


.. code-block:: bash

    git checkout sim   # for simulation

    # in terminal A
    ros2 launch turtlebot4_manipulator_ignition t4_manipulator_ignition.launch.py localization:=true nav2:=true slam:=false use_sim_time:=true use_sim:=true

    # in another terminal B
    ros2 run turtlebot4_trash_actions trash_collection_task.py --ros-args -r __ns:=/emrs


Ignition Gazebo Simulation - Localization and Navigation
______________________________________

.. code-block:: bash

    cd ~/rsp_ws/src/rsp-project-team-emrs
    git checkout sim   # for simulation

.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/jiAYWyJmKLg?autoplay=1&mute=1" title="SLAM" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


- **Description**: Physics-based simulation in custom apartment and Wyman lab worlds.
- **Launch Command**::

.. code-block:: bash

    ros2 launch turtlebot4_manipulator_ignition t4_manipulator_ignition.launch.py localization:=true nav2:=true slam:=false

- **Key Nodes**:

  - `ign_gazebo`: simulation server and physics engine
  - `ros_ign_bridge`: bridges ROS2 topics/services to Ignition
  - Nav2 nodes (`controller_server`, `planner_server`, `lifecycle_manager_navigation`)

Autonomous Patrolling In Simulation
___________________________________

.. raw:: html

    <iframe width="100%" height="450" src="https://www.youtube.com/embed/n4iy4K0s2rE?autoplay=1&mute=1" title="SLAM" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


- **Description**: Waypoint-based patrol using Nav2 behavior tree.
- **Parameters**: Defined in `config/patrol_waypoints.yaml`.
- **Launch Command**::

.. code-block:: bash

    ros2 launch turtlebot4_manipulator_navigation patrol.launch.py

- **Key Nodes**:

  - `patrol_robot_node`: reads waypoints and publishes goals
  - `bt_navigator`: executes behavior tree for navigation
  - `waypoint_follower`: follows sequential goals


