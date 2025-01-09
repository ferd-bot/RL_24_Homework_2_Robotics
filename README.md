# RL24_HW_2
Ferdinando Dionisio, Vittorio Lisi, Giovanni Gabriele Imbimbo, Emanuele Cifelli

## **Overview** 
The goal of this homework is to develop a ROS package to dynamically control a 7-degrees-of-freedom
robotic manipulator arm into the Gazebo environment.

---

## **Instructions**  

1. Download the repository from GitHub:  
   ```bash
   cd src
   git clone -b REV_3 https://github.com/ferd-bot/RL_24_Homewrok_2_Robotics.git .
   ```
**Warn**
the git clone reported (with the point) works only if the directory in which it is executed is empty; 
if this is not the case, download the repository normally (without a point) and manually move the files out of the "RL_24_Homework_1_Robotics" folder, always remaining inside the src.

2. To configure and build all the packages in the workspace:  
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build
   source install/setup.bash
   ```

---

## Launching Packages

### 1. Launch the `iiwa` model with RViz:
To launch the IIWA model with RViz and the default command interface set to "position":
```bash
ros2 launch iiwa_bringup iiwa.launch.py
```

### 2. Launch the KDL node:
To launch the KDL node with the default command interface set to "position":
```bash
ros2 run ros2_kdl_package ros2_kdl_node
```

### 3. Use velocity commands:
To use velocity commands:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

### 4. Use effort commands (with Gazebo):
To use effort commands with Gazebo simulation:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:="true"
```

---

## Planner Selection

You can use the following commands to select and start a specific trajectory. Open another terminal and use these commands to set the planner:

### Available Planners:

- **Planner 0 (Trapezoidal Linear):**
  ```bash
  ros2 param set /ros2_kdl_node current_planner_index 0
  ```

- **Planner 1 (Cubic Linear):**
  ```bash
  ros2 param set /ros2_kdl_node current_planner_index 1
  ```

- **Planner 2 (Trapezoidal Circular):**
  ```bash
  ros2 param set /ros2_kdl_node current_planner_index 2
  ```

- **Planner 3 (Cubic Circular):**
  ```bash
  ros2 param set /ros2_kdl_node current_planner_index 3
  ```

- **Planner 4 (Home):**
  ```bash
  ros2 param set /ros2_kdl_node current_planner_index 4
  ```

**Note**: The possible sequence of trajectories is the selected trajectory followed by `home`. To start a new trajectory, you need to restart the KDL node.

---

## Topic Monitoring

You can monitor trajectory or torque errors using the following commands:

- **Trajectory error:**
  ```bash
  ros2 topic echo /trajectory_error
  ```

- **Torque error:**
  ```bash
  ros2 topic echo /torque_error
  ```

--- 

With this configuration, you can launch and test trajectories, monitor data, and configure the robot for different operational modes.
