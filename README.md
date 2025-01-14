# **RL24_HW_2**
Ferdinando Dionisio, Vittorio Lisi, Giovanni Gabriele Imbimbo, Emanuele Cifelli

## **Overview** 
This homework involves developing a ROS package to dynamically control a 7-degrees-of-freedom robotic manipulator in the Gazebo simulation environment.

---

## **Instructions**

1. Clone the repository from GitHub:  
   ```bash
   cd src
   git clone -b REV_3 https://github.com/ferd-bot/RL_24_Homework_2_Robotics.git .
   ```
   **Warning**:  
   The above `git clone` command (with the dot) works only if the target directory is empty.  
   - If not, you can remove extra files using:
     ```bash
     rm -rf *
     ```
   - Alternatively, download the repository normally (without the dot) and manually move the files out of the `RL_24_Homework_2_Robotics` folder while remaining inside the `src` directory.

2. Configure and build all packages in the workspace:  
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build
   source install/setup.bash
   ```

---

## **Launching the Manipulator**

### 1. Control Modes
The new code supports controlling the manipulator in three modes:

- **Position Mode**:  
  Launch the `iiwa_arm_controller` (with RViz):  
  ```bash
  ros2 launch iiwa_bringup iiwa.launch.py command_interface:="position" robot_controller:="iiwa_arm_controller"
  ```

- **Velocity Mode**:  
  Launch the `velocity_controller` (with RViz):  
  ```bash
  ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
  ```

- **Effort Mode**:  
  Launch the `effort_controller` (with Gazebo simulation):  
  ```bash
  ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:="true"
  ```

### 2. Launching Trajectories
To execute trajectories, open a new terminal and run the following commands depending on the desired control mode:

- **Position Mode**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=position
  ```

- **Velocity Mode**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=velocity
  ```

- **Effort Mode with Operational Space**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=effort -p control_space:=operational_space
  ```

- **Effort Mode with Joint Space**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=effort -p control_space:=joint_space
  ```

---

## **Available Trajectories**

The following trajectories are supported, all with effort-based control in Operational Space.  
Trajectories are numbered from 0 to 3:

1. **Linear with Trapezoidal Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=effort -p control_space:=operational_space
   ```

2. **Linear with Cubic Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 1 --ros-args -p cmd_interface:=effort -p control_space:=operational_space
   ```

3. **Circular with Trapezoidal Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 2 --ros-args -p cmd_interface:=effort -p control_space:=operational_space
   ```

4. **Circular with Cubic Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 3 --ros-args -p cmd_interface:=effort -p control_space:=operational_space
   ```

To execute trajectories in **Joint Space**, simply set `control_space:=joint_space`.  
For **Position** or **Velocity Control**, modify the `cmd_interface` parameter as explained above.

---

## **Notes**
A new topic called `/torque_plot` has been defined in the code.  
This topic allows you to visualize the torques sent to the manipulator's joints.

You can monitor this topic in the terminal:
```bash
ros2 topic echo /torque_plot
```

Alternatively, use `rqt_plot` for graphical visualization:
```bash
rqt
```

Within `rqt`, set up the plugin for plotting and insert the data corresponding to the joints:
```
/torque_plot/data[0]
/torque_plot/data[1]
/torque_plot/data[2]
/torque_plot/data[3]
/torque_plot/data[4]
/torque_plot/data[5]
/torque_plot/data[6]
```

**Note**:  
The `/torque_plot` topic is activated within the `ros2_kdl` node.  
To visualize it, you must run the KDL node in **Effort Mode** at least once.

---

## **Videos**
For simplicity, only videos demonstrating **Effort-Based Control in Operational Space with Gazebo** are attached:
- [Operational Space](https://www.youtube.com/watch?v=XFmY7oIKG8c)
- [Joint Space](https://www.youtube.com/watch?v=Fi35H55Z6VA)
- [Velocity Controller: Trajectory Linear Trapezoidal](https://youtu.be/u4lxzMbrPqs)
