This is a ros2 package for the practices of the subject Mobile Robotics and Navigation (MRN) of the Master in Automation and Robotics (MUAR) of the Universitat Politècnica de Catalunya (UPC).

These practices are performed on Turtlebot4 lite platforms simulated and real.

# 1. Dependencies

For these practices you will need a PC with Ubuntu 22.04 and ros humble installed (ros-humble-desktop-full recommended).

Apart from ROS humble, the following dependencies are required for the practices. They all can be installed via apt:
- ssh
- git
- python3-colcon-common-extensions
- ignition-fortress
- ros-humble-rplidar-ros 
- ros-humble-rqt-tf-tree
- ros-humble-teleop-twist-keyboard
- ros-humble-turtlebot4-simulator 
- ros-humble-turtlebot4-desktop 
- ros-humble-turtlebot4-navigation 
- ros-humble-turtlebot4-node 
- ros-humble-turtlebot4-tutorials

## 1.1. Setup script

For the students convenience, we provide an script to install the dependencies, configure a ros workspace (~/ros2_ws) and clone this package, in your personal laptop (Ubuntu 22.04 and ros humble required).
This script needs to be executed once. 

First, download the script:

```bash
wget --no-check-certificate -O mrn_setup.sh http://bit.ly/upc_mrn_laptop
```

Terminal output should say it downloaded the file successfully, otherwise, try again. 
Then, execute the script (your password will be required):

```bash
source mrn_setup.sh
```

Check that no errors occurred and then you can remove the script and the logs that were generated (rm mrn_setup.sh log_mrn_setup.log).

# 2. Example

## 2.1. Compile `upc_mrn`

To check that everything is working you need first to compile the upc_mrn package:

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-up-to upc_mrn
```

## 2.2. Simulation of turtlebot4 lite

And then, launch the simulation (gazebo ignition) of the Turtlebot4 lite (rgbd camera is not simulated for efficiency) in a room-like environment:

```bash
ros2 launch upc_mrn sim.launch.py world:=rooms gui:=false
```

**`sim.launch.py` arguments:**

- `gui`: Whether launch the gazebo GUI or not. (default: `true`)
- `world`: which world to load. Available: `empty`, `rooms`, `simple`, `large`, `small`. (default: `empty`)
- `x`, `y`, `yaw`: initial pose of the robot. (default: origin)
- `oak`: Whether a robot with the plugin of the OAK rbd camera or a robot without it. (default: `false`)

## 2.3. SLAM_toolbox

In this example we will run the `slam_toolbox` to create a map of the environment.
In another terminal, run the launch:

```bash
ros2 launch upc_mrn slam.launch.yaml
```

Then, the visualization (RVIZ2) in a third terminal:

```bash
rviz2 -d ~/ros2_ws/src/upc_mrn/rviz/slam.rviz
```

And finally, to move the robot, run the `teleop_twist_keyboard` node in a fourth terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 2.4 Navigation

Instead of the teleoperation, we can launch the navigation (nav2) with a launch also provided:

```bash
ros2 launch upc_mrn nav.launch.yaml
```