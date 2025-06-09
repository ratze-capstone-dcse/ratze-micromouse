# Ratada - Micromouse Simulation

This repository contains two main workspaces for the Ratada micromouse simulation project.

---

## gz_ws - Gazebo Workspace

The `gz_ws` workspace contains Gazebo plugins for the Ratada simulation.

### Components

Source files are located in `src` and are installed to `install`.

- **`encoder_sensor`** - Implementation of an incremental encoder for Gazebo as a custom sensor
- **`encoder_sensor_system`** - System that manages the encoder instances
- **`micromouse`** - GUI plugin for setting up and interacting with the simulation
- **`micromouse_system`** - Underlying system that processes commands sent via the GUI plugin and keeps track of lap times

### Setup

Make the scripts executable:

Go to the gz_ws folder

```sh
cd gz_ws
```

then

```sh
chmod +x buildws.sh
chmod +x installws.sh
chmod +x cleanws.sh
```

Build and install the plugins:

```sh
./buildws.sh . && ./installws.sh .
```

Configure Gazebo plugin paths (You can put it in .bashrc):

```sh
export GZ_GUI_PLUGIN_PATH=~/gz_ws/install/gui
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/gz_ws/install/system
```

For Wayland users ([reference](https://gazebosim.org/docs/harmonic/troubleshooting/#wayland-issues)):

```sh
export QT_QPA_PLATFORM=xcb 
```

**💡 Tip:** Add the export commands above to `~/.bashrc` to run them automatically on terminal startup.

---

## ros2_ws - ROS2 Workspace

This workspace contains ROS2-related assets and packages for the micromouse simulation.

### Setup and Build

Navigate to the ROS2 workspace:

```sh
cd ros2_ws
```

Build all packages:

```sh
colcon build
```

Source the workspace:

```sh
source install/setup.bash
```

Launch the simulation:

```sh
ros2 launch micromouse micromouse_launch.py
```

### Configuration

In the Gazebo window, you'll need to load the following files:

- **Maze map:** `mazefiles/halfsize`
- **Robot URDF:** `ros2_ws/src/micromouse/urdf/micromouse_robot.urdf`

---
