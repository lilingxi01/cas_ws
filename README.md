<img src="https://imagedelivery.net/Dr98IMl5gQ9tPkFM5JRcng/921d64c3-8673-48e5-e056-eb899cfda800/HD" alt="Cover"/>

**Link to presentation recording: https://t.ly/YCy4**

# Get Started

Before doing anything, you need to clone this repository to your local computer, and run following commands for installing all dependencies:

```bash
cd /path/to/cas_ws
pip install -r requirements.txt
```

If you don't want to install them all, alternatively, you can choose to manually install the following (assuming that you already have installed the `rospy`):

```bash
sudo apt-get install python3-opencv
pip install ultralytics
```

## Build this workspace

You need to build this workspace using Catkin tools.

```bash
cd /path/to/cas_ws
catkin build
```

## Source this workspace

For bash terminal as an example:

```bash
cd /path/to/cas_ws
source cas_setup.bash
source devel/setup.bash
```

## How to start the CAS evaluation with simulator

This launch file will start the CAS core node with CAS simulator and automatically start spawning balls.

```bash
roslaunch cas_core sim.launch
```

## How to start the simulator only

After building this workspace, you can launch the simulator by using the simulation launch file:

```bash
roslaunch cas_simulator map.launch
```

# Within Packages

1. `cas_core` - This is the core package of Collision Avoidance System. It has all the core components for running the system (including object detection model, Kalman Filter, frame alignment, and action lifecycle) and the simulation.
   * `__init__.py` - The core file of the package. It contains the main lifecycle of CAS and calls all other components accordingly.
   * `frame.py` - A sub-module for aligning frames (assigning balls) between different time steps.
   * `kalman.py` - A sub-module for Kalman Filter.
   * `sim_helper.py` - Helper functions for enabling simulation ball-shooting actions.
2. `cas_simulator` - This is the package of CAS Simulator that is used to simulate the evaluation environment of the system. It includes autonomous ball-shooting algorithm that samples the ball-shooting direction and position based on the future trajectory of the robot to provide a reliable evaluation experience.

# Additional Resources

* Frame Mapping Algorithm (Google Colab): https://t.ly/ZdXd
* Kalman Filter Module (Google Colab): https://t.ly/9cBbm
