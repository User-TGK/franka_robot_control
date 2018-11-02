# Date 04-06-2018
# Author Ties Klappe

## Prerequisites
There are no prerequisites to build the high_level_interface package.

## Build with CATKIN

```bash
cd <your-catkin-workspace>/src
git clone <this-repo-url> .
cd ..
catkin_make --pkg high_level_interface
```

## Run
Supported RobotInstances are: AL5D || FRANKA || FRANKA_SIMULATION || FRANKA_COMBINED *When both the simulation and the real robot are attached*

```bash
source devel/setup.bash
rosrun high_level_interface high_level_interface <RobotInstance>
```

Run with AL5D Driver attached
```bash
source devel/setup.bash
roslaunch high_level_interface al5d_driver.launch serial_port:=<path_to_serial_port>
```

Run with Franka Driver attached
```bash
source devel/setup.bash
roslaunch high_level_interface franka_driver.launch hostname:=<franka_control_ip>
```

Run with Franka Simulation attached
```bash
source devel/setup.bash
roslaunch high_level_interface franka_sim_driver.launch serial_port:=<path_to_serial_port>
```

Run with both Franka Driver and Franka Simulation attached
```bash
source devel/setup.bash
roslaunch high_level_interface franka_combined.launch hostname:=<franka_control_ip> serial_port:=<path_to_serial_port>
```

## Test (and example)
Make sure to read the pre conditions documented in the doxygen of the test function.

To test the High Level Interface its functionality, open a new terminal in the same directory and execute the following commands:
```bash
source devel/setup.bash
rosrun high_level_interface high_level_interface_integration_test
```

## Phython test
Because everybody gets angry when they have to build the repository here is a python script to test the HLI
```bash
source devel/setup.bash
rosrun high_level_interface ClientScript.py <DOF0Config> <DOF1Config> <DOF2Config> <DOF3Config> <DOF4Config> <DOF5Config> <DOF6Config>
```

## EmergencyStop
To execute the emergency stop from commandline (with a sourced setup.bash) while executing a ConfigurationAction:
```bash
rosservice call /emergency_stop true
```

## Unlock EmergencyStop
To execute an unlock emergency stop operation from commandline (with a sourced setup.bash) after calling the emergency stop:
```bash
rosservice call /unlock_robot_driver true
```

## Updating manual position
If the Franka robot is attached you can move it freely by holding the button on the side. To remember a manually set position as preprogrammed position MANUAL you can use a rosservice (with a sourced setup.bash) that will store the current configuration of the arm.
```bash
rosservice call /update_manual true
```

## Preprogrammed position
To move the robot to a Preprogrammed position from commandline (with a sourced setup.bash):
```bash
rosservice call /robot_position <position>
```
Replace position with a supported preprogrammed position. Currently the following positions are supported:
1. PARK
2. STRAIGHT
3. MANUAL *Has the same position as PARK if it's never updated*