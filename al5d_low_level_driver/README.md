# Date 23-05-2018
# Author Ties Klappe

## Prerequisites
There are no prerequisites to build the al5d_low_level_driver package.

## Build with CATKIN

```bash
cd <your-catkin-workspace>/src
git clone <this-repo-url> .
cd ..
catkin_make
```

## Run

```bash
source devel/setup.bash
rosrun al5d_low_level_driver al5d_low_level_driver <serial-port-path>
```

## Client test
To test the driver with an Al5dConfigurationAction Open a new terminal in the same directory and execute the following commands:
```bash
source devel/setup.bash
rosrun al5d_low_level_driver al5d_driver_test_program
```

## EmergencyStop
To execute the emergency stop from commandline (with a sourced setup.bash) while executing an Al5dConfigurationAction:
```bash
rosservice call /al5d_emergency_stop true
```