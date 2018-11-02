# Date 09-06-2018
# Author Ties Klappe

## Prerequisites
Installation of LibFranka https://frankaemika.github.io/docs/installation.html

```bash
sudo apt remove "*libfranka*"
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

To prevent having to specify the path to libfranka each time you build you can set a permanent [environment variable](https://unix.stackexchange.com/questions/21598/how-do-i-set-a-user-environment-variable-permanently-not-session) in your bash.

## Build with CATKIN

```bash
cd <your-catkin-workspace>/src
git clone <this-repo-url> .
cd ..
catkin_make -DFranka_DIR:PATH=</path_to_franka_installation/build>
```

## Run

```bash
source devel/setup.bash
rosrun franka_low_level_driver franka_low_level_driver <robot_hostname>
```

## Client test

```bash
source devel/setup.bash
rosrun franka_low_level_driver franka_driver_test_program
```

## EmergencyStop
To execute the emergency stop from commandline (with a sourced setup.bash) while executing a FrankaConfigurationAction:
```bash
rosservice call /franka_emergency_stop true
```

## Unlock (automatic error recovery)
To execute the unlock from commandline (with a sourced setup.bash) after executing an Emergency Stop or
getting a Franka Control error:
```bash
rosservice call /unlock_franka true
```

## Discontinuity errors
If you get Franka discontinuity errors when trying to execute a configuration action you can check the [Franka Emika troubleshooting page](https://github.com/frankaemika/docs/blob/master/source/troubleshooting.rst)