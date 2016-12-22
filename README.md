# ROS-simulator

ROS-based simulator for subCULTron project (www.subcultron.eu). 
It provides simulation of multiple aMussel, aFish and aPad agents. UWSim (http://www.irs.uji.es/uwsim/) is used for visualization.

## Prerequisites

The code has been developed and tested on an Ubuntu 14.04 system with ROS Indigo installed. The following libraries are required to build the code:
<!--and the developer docs:-->

 * TODO
<!-- * doxygen -->

On Ubuntu 14.04 the abovementioned prerequisites can be installed with the following commands:
```
sudo apt-get ...
```

A similar procedure should work on most modern Linux systems. 

## Quickstart

### Setting up the workspace

For the project to work properly, it needs to be a part of a catkin workspace. To initialize new workspace follow the commands:
```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```

Of course, `catkin_ws` is arbitrarily chosen name, you can use a name which describes the created workspace more closely.

### Building the code

After installing the prerequisites and initializing the workspace, `cd` to the project folder and build the code:
```
cd catkin_ws
catkin_make
```

If everything finishes without errors, try running the simulator:
```
roslaunch subcultron_sim simulator.launch
```

### Building and viewing the docs

TODO
<!--From the project root (where `Doxyfile` is located), run:

```
doxygen
firefox doc/html/index.html &
```
-->

## Detailed description
 
TODO..
