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
While positioned in the src folder, clone the simulator code by running:
```
git clone git@github.com:subCULTron-project/ROS-simulator.git
```
or
```
git clone https://github.com/subCULTron-project/ROS-simulator.git
```

In the simulator files, we already have an example user controller package set up. It is located in ```controllers/template/```.

To make your own controller package, we suggest copying the whole directory and renaming the package to match your own specific application. The only necassary modifications are in ```CMakeLists.txt``` and ```package.xml``` files. We suggest opening them in the text editor and running "find and replace" command for all occurences of the word template to your custom name.

Controller source is located in the ```src``` folder, where you are free to modify the existing template. Simulation configuration files are present in ```data``` folder.

The main config file, where the number of robots, configuration file for each robot type and controllers for robots are specified is in the root of the folder and is named ```simulation_config.xml```. To specify multiple simulation config files, the simplest way is to copy the existing file, rename it and make the needed modifications.

Robot positions and battery states are specified in specially formatted ```.txt``` files, for example ```battery.txt``` and ```positions.txt``` in the ```data``` folder.

There is also a specification file for each robot in ```apad```, ```afish``` and ```amussel``` folders, where soecifics of each robot are defined (used sensors and actuators and their parameters). To make a new specification file, simply follow the same procedure as for the rest of the templates (copy, modify). Make sure you link the newly created files in the main config file.

### Building the code

After installing the prerequisites and initializing the workspace, `cd` to the project folder and build the code:
```
cd catkin_ws
catkin_make
```
Next, you need to prepare the launch file with predefined simulation parameters (in the ```data``` folder of your experiment package. For that purpose, run:
```
rosrun subcultron_launch setup.py -c <path_to_specification_file>
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
