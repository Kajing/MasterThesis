# Installing the Rover
## Installation Robot Operating System on your linux distribution
### Linux distribution
To install ROS be sure to have a linux distribution on your Raspberry Pi.

A linux distribution can be found [here](https://ubuntu.com/download/raspberry-pi) or [here](https://ubuntu-mate.org/download/)

### Installing ROS

To install the Robot Operating System visit [this site](http://wiki.ros.org/ROS/Installation) and choose a suitable version for your linux distribution.

Then proceed to do the following

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

This will create a workspace for the rover.
Then proceed to execute the code below.

```
$ source devel/setup.bash
```

You can add this code in .bashrc file. To do this, execute the code below and add that line at the bottom of the file.

```
$ nano ~/.bashrc
```

Your system is now able to download the necessary libraries and main execution file.
First create a directory in the src directory of catkin_ws.

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg robot rospy
```

This directory will contain the main code to be executed. To create the library directory execute the lines below.

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg common_tools rospy
```

Navigate inside the *common_tools* directory and inside the src directory. This can be done by doing the following:

```
$ cd ~/catkin_ws/src/common_tools/src
```

Make a directory in the src fould named *common_tools*

```
$ mkdir common_tools
```

The necessary libraries can now be downloaded.

## Downloading the libraries

Two libraries that will be used for the rover should sit in the *common_tools* directory that was made earlier. The first library is the Marvelmind code and the other one is the IOPi code. 

### Marvelmind
For the localisation of the rover, Marvelmind beacons are used. To get the necessary data from the beacons a library from Marvelmind should be installed.
Navigate to the *common_tools* directory.

```
$ cd ~/catkin_ws/src/common_tools/src/common_tools
```
Download the Marvelmind file with the line below.
```
$ wget https://github.com/MarvelmindRobotics/marvelmind.py/blob/master/src/marvelmind.py
```

### IOPi
The IOPi pi plus pin extender uses a library, found on [this website](https://github.com/abelectronicsuk/ABElectronics_Python_Libraries/tree/master/IOPi) just like the Marvelmind library, we install it using wget.

```
$ cd ~/catkin_ws/src/common_tools/src/common_tools
$ wget https://github.com/abelectronicsuk/ABElectronics_Python_Libraries/blob/master/IOPi/IOPi.py
```

navigate to the catkin_ws and execute *catkin_make*

```
$ cd ~/catkin_ws
$ catkin_make
```

## Downloading the main file

To download the main fail, navigate to the *robot* directory.

```
$ cd ~/catkin_ws/src/robot
```

Make a new directory named *scripts*

```
$ mkdir scripts
$ cd scripts
```

Just like before we can download the main file using wget.

```
$ wget https://github.com/Kajing/MasterThesis/blob/main/AutonomeNavigatieMidas.py
```

## Installing necessary packages

The rover uses many packages, the following packages can be installed in the following manner.

### Numpy

```
$ pip install numpy
```

### Scipy

```
$ python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
```

### Filterpy

```
$ pip install filterpy
```
