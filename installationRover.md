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

### Click

```
$ pip install -U click
```

## Click
The software uses click to read necessary files in. These files are a map of the area, the coordinates to drive to and an output file that will contain the map after the rover's finished.

### The command line
The command line to run the rover is as follows.

```
$ rosrun robot AutonomeNavigatieMidas.py -m maze.txt -c coordinates.txt -o output.txt
```
The different elements of the command line are:
+ *rosrun* can be compared to the python command to execute a python file.
+ *robot* is the directory that contains the file to be executed.
+ *AutonomeNavigatieMidas.py* is the file to be executed.
+ *-m maze.txt* to load the file of the map in. Note that the file should receive an absolute path. In this case, we are in the robot directory where the executable file resides and the 3 other text files mentioned in the command line.
+ *-c coordinates.txt* contains the coordinates of the rover to drive to.
+ *-o output.txt* contains the map of the area after the rover's finished with all experiments.

### The text files
#### maze.txt
The map file contains 84 characters by 40 characters. This means that the distance between each character corresponds with 10 cm. It is possible to alter the grid size but this will result in a longer calculation of the path to the destination. 

The default map is given below. the "." represents a part of the space where the rover can go. An obstacle on the map is marked by "#" characters. The default map does not include any obstacles.

```
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
```

#### coordinates.txt
As mentioned above, this map contains the coordinates for the rover to navigate to. Different coordinates are seperated by an enter. The x-coordinate always goes first and the x- and y-coordinate are seperated by a comman (","). Note that the coordinates should be written in meters. An example is given below.

```
1.2,2.4
3.3,5.6
```

#### output.txt
The output of the map is stored in for example output.txt.  You can use this map instead of the maze.txt for the next set of experiments. An example of the output is seen below.
```
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
.....................#######............
.....................#######............
.....................#######............
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
....#######.............................
....#######.............................
....#######.............................
....#######.............................
....#######.............................
....#######.............................
....#######.............................
....#######.............................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
........................................
```
