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

Your system is now able to download the necessary packages and main execution file.
First create a folder in the src folder of catkin_ws.

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg robot rospy
```

This folder will contain the main code to be executed. To create the package folder execute the lines below.

```
$ cd ~/catkin_ws/src
$ catkin_create_pkg common_tools rospy
```

Navigate inside the *common_tools* folder and inside the src folder. This can be done by doing the following:

```
$ cd ~/catkin_ws/src/common_tools/src
```

Make a directory in the src fould named *common_tools*

```
$ mkdir common_tools
```

The necessary packages can now be downloaded.

## Downloading the packages

Two packages that will be used for the rover should sit in the *common_tools* folder that was made earlier. The first package is the Marvelmind code and the other one is the IOPi code. 

### Marvelmin
For the localisation of the rover, Marvelmind beacons are used. To get the necessary data from the beacons a package from Marvelmind should be installed.
