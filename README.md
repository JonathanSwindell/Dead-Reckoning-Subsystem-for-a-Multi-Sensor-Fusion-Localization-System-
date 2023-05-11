# Dead Reckoning
This dead reckoning system is one part of a multi-sensor fusion GPS-denied localization system implemented at the University of Alabama in Huntsville as a Senior Design Project in 2023.


## Arduino

The updated arduino code is placed in /deadreck

All of the deadreckoning calculations take place on the arduino itself, and if the character 'r' is sent over serial to it the arduino, the arduino resets the deadreckoning.

## ROS

This project uses ROS in order for the sensors to communicate. All of the ROS code is placed inside of /ros_workspace

In order to start logging, navigate to /ros_workspace and then run the command 
```
source devel/setup.sh
```

That command sets up environment variables so ROS commands are recognized in the command line. Next, run this command:

```
roslaunch navigation navigation.launch
```
This launches all of the project's nodes. There are other launch files placed at [launch files](ros_workspace/src/navigation/launch/) 

All of the nodes (python script files) are placed [here](ros_workspace/src/navigation/scripts/)

If a new script file is to be a added, the project's [CMakeLists.txt](ros_workspace/src/navigation/CMakeLists.txt?plain=1#L113) will need to be edited. 

We have also added a way to manually reset the deadreckoning. To do so, in a sperate console, run the following command:
```
rosrun navigation reset_dr.py
```

The bluetooth node is prone to errors on startup. Some helpful links are placed at [ros_links.txt](ros_links.txt). That file also contains some useful information for installing ROS.
