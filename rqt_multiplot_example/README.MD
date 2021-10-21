# RQT-MULTIPLOT Example
## Overview
This is a simple usage example of the package [rqt_multiplot](http://wiki.ros.org/rqt_multiplot).

Click on the link to quickly discover how to use this package.

This configuration file will display all the information contained in the motor_1_state and motor_2_state ROStopics.


#### How to run it

To be able to use this file you will need to install the rqt-multiplot package.
You can run the __apt-get install__ command : 

*sudo apt-get install ros-<ros-distro>-rqt-multiplot*

In alternative, you could install it from source  cloning the relative [git repository](https://github.com/ANYbotics/rqt_multiplot_plugin)


Then you will just need to launch the actuator example available in the template_description directory using this command:

## *roslaunch template_description template_gazebo.launch*

and then in a separate shell, you will only need to run the rqt_multiplot node.
Typically there could be two options, you either just type in a separate shell the command 
## *rqt_multiplot*
or alternatively 
## *rosrun rqt_multiplot rqt_multiplot*


