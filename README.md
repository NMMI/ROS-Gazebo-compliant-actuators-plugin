# ROS-Gazebo-compliant-actuators-plugin
ROS-Gazebo toolbox for simulating different ASRs, easily and reliabily.
Different compliant-actuated joints can be simulated and additional ones can be implemented, exploiting the toolbox.

## How to use the VIACTORS plugins
There are currently four different compliant actuator models implemented, taken from the [VIACTORS](https://viactors.org/) EU Project:
- *SEA*, compliant joint with a fixed but customizable stiffness value (see [Pratt G.A. and Williamson M.](http://www.cs.cmu.edu/~cga/legs/jh1c.pdf));
- *qbMove*, compliant joint based on agonistic-antagonistic principle with varible stiffness, presented in [Catalano M. et. al.](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5980457), developed by [qbRobotics](https://qbrobotics.com/);
- *BAVS*, compliant joint based on agonistic-antagonistic principle with varible stiffness, presented in [Petit F. et. al.](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6824187), developed at [DLR](https://www.dlr.de/rm/en/desktopdefault.aspx/tabid-11666/8995_read-16713/);;
- *AwAS_II*, compliant joint based on pivot modulation to vary the stiffness, presented in [Jafari A. et. al.](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5979994).

You can find the relative plugin implementation inside the folder  *viactors_plugins/src/*.

### Plugin insertion
To use these plugins within your Gazebo robot, you just have to insert the following strings in your robot's URDF, for each joint (*your_joint*) that you want to transform in a compliant-actuated joint.
```xml
<!-- Custom plugin inseriton -->
<gazebo>
    <plugin name="compliant_act" filename="PLUGIN_NAME">
        <joint>your_joint</joint>
        <operation_mode>5</operation_mode>
        <pub_eltau>true</pub_eltau>
        <pub_state>true</pub_state>
        <sub_ext_tau>false</sub_ext_tau>
    </plugin>
</gazebo>
```
where the *PLUGIN_NAME* can change according to the four different (or your custom one!) compliant actuators models available, i.e. *libqbmove_plugin.so, libsea_plugin.so, libbavs_plugin.so, libawas_ii_plugin.so*.

The required and optional tags that you should insert are detailed in the relative files inside the *viactors_plugins/urdf_templates/* folder. 
Some of these tags will depend upon the type of compliant actuator used (they can be also customized by the user within the *InitParams* virtual function). (see also the referenced [article](https://www.frontiersin.org/articles/10.3389/frobt.2021.713083/full?&field=&journalName=Frontiers_in_Robotics_and_AI&id=713083))

### Usage example
A simple example of usage is given inside the folder *template_description/* that can be executed with 
```launch
roslaunch template_description template_gazebo.launch
```

## How to implement custom plugin
To implement another compliant actuator plugin, according to the [article](https://www.frontiersin.org/articles/10.3389/frobt.2021.713083/full?&field=&journalName=Frontiers_in_Robotics_and_AI&id=713083), you have to implement the code of the virtual functions for your compliant actuation model.
A template example is given in the file named *template_plugin.cpp* inside the folder  *viactors_plugins/src/*.

**Make sure** to also change the CMakeLists as follows
```cmake
## Declare a C++ library
add_library(template_plugin 
  src/template_plugin.cpp )
target_link_libraries(template_plugin 
  ${catkin_LIBRARIES}  
  ${GAZEBO_LIBRARIES} )
```
where the *template_plugin* name should be changed according to your compliant actuator type (e.g., *myNEWvsa_plugin*).

## Citation
The results were published in "Frontiers in Robotics and AI (Sim2Real Research Topic)" journal. [(paper link)](https://www.frontiersin.org/articles/10.3389/frobt.2021.713083/full?&field=&journalName=Frontiers_in_Robotics_and_AI&id=713083)

If you use this toolbox in your work and you want to cite it, please use the following.
```
@article{mengacciopen,
  title={An Open-Source ROS-Gazebo Toolbox for Simulating Robots with Compliant Actuators},
  author={Mengacci, Riccardo and Zambella, Grazia and Grioli, Giorgio and Caporale, Danilo and Catalano, Manuel and Bicchi, Antonio},
  journal={Frontiers in Robotics and AI},
  pages={246},
  publisher={Frontiers}
}
```

Also feel free to upload other compliant actuators model to expand the library.
Thank you! :D



