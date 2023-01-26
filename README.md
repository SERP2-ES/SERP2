# SERP2

This repository contains the implementations made for SERP robot in Engineering Systems course. 

Folder organization:
 - **SERP1** - Implementation made by the first team in the 2021/2022 course edition. [(Original repo)](https://github.com/jorgef1299/SERP).
 - **SERP2** - Implementation made by the team of the 2022/2023 course edition.

 ## SERP2
 To run the program the `serp` and `rosserial` packages must be in the ROS workspace folder (In the given robots the workspace is  the `catkin_ws` folder, the packages must be directly inside `catkin_ws\src`).

In the `Arduino` folder, the `.ino` file is used to control the motors. The files for the needed dependencies are in `Arduino/libraries`.

The `setup.py` is a python script that allows to connect the raspberry to a new wi-fi via a bluetooth socket (using [Serial Bluetooth Terminal App](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal&hl=pt_PT&gl=US)).

The `serp` package was implemented by us and has the following structure:

![ROS node graph](SERP2/serp/extras/Node_diagram.png)

### Nodes
 - **GUI** - main node, controls the graphical interface (developed with gtk).
 - **Program detection** - converts the captured image in two matrices that represente the connections between blocks in graph form.
 - **Logic** - traduces the graph into velocities for each motor.
 - **Object detection** -  Converts the image aquired from the fisheye camera into distances to objects in 4 directions (front, left, right and rear).
 - **Arduino** - Implements the motor controllers, so that the robot achieves the desired velocities.

