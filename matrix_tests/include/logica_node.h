#ifndef LOGICA_H
#define LOGICA_H

///ROS include
#include <ros/ros.h>

///ROS MSGs includes
#include <serp/Matrix.h> 
#include <serp/velocity.h>

ros::Publisher pub_vel;

int left, right, front, back;

#endif
