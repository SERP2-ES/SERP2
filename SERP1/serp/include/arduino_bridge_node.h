#ifndef SRC_ARDUINO_BRIDGE_NODE_H
#define SRC_ARDUINO_BRIDGE_NODE_H

#include "ros/ros.h"
#include "serp/VelocitySetPoint.h"
#include "std_srvs/Trigger.h"
#include <std_msgs/String.h>
#include "serp/Matrix.h"
#include <serp/RobotInfo.h>
#include <serp/Velocity.h>
#include "logica.h"

// Global Variables
enum RobotState {
    Stopped,
    ManualControl,
    Executing
} robot_state;


struct Robot {
    int8_t motor_left_velocity;
    int8_t motor_right_velocity;
    int8_t battery_level;
} robot;


serp::RobotInfo robotget;
float matrixValores[100][100]={0};
int ligacoes[100][100]={0};



#endif //SRC_ARDUINO_BRIDGE_NODE_H
