#ifndef LOGICA_H
#define LOGICA_H

///ROS include
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <ctime>

///ROS MSGs includes
#include <serp/Matrix.h> 
#include <serp/velocity.h>

#define SUM 0
#define PRODUCT 1
#define SIMETRIC 2
#define IF 3
#define LESS 4
#define GREATER 5
#define EQUAL 6
#define M_LEFT 7
#define M_RIGHT 8
#define S_LEFT 9
#define S_RIGHT 10
#define S_FRONT 11
#define S_BACK 12
#define DELAY 13
#define ZERO 14
#define ONE 15
#define TWO 16
#define THREE 17
#define FOUR 18
#define FIVE 19
#define SIX 20
#define SEVEN 21
#define EIGHT 22
#define NINE 23
#define COMMA 24
#define AND 25
#define OR 26
#define MUX 27
#define ELSE_IF 32
#define DOUBLE 33
#define T_LEFT 34
#define T_RIGHT 35

// structure to ArUco Blocks
struct blocks {
    int id;
    int class_id;
    string type;
    string name;
    float out_f;
    bool out_b;
    string constant;
    clock_t timer;
    bool time_done;
};

// stores adjacency list items
struct adjNode {
    int val;
    adjNode* next;
};

// structure to store edges
struct graphEdge {
    int start_ver; // start block id
    int end_ver; // end block id
    int output_id; // 1 or 2
    int input_id; // 1,2 or 3
};

ros::Publisher pub_vel;

int error[4];

//velocity to apply to the motors
float out_vel[2] = { 0,0 }; // 1st - left, 2nd - right

//velocity measured in the motors
float real_vel[2] = { 0,0 }; // 1st - left, 2nd - right

//sensors (order: left, right, front, back)
float sensors[4] = { 0,0,0,0 };

int left, right, front, back;

#endif
