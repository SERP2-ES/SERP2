#include <ros/ros.h>

///ROS MSGs includes
#include <serp/Matrix.h>
#include <serp/Velocity.h>
#include <std_msgs/Int8.h>

ros::Publisher pub_vel, pub_errors;

struct graphEdge {
    int start_ver; // start block id
    int end_ver; // end block id
    int output_id; // 1 or 2
    int input_id; // 1,2 or 3
};

struct blocks {
    int id;
    int class_id;
    std::string type;
    std::string name;
    float out_f;
    bool out_b;
    std::string constant;
    clock_t timer;
    bool time_done;
};

bool matrix_rcv;

float out_vel[2];
int error[4];
int sensors[4];

std::vector <graphEdge> edges;
std::vector <blocks> blocks_list;
std::vector<int> tokens;
std::vector<int> init;
int num_block, num_rows, num_col;
