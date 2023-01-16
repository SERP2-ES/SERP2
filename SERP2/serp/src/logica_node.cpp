#include "logica_node.h"

void cbMatrix(const serp::Matrix::ConstPtr &msg){
  serp::Velocity vel;


  vel.vel_motor_left = 0;
  vel.vel_motor_right = 0;
  pub_vel.publish(vel);
  return;
}

void cbSensors(const serp::ObjectDetection::ConstPtr &msg){
    left = msg->left;
    right = msg->right;
    front = msg->front;
    back = msg->back;
    return;
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "logica_node");
  
  ros::NodeHandle node;
  
  ros::Subscriber matrix_sub = node.subscribe("/matrix", 1, cbMatrix);
  ros::Subscriber sensors_sub = node.subscribe("/sensors", 1, cbSensors);

  pub_vel = node.advertise<serp::Velocity>("/vel", 1);

  ros::spin();
  return 0;
}
