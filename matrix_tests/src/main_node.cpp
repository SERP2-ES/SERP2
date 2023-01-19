#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <serp/Matrix.h>

std::string path;

void getParam(ros::NodeHandle &node){
  node.getParam("/img_path", path);
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "main_node");

  ros::NodeHandle node;
  //getParam(node);

  //IMAGEM
  cv::Mat img = cv::imread("/home/percmap/Documents/ES/SERP2/matrix_tests/resources/img1_edge_quant.jpg");

  if (img.empty())
  {
      std::cout << "!!! Failed imread(): image not found" << std::endl;
      return 0;
  }

  image_transport::ImageTransport it(node);
  image_transport::Publisher img_pub = it.advertise("/image", 1);

  sensor_msgs::ImagePtr msg;
  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  std::cout << "10..." << std::endl;
  ros::Duration(1).sleep();
  std::cout << "9..." << std::endl;
  ros::Duration(1).sleep();
  std::cout << "8..." << std::endl;
  ros::Duration(1).sleep();
  std::cout << "7..." << std::endl;
  ros::Duration(1).sleep();
  std::cout << "6..." << std::endl;
  ros::Duration(1).sleep();
  std::cout << "5..." << std::endl;
  ros::Duration(1).sleep();
  std::cout << "4..." << std::endl;
  ros::Duration(1).sleep();
  std::cout << "3..." << std::endl;
  ros::Duration(1).sleep();
  std::cout << "2..." << std::endl;
  ros::Duration(1).sleep();
  std::cout << "1..." << std::endl;
  ros::Duration(1).sleep();
  
  msg->header.stamp = ros::Time::now();

  img_pub.publish(msg);
  ROS_INFO("Published image\n");

  //MATRIZ
/*   serp::Matrix mat;

  mat.manual_mode = true;
  mat.vel_motor_left = 30;
  mat.vel_motor_right = 2;

  ros::Publisher mat_pub = node.advertise<serp::Matrix>("/matrix", 1);
  ros::Duration(2).sleep();
  mat_pub.publish(mat);

  mat.vel_motor_left = 0;
  mat.vel_motor_right = 0;
  ros::Duration(5).sleep();
  mat_pub.publish(mat);

  ros::spin(); */
  return 0;
}