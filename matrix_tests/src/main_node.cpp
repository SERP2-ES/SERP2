#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <serp/Matrix.h>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "main_node");

  ros::NodeHandle node;

  //IMAGEM
  cv::Mat img = cv::imread("/home/percmap/Documents/ES/SERP2/matrix_tests/resources/01.jpg");

  if (img.empty())
  {
      std::cout << "!!! Failed imread(): image not found" << std::endl;
      return 0;
  }

  image_transport::ImageTransport it(node);
  image_transport::Publisher img_pub = it.advertise("/image", 1);

  sensor_msgs::ImagePtr msg;
  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  ros::Duration(10).sleep();
  msg->header.stamp = ros::Time::now();

  ROS_INFO("Published image\n");
  img_pub.publish(msg);

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