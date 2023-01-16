#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "main_node");

  ros::NodeHandle node;

  cv::Mat img = cv::imread("/home/percmap/Documents/ES/SERP2/matrix_tests/resources/1.jpg");

  if (img.empty())
  {
      std::cout << "!!! Failed imread(): image not found" << std::endl;
      return 0;
  }

  image_transport::ImageTransport it(node);
  image_transport::Publisher img_pub = it.advertise("/image", 3);

  sensor_msgs::ImagePtr msg;
  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  ros::Duration(2).sleep();
  msg->header.stamp = ros::Time::now();
  img_pub.publish(msg);

  ros::spin();
  return 0;
}