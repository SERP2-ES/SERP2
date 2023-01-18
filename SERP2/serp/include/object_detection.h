#ifndef SRC_OBJECT_DETECTION_H
#define SRC_OBJECT_DETECTION_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <raspicam/raspicam_cv.h>
#include <iostream>
#include "serp/ObjectDetection.h"

uint8_t levelDanger(float n);
void sortDetectedObjects(std::string pos, cv::Mat img, std::vector<cv::Point2f> centers, std::vector<float> radius, std::vector<cv::Rect> rectangles);
bool inRadious(cv::Point2f centerOrigin, cv::Point2f center2, int r);
void detectObj(cv::Mat img, int lowThreshold, int threshMult, int kernel_size, std::string pos);
std::vector<uint8_t> frameProcessing(cv::Mat frame);
std::string find_sensorIMG_path(char sensor , int valor_sensor);
cv::Mat place_sensor(cv::Mat frame, cv::Mat sensor, int position_x, int position_y);
cv::Mat process_frame(cv::Mat frame, std::vector<uint8_t> sensor_valores);


#endif