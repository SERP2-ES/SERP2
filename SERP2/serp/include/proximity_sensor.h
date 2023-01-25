#ifndef SRC_PROXIMITY_DETECTOR_H
#define SRC_PROXIMITY_DETECTOR_H

#include <math.h>
#include <numeric>
#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <raspicam/raspicam_cv.h>
#include <serp/ObjectDetection.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>


/**
*  The following functions convert the pixel distance value to a metric distance value for the respective direction
*  Return value is in milimeters
*/
double convert_Left(double distance);    // -45º
double convert_Front(double distance);   // 0º
double convert_Right(double distance);   // 45º
double convert_Back(double distance);    // 180º

#endif
