#ifndef SRC_RPI_CAMERA_H
#define SRC_RPI_CAMERA_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/SetBool.h>
#include "serp/Sensors.h"
#include <raspicam/raspicam_cv.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include "objDetect.h"
#include <opencv2/aruco.hpp>
#include "video_tests.h"
#include "serp/Matrix.h"
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



// Global variables
enum state {
    NormalOperation,
    ReadProgrammingSheet,
    DetectObstacles
} camera_state;

#endif
