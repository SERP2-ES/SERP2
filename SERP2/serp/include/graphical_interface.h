#include <gtk/gtk.h>
#include <stdlib.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <serp/Matrix.h>

// ROS custom services
#include <serp/VelocitySetPoint.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <unistd.h>

#define IMAGE_WIDTH 462

// Global Variables
GtkStyleContext *context;
GtkWidget *window;
GtkWidget *widget_robot_state;
GtkTextView *log_mensagens;
GtkTextBuffer *log_buffer;
GtkTextIter *log_text_iter;
GtkTextIter* log_text_iter_start;
GtkTextIter* log_text_iter_end;
GtkWidget *button_manual_go;
GtkWidget *button_manual_stop;
GtkWidget *button_global_stop;
GtkRange *range_motor_left;
GtkRange *range_motor_right;
GtkLabel  *label_battery;
GtkLabel *label_robot_state;
GtkImage *camera_image_frame;
GtkWidget *dialog_image;

GMutex mutex_camera;
GMutex mutex_camera_detections;

ros::ServiceClient client_velocity_setpoint;
ros::ServiceClient client_battery_level;

//dont need this
//ros::ServiceClient client_read_programming_sheet;

image_transport::ImageTransport *it;
image_transport::Publisher captured_frame;
ros::Publisher pub_robot_state;
ros::Publisher vel_matrix;

//dont need this
//image_transport::Publisher captured_frame;

enum RobotState {
    Stopped,
    ManualControl,
    ReadingProgrammingSheet,
    Executing
};

struct Robot {
    int8_t motor_left_velocity;
    int8_t motor_right_velocity;
    int8_t motor_left_requested_velocity;
    int8_t motor_right_requested_velocity;
    RobotState state;
    int8_t battery_level;
} robot;

cv::Mat frame;
cv::Mat aux_frame1;
cv::Mat aux_frame2;
cv::Mat current_frame;
cv::Mat last_detected_sheet;
cv::Mat current_detected_sheet;
bool dialog_destroy_is_first_time = true;
uint8_t robot_state_num_chars;
int actual_window_width;
int actual_window_height;
int last_detected_image_last_width = 0;
int last_detected_image_last_height = 0;
int dialog_last_width = 0;
int dialog_last_height = 0;
bool is_startup = true;
