#ifndef SRC_VIDEO_TESTS_H
#define SRC_VIDEO_TESTS_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <numeric>


// ---------- CAMERA CALIBRATION + UNDISTORTION ---------- (add to separate library)

// Defining the dimensions of checkerboard

extern int CHECKERBOARD[2]; // Number of corners (vertical, horizontal)

typedef struct {
    cv::Mat cameraMatrix, distCoeffs, R, T;
} camera_info;

extern camera_info cam_info;

extern const double PI;



// ---------- ARUCO IDENTIFICATION ---------- (add to separate library)

//X and Y coordinates of left corners of orientation arucos
 struct coordinates {
    int x;
    int y;
};

 struct links {
    coordinates point;
    int order;
    bool linked;
    int link_end;
};

//X and Y coordinates of 4 corners of block arucos
 struct block {
    int id;
    int count;
    int size_aruco;
    coordinates b_sup_left;
    coordinates b_sup_right;
    coordinates b_inf_left;
    coordinates b_inf_right;
    links input1;
    links input2;
    links outputs;
    links condition;
};

 struct combination{
    float number;
    int matrix_pos;
    int dest;
};

//global variables
extern int size_detect;
extern int pos_list;
extern int current_ids_size;

//paper validation
extern bool orientation_check ;
extern bool pictureValidated ;
extern bool vertical ;
extern std::vector<int> detections; // 40 values equal to 0
extern int count_stable_frames;
extern int count_total_arucos;

//combinations of Ks
extern int num_combinations ;

//sensor values
extern int sensor_value_se;
extern int sensor_value_sd;
extern int sensor_value_sf;
extern int sensor_value_st;

//dictionary 4X4
extern cv::Ptr<cv::aruco::Dictionary> dictionary;


 struct orientation_block {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
};



// ---------- LINE DETECTION ---------- (add to separate library)

extern std::vector<cv::Vec4i> masks;
extern std::vector<cv::Point2f> crossingPoints;





//function declaration
void camera_parameters(int type);
cv::Point2f findFisheye(int Xe, int Ye, double R, double Cfx, double Cfy, double He, double We);
cv::Mat correctImage(cv::Mat frame, int type);
orientation_block detect_orientation_blocks(std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids);
cv::Point2f extendLines(cv::Mat frame, cv::Point2f point1, cv::Point2f point2, int height, int width);
std::vector<cv::Point2f> calculateExtendedPoints(cv::Mat frame, std::vector<cv::Point2f> pts_src, int pos_id, int height, int width);
std::vector<cv::Point2f> calculateNewDimensions(cv::Mat frame, orientation_block markers);
cv::Mat perspective_correction(cv::Mat original, std::vector<cv::Point2f> new_points);
void validatePicture(std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids);
void draw_check_function(int id, int x, int y, cv::InputOutputArray image);
void draw_points(int id, coordinates sup_esq, coordinates sup_dir, coordinates inf_esq, coordinates inf_dir, cv::InputOutputArray image);
void draw_full_block(cv::InputOutputArray image_camera, int id, int pos, std::vector<std::vector<cv::Point2f>> corners, std::vector <block> block_i);
void drawing_functions(cv::InputOutputArray image, std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids, std::vector <block> block_i);
int check_occurences(int id, std::vector<block> vect);
int get_topmargin(int size_aruco) ;
int get_bottommargin(int size_aruco);
int get_topblock(int size_aruco);
std::vector <block> corners_blocks(int id, int pos, std::vector<std::vector<cv::Point2f>> corners, std::vector <block> block_i);
std::vector <block> save_in_out(coordinates sup_esq, coordinates sup_dir, coordinates inf_esq, coordinates inf_dir, int pos_list, int id, std::vector <block> block_i);
std::vector <block> saving_coordinates(std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids, std::vector <block> block_i);
std::string check_function(int id);
std::vector<block> put_arucos_order(std::vector<block> blocks );
void DebugBlocks(std::vector<block> block_in_order);
void Debugcombs(std::vector<combination> comb);
void Debugmatrixlinks(int matrix[100][100]);
void Debugmatrixvalues(float matrix[100][100]);
int position_matrix_input1(int id, int count);
int position_matrix_input2(int id,int count);
int position_matrix_output(int id, int count);
int position_matrix_condition(int id, int count);
bool isInside(int circle_x, int circle_y, int rad, int x, int y);
std::vector<block> check_lines(cv::Vec4i lin, size_t j, int radius, std::vector<block> vec);
std::vector<cv::Point2f> detectCrossings(cv::Mat image);
std::vector<cv::Vec4i> detectLines(cv::Mat paper);
std::vector<block> saveLines(std::vector<cv::Vec4i> linesP, std::vector<block> blocks);
coordinates findEndPoint(int begin, std::vector<block> blocks);
void drawLines(cv::InputOutputArray paper, std::vector<block> blocks);
void drawMatrixLinks(int m_links[100][100],std::vector<block> block,std::vector<combination> comb);
int get_k(int id);
void drawMatrixValues(float m_values[100][100], std::vector<block> block, std::vector<combination> comb);
std::vector<combination> getCombinations(std::vector<block> blocks, std::vector<combination> comb);
std::vector<combination> makeCombinations(std::vector<block> blocks, std::vector<combination> comb);
void detectAndInterpret_Lines(cv::Mat& frame, cv::Ptr<cv::aruco::Dictionary> dict, int m_links[100][100], float m_values[100][100], int& ready);
int detectAndInterpret_Paper(cv::Mat& frame, cv::Ptr<cv::aruco::Dictionary> dict, int m_links[100][100], float m_values[100][100], int ready);






#endif
