#include "video_tests.h"


int CHECKERBOARD[2]{5,8}; // Number of corners (vertical, horizontal)

camera_info cam_info;

const double PI = 3.141592653589793;



// ---------- ARUCO IDENTIFICATION ---------- (add to separate library)



//global variables
int size_detect=0;
int pos_list;
int current_ids_size=0;
// Create Link and Value Matrices
float matrix_values[100][100];
int matrix_links[100][100];

//paper validation
bool orientation_check = false;
bool pictureValidated = false;
bool vertical = false;
std::vector<int> detections(35, 0); // 40 values equal to 0
int count_stable_frames = 0;
int count_total_arucos = 0;

//combinations of Ks
int num_combinations=0;

//sensor values
int sensor_value_se;
int sensor_value_sd;
int sensor_value_sf;
int sensor_value_st;



//dictionary 4X4
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);



// ---------- LINE DETECTION ---------- (add to separate library)

std::vector<cv::Vec4i> masks;
std::vector<cv::Point2f> crossingPoints;

cv::Mat FinalFrame;






// ---------- CAMERA CALIBRATION + UNDISTORTION ---------- (add to separate library)

void camera_parameters(int type)
{
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgpoints;

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for(int i{0}; i<CHECKERBOARD[1]; i++)
    {
        for(int j{0}; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j,i,0));
    }


    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;

    // Path of the folder containing checkerboard images
    char *project_path = getenv("SERP_PROJECT_PATH");
    char checkerboard_file_name[25] = "include/imgset/*.jpg";
    char checkerboard_file_path[200];
    sprintf(checkerboard_file_path, "%s%s", project_path, checkerboard_file_name);

    cv::glob(checkerboard_file_path, images);

    cv::Mat frame, gray;
    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_pts;
    bool success;

    // Looping over all the images in the directory
    for(int i{0}; i<images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

        // Finding checker board cornersopen
        // If desired number of corners are found in the image then success = true
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

        /*
       * If desired number of corner are detected,
       * we refine the pixel coordinates and display them on the images of checker board
      */
        if(success)
        {
            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }

        //      cv::imshow("Image",frame);
        //      cv::waitKey(0);
    }

    //    ROS_WARN_STREAM("3D->"<<objpoints.size()<<" 2D->"<<imgpoints.size());

    cv::destroyAllWindows();

    /*
     * Performing camera calibration by passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the detected corners (imgpoints)
    */

    if(type==0) cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cam_info.cameraMatrix, cam_info.distCoeffs, cam_info.R, cam_info.T);
    else if(type==1) cv::fisheye::calibrate(objpoints, imgpoints, cv::Size(gray.rows,gray.cols), cam_info.cameraMatrix, cam_info.distCoeffs, cam_info.R, cam_info.T, cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_CHECK_COND |cv::fisheye::CALIB_FIX_SKEW, cv::TermCriteria(cv::TermCriteria::EPS|cv::TermCriteria::MAX_ITER, 30, 1e-6));

    //    ROS_WARN_STREAM("Camera Matrix");
    //    ROS_WARN_STREAM(cam_info.cameraMatrix);

    //    ROS_WARN_STREAM("Distortion Coefficients");
    //    ROS_WARN_STREAM(cam_info.distCoeffs);

    //    ROS_WARN_STREAM("Rotation Matrix");
    //    ROS_WARN_STREAM(cam_info.R);

    //    ROS_WARN_STREAM("Translation Matrix");
    //    ROS_WARN_STREAM(cam_info.T);
}


//Find the corresponding fisheye output point corresponding to an input cartesian point
cv::Point2f findFisheye(int Xe, int Ye, double R, double Cfx, double Cfy, double He, double We)
{
    cv::Point2f fisheyePoint;
    double theta, r, Xf, Yf; //Polar coordinates

    r = Ye/He*R;
    theta = Xe/We*2.0*PI;
    Xf = Cfx+r*sin(theta);
    Yf = Cfy+r*cos(theta);
    fisheyePoint.x = Xf;
    fisheyePoint.y = Yf;

    return fisheyePoint;
}


cv::Mat correctImage(cv::Mat frame, int type)
{
    // Undistort
    cv::Mat undist_img;

    if(type==0)
    {
        // Direct Undistort
        //    cv::undistort(img, undist_img, cam_info.cameraMatrix, cam_info.distCoeffs);
        //    cv::fisheye::undistortImage(img, undist_img, cam_info.cameraMatrix, cam_info.distCoeffs);

        // initUndistortRectifyMap With Balance=0.0
        cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);
        cv::Mat map1, map2;
        cv::fisheye::initUndistortRectifyMap(cam_info.cameraMatrix, cam_info.distCoeffs, E, cam_info.cameraMatrix, cv::Size(frame.cols, frame.rows), CV_16SC2, map1, map2);
        cv::remap(frame, undist_img, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        //        cv::imshow("Corrected Image", undist_img);
        //        cv::waitKey(0);
    }
    else if(type==1)
    {
        // initUndistortRectifyMap With Balance>0.0

        cv::Size dim1 = cv::Size(frame.cols, frame.rows); // Dimension of the original image
        cv::Size dim2 = dim1; // Dimension of the box you want to keep after un-distorting the image
        cv::Size dim3 = dim1; // Dimension of the final box where OpenCV will put the undistorted image

        // When balance = 0,OpenCV will keep the best part of the image for you
        // Whereas balance = 1 tells OpenCV to keep every single pixel of the original image
        double balance = 1.0;

        cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);

        cv::Mat new_K;
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cam_info.cameraMatrix, cam_info.distCoeffs, dim2, E, new_K, balance);

        cv::Mat map1, map2;
        cv::fisheye::initUndistortRectifyMap(cam_info.cameraMatrix, cam_info.distCoeffs, E, new_K, dim3, CV_16SC2, map1, map2);
        cv::remap(frame, undist_img, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }
    else if(type==2)
    {
        cv::Mat panoramicImage;

        int Hf, Wf, He, We;
        double R, Cfx, Cfy;

        Hf = frame.size().height;
        Wf = frame.size().width;
        R = Hf/2; //The fisheye image is a square containing a circle so the radius is half of the width or height size
        //        Cfx = Wf/2; //The fisheye image is a square so the center in x is located at half the distance of the width
        //        Cfy = Hf/2; //The fisheye image is a square so the center in y is located at half the distance of the height
        Cfx = cam_info.cameraMatrix.at<double>(0,2);
        Cfy = cam_info.cameraMatrix.at<double>(1,2);

        He = (int)R;
        We = (int)2*PI*R;

        panoramicImage.create(He, We, frame.type());

        for (int Xe = 0; Xe <panoramicImage.size().width; Xe++)
        {
            for (int Ye = 0; Ye <panoramicImage.size().height; Ye++)
            {
                panoramicImage.at<cv::Vec3b>(cv::Point(Xe, Ye)) =    frame.at<cv::Vec3b>(findFisheye(Xe, Ye, R, Cfx, Cfy, He, We));
            }
        }

        //        cv::imshow("Panoramic Image", panoramicImage);
        //        cv::waitKey(0);

        undist_img = panoramicImage;

        cv::imwrite("../catkin_ws/src/SERP/serp/include/tests/panorama.jpg", panoramicImage);
    }

    return undist_img;
}



// ---------- ARUCO IDENTIFICATION AND HOMOGRAPHY ---------- (add to separate library)


orientation_block detect_orientation_blocks(std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids)
{
    // Reset flag
    orientation_check=false;

    std::vector<std::vector<cv::Point2f>> orientation_corners;
    orientation_block markers;

    if(ids.size() > 0)
    {
        int corners_detected=0;

        //run through every detected aruco
        for (int i = 0; i < corners.size(); i++)
        {
            if(ids[i] == 28 || ids[i] == 29 || ids[i] == 30 || ids[i] == 31)
            {
                corners_detected++;

                markers.corners.push_back(corners[i]);
                markers.ids.push_back(ids[i]);
            }

            if(corners_detected == 4) orientation_check=true;
        }
    }

    return markers;
}


cv::Point2f extendLines(cv::Mat frame, cv::Point2f point1, cv::Point2f point2, int height, int width)
{
    double x1, y1, x2, y2;
    double m, b;

    x1 = point1.x;
    y1 = point1.y;

    x2 = point2.x;
    y2 = point2.y;

    // Calculate line equation
    m = (y1-y2) / (x1-x2);
    b = y1 - (x1 * m);

    //    ROS_WARN_STREAM("y1="<<y1<<" y2="<<y2);
    //    ROS_WARN_STREAM("x1="<<x1<<" x2="<<x2);
    //    ROS_WARN_STREAM("m="<<m<<" b="<<b);

    // Calculate New Point
    cv::Point2f new_point;

    new_point.x = point1.x - (0.3 * height); // extends point by a ratio
    new_point.y = m * new_point.x + b;
    cv::circle(frame, new_point, 5, cv::Scalar(0,255,255), cv::FILLED, 8, 0);
    cv::line(frame, new_point, point1, (0, 0, 255), 2);

    return new_point;
}


std::vector<cv::Point2f> calculateExtendedPoints(cv::Mat frame, std::vector<cv::Point2f> pts_src, int pos_id, int height, int width)
{
    cv::Point2f id_28 = pts_src[0];
    cv::Point2f id_29 = pts_src[1];
    cv::Point2f id_30 = pts_src[2];
    cv::Point2f id_31 = pts_src[3];

    std::vector<cv::Point2f> new_points;

    if(pos_id == 0) // Paper is vertically oriented and 28 is at the bottom left
    {
        // Draw Extended Lines
        cv::Point2f new_point1 = extendLines(frame, id_28, id_29, height, width);
        cv::Point2f new_point2 = extendLines(frame, id_31, id_30, height, width);
        cv::line(frame, new_point1, new_point2, (0, 0, 255), 2);

        // Substitute exterior detections by extensions
        id_28 = new_point1;
        id_31 = new_point2;
    }
    else if(pos_id == 1) // Paper is vertically oriented and 28 is at the top right
    {
        // Draw Extended Lines
        cv::Point2f new_point1 = extendLines(frame, id_29, id_28, height, width);
        cv::Point2f new_point2 = extendLines(frame, id_30, id_31, height, width);
        cv::line(frame, new_point1, new_point2, (0, 0, 255), 2);

        // Substitute exterior detections by extensions
        id_29 = new_point1;
        id_30 = new_point2;
    }

    // Save new points
    new_points.push_back(id_28);
    new_points.push_back(id_29);
    new_points.push_back(id_30);
    new_points.push_back(id_31);

    return new_points;
}


std::vector<cv::Point2f> calculateNewDimensions(cv::Mat frame, orientation_block markers)
{
    std::vector<cv::Point2f> pts_src;

    cv::Point2f id_28, id_29, id_30, id_31;

    //    ROS_WARN_STREAM("Detected orientation blocks:");

    for(int i=0; i<markers.ids.size(); i++)
    {
        // Save corners take keep the most info from the paper (the farthest ones)
        if(markers.ids[i]==28)
        {
            id_28 = cv::Point2f(markers.corners[i][0].x,markers.corners[i][0].y);
            cv::circle(frame, id_28, 5, cv::Scalar(255,255,255), cv::FILLED, 8, 0);
        }
        else if(markers.ids[i]==29)
        {
            id_29 = cv::Point2f(markers.corners[i][3].x,markers.corners[i][3].y);
            cv::circle(frame, id_29, 5, cv::Scalar(255,255,255), cv::FILLED, 8, 0);
        }
        else if(markers.ids[i]==30)
        {
            id_30 = cv::Point2f(markers.corners[i][2].x,markers.corners[i][2].y);
            cv::circle(frame, id_30, 5, cv::Scalar(255,255,255), cv::FILLED, 8, 0);
        }
        else if(markers.ids[i]==31)
        {
            id_31 = cv::Point2f(markers.corners[i][1].x,markers.corners[i][1].y);
            cv::circle(frame, id_31, 5, cv::Scalar(255,255,255), cv::FILLED, 8, 0);
        }
    }

    // Conect real points
    cv::line(frame, id_28, id_29, (255, 0, 0), 2);
    cv::line(frame, id_29, id_30, (255, 0, 0), 2);
    cv::line(frame, id_30, id_31, (255, 0, 0), 2);
    cv::line(frame, id_31, id_28, (255, 0, 0), 2);

    // Points in original frame
    pts_src.push_back(id_28);
    pts_src.push_back(id_29);
    pts_src.push_back(id_30);
    pts_src.push_back(id_31);

    std::vector<cv::Point2f> new_points;

    // Check orientation and calculate new frame dimensions
    int height, width;

    if(abs(id_28.y-id_29.y) < abs(id_28.x-id_29.x))
    {
        vertical = true;
        //        ROS_WARN_STREAM("Paper is vertically oriented");

        // Calculate original dimensions
        height = abs(id_29.x-id_28.x);
        width = abs(id_30.y-id_29.y);

        if(id_28.x < id_29.x)
        {
            //            ROS_WARN_STREAM("28 is at the bottom left\n");

            // Replace original points by extensions, to account for distortion
            new_points = calculateExtendedPoints(frame, pts_src, 0, height, width);
            id_28 = new_points[0];
            id_29 = new_points[1];
            id_30 = new_points[2];
            id_31 = new_points[3];
        }
        else
        {
            //            ROS_WARN_STREAM("28 is at the top right\n");

            // Replace original points by extensions, to account for distortion
            new_points = calculateExtendedPoints(frame, pts_src, 1, height, width);
            id_28 = new_points[0];
            id_29 = new_points[1];
            id_30 = new_points[2];
            id_31 = new_points[3];
        }
    }
    else
    {
        vertical = false;
        ROS_WARN_STREAM("Paper is horizontally oriented");
        ROS_WARN_STREAM("Rotate paper for detection\n");
    }

    return new_points;
}


cv::Mat perspective_correction(cv::Mat original, std::vector<cv::Point2f> new_points)
{
    std::vector<cv::Point2f> pts_dst;

    cv::Mat new_frame;

    int height, width;

    if(vertical)
    {
        // Calculate new dimensions
        height = abs(new_points[1].x-new_points[0].x);
        width = abs(new_points[2].y-new_points[1].y);

        //        ROS_WARN_STREAM("Height="<<height<<" Width="<<width<<"\n");

        // Points in new frame
        pts_dst.push_back(cv::Point2f(0, 0)); // Matches id_28
        pts_dst.push_back(cv::Point2f(0, height-1)); // Matches id_29
        pts_dst.push_back(cv::Point2f(width-1, height-1)); // Matches id_30
        pts_dst.push_back(cv::Point2f(width-1, 0)); // Matches id_31

        // Calculate Homography
        cv::Mat h = cv::findHomography(new_points, pts_dst);

        // Warp source image to destination based on homography
        cv::warpPerspective(original, new_frame, h, cv::Size(width, height));
    }

    return new_frame;
}


void validatePicture(std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids, cv::Mat frame)
{
    cv::Mat frameCopy = frame.clone();

    // Limit to discard points not in the front

    int x_limit = frameCopy.cols/2-150;

    //    // Vizualize x limit
    //    cv::Point limit(x_limit, frameCopy.rows/2);
    //    cv::circle(frameCopy, limit, 15, cv::Scalar(0,255,0), -1, 8, 0);
    //    cv::imshow("input", frameCopy);
    //    cv::waitKey(0);


    // Conditions to skip count

    if(!orientation_check) return;

    for(int i=0; i<ids.size(); i++)
    {
        // To account for detections with unwanted ids
        if(ids[i]<0 && ids[i] > 35)
        {
            ROS_WARN_STREAM("-------------- Invalid id="<<ids[i]);
            return;
        }

        // To account for detections not in front
        if(corners[i][0].x > x_limit || corners[i][1].x > x_limit || corners[i][2].x > x_limit || corners[i][3].x > x_limit)
        {
            ROS_WARN_STREAM("-------------- id="<<ids[i] << " NOT IN FRONT");
            return;
        }
    }


    // Picture Validation

    pictureValidated = false;

    sort(ids.begin(), ids.end());

    for(auto id = std::cbegin(ids); id != std::cend(ids); )
    {
        // Count detections
        int count = std::count(id, std::cend(ids), *id);
        //        ROS_WARN_STREAM("id=" << *id << " count="<< count);

        // Save number of detections
        if(detections[*id] < count) detections[*id] = count;

        for(auto last = *id; *++id == last; );
    }

    int total = std::accumulate(detections.begin(), detections.end(), 0);


    int n_stable = 5;


    if(count_total_arucos == total && count_stable_frames < n_stable)
    {
        // If total hasn't changed, increase count of stable frames
        count_stable_frames++;
        ROS_WARN_STREAM("Stable frame n=" << count_stable_frames);
    }
    else if(count_total_arucos < total)
    {
        ROS_WARN_STREAM("ArUco count=" << count_total_arucos << " VS detected="<< total << "\n");

        for(int i=0; i<ids.size(); i++)
        {
            ROS_WARN_STREAM(i+1 << ") Id=" << ids[i]);
        }

        count_total_arucos = total;
        count_stable_frames = 0;
    }


    // Signal Picture To Be Chosen

    if(count_stable_frames == n_stable)
    {
        // Get a frame where number of detections is equal to expected total
        if(ids.size() == count_total_arucos)
        {
            ROS_WARN_STREAM("Final total = " << count_total_arucos);

            ROS_WARN_STREAM("Detected " << ids.size() << " ArUcos");

            for(int i=0; i<ids.size(); i++)
            {
                ROS_WARN_STREAM(i+1 << ") Id=" << ids[i]);
            }

            pictureValidated = true;
            ROS_WARN_STREAM("Picture Validated\n");
        }
    }

}



// ---------- BLOCK FORMATION ---------- (add to separate library)


//check what function is associated with the detected aruco
void draw_check_function(int id, int x, int y, cv::InputOutputArray image) {
    if (id == 0) putText(image, "Sum", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 1) putText(image, "*", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 2) putText(image, "-x", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 3) putText(image, "If", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 4) putText(image, "<", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 5) putText(image, ">", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 6) putText(image, "=", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 7) putText(image, "Me", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 8) putText(image, "Md", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 9) putText(image, "Se", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 10) putText(image, "Sd", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 11) putText(image, "Sf", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 12) putText(image, "St", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 13) putText(image, "Temp", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 14) putText(image, "0", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 15) putText(image, "1", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 16) putText(image, "2", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 17) putText(image, "3", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 18) putText(image, "4", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 19) putText(image, "5", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 20) putText(image, "6", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 21) putText(image, "7", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 22) putText(image, "8", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 23) putText(image, "9", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 24) putText(image, ".", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 25) putText(image, "AND", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 26) putText(image, "OR", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
    else if (id == 27) putText(image, "mux", cv::Point(x, y), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(0, 0, 255), 1.5);
}


//draw inputs and outputs of blocks (to-do: mux-->condition circle not working)
void draw_points(int id, coordinates sup_esq, coordinates sup_dir, coordinates inf_esq, coordinates inf_dir, cv::InputOutputArray image)
{
    if (id == 0 || id == 1 || id == 3 || id == 4 || id == 5 || id == 6 || id == 13 || id == 25 || id == 26) {
        //2 inputs 1 output
        circle(image, cv::Point(sup_esq.x, sup_esq.y+((inf_esq.y-sup_esq.y)*(0.25))), 9, CV_RGB(0, 0, 255), 1.5); //input top
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.75))), 9, CV_RGB(0, 0, 255), 1.5);//input bottom
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
    }
    else if (id == 2 || id == 14 || id == 15 || id == 16 || id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22 || id == 23 || id == 24) {
        //1 input 1 output
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.5))), 9, CV_RGB(0, 0, 255), 1.5); //input top
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
    }
    else if (id == 7 || id == 8) {
        //1 input
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.5))), 9, CV_RGB(0, 0, 255), 1.5); //input top
    }
    else if (id == 9 || id == 10 || id == 11 || id == 12) {
        //1 output
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
    }
    else if (id == 27) {
        //2 inputs 1 output 1 condition
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.25))), 9, CV_RGB(0, 0, 255), 1.5); //input top
        circle(image, cv::Point(sup_esq.x, sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.75))), 9, CV_RGB(0, 0, 255), 1.5);//input bottom
        circle(image, cv::Point(sup_dir.x, sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5)), 9, CV_RGB(0, 0, 255), 1.5); //output
        circle(image, cv::Point(((inf_dir.x-inf_esq.x)*0.5)+inf_esq.x, inf_dir.y), 9, CV_RGB(0, 0, 255), 1.5); //condition
    }
}


// Draws block body
void draw_full_block(cv::InputOutputArray image_camera, int id, int pos, std::vector<std::vector<cv::Point2f>> corners, std::vector <block> block_i)
{
    line(image_camera, cv::Point(block_i[pos].b_sup_left.x, block_i[pos].b_sup_left.y), cv::Point(block_i[pos].b_sup_right.x, block_i[pos].b_sup_right.y), cv::Scalar(255), 2, 8, 0); //topo
    line(image_camera, cv::Point(block_i[pos].b_sup_left.x, block_i[pos].b_sup_left.y), cv::Point(block_i[pos].b_inf_left.x, block_i[pos].b_inf_left.y), cv::Scalar(255), 2, 8, 0); //left
    line(image_camera, cv::Point(block_i[pos].b_inf_left.x, block_i[pos].b_inf_left.y), cv::Point(block_i[pos].b_inf_right.x, block_i[pos].b_inf_right.y), cv::Scalar(255), 2, 8, 0); //bottom
    line(image_camera, cv::Point(block_i[pos].b_inf_right.x, block_i[pos].b_inf_right.y), cv::Point(block_i[pos].b_sup_right.x, block_i[pos].b_sup_right.y), cv::Scalar(255), 2, 8, 0); //right

    draw_check_function(id, block_i[pos].b_sup_left.x, block_i[pos].b_sup_left.y - 5, image_camera);

    draw_points(id, block_i[pos].b_sup_left, block_i[pos].b_sup_right, block_i[pos].b_inf_left, block_i[pos].b_inf_right, image_camera);
}


void drawing_functions(cv::InputOutputArray image, std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids, std::vector <block> block_i)
{
    if (ids.size() > 0) {
        current_ids_size = ids.size();
        //aruco::drawDetectedMarkers(image, corners, ids);

        //run through every detected aruco
        for (int i = 0; i < corners.size(); i++)
        {
            draw_full_block(image, ids[i], i, corners, block_i);
        }
    }
}


// Count number of times ArUco type was detected
int check_occurences(int id, std::vector<block> vect)
{
    int count=0;

    for (int j = 0; j < vect.size()-1; j++)
    {

        if(vect[j].id == id) count++;
    }

    return count;
}


//get measures of blocks given the size of aruco that is being read
int get_topmargin(int size_aruco) {
    return size_aruco * (0.2/ 0.8);
}

int get_bottommargin(int size_aruco) {
    return size_aruco * (0.15/ 0.8);
}

int get_topblock(int size_aruco) {
    return size_aruco * (1 / 0.8) * 0.15;
}


// Saves corners of the Block
std::vector <block> corners_blocks(int id, int pos, std::vector<std::vector<cv::Point2f>> corners, std::vector <block> block_i)
{
    if ((id != 28) && (id != 29) && (id != 30) && (id<31))
    {


        block_i.push_back(block());

        block_i[pos].count=check_occurences(id,block_i)+1;

        block_i[pos].id=id;



        block_i[pos].size_aruco = corners[pos][1].x - corners[pos][0].x;
        block_i[pos].b_sup_left.x = corners[pos][0].x - get_topblock(block_i[pos].size_aruco);
        block_i[pos].b_sup_left.y = corners[pos][0].y - get_topmargin(block_i[pos].size_aruco);

        block_i[pos].b_sup_right.x = corners[pos][1].x + get_topblock(block_i[pos].size_aruco);
        block_i[pos].b_sup_right.y = corners[pos][1].y - get_topmargin(block_i[pos].size_aruco);

        block_i[pos].b_inf_left.x = corners[pos][3].x - get_topblock(block_i[pos].size_aruco);
        block_i[pos].b_inf_left.y = corners[pos][3].y + get_bottommargin(block_i[pos].size_aruco);

        block_i[pos].b_inf_right.x = corners[pos][2].x + get_topblock(block_i[pos].size_aruco);
        block_i[pos].b_inf_right.y = corners[pos][2].y + get_bottommargin(block_i[pos].size_aruco);
    }

    return block_i;
}


// Saves Block Inputs and Outputs
std::vector <block> save_in_out(cv::InputOutputArray image, coordinates sup_esq, coordinates sup_dir, coordinates inf_esq, coordinates inf_dir, int pos_list, int id, std::vector <block> block_i)
{

    masks.push_back(cv::Vec4i());
    //min_x max_x min_y max_y
    masks[pos_list][0]=sup_esq.x;
    masks[pos_list][1]=inf_dir.x;
    masks[pos_list][2]=sup_esq.y;
    masks[pos_list][3]=inf_dir.y;

    if (id == 0 || id == 1 || id == 3 || id == 4 || id == 5 || id == 6 || id == 13 || id == 25 || id == 26)
    {
        //2 inputs 1 output

        block_i[pos_list].id = id;
        block_i[pos_list].outputs.point.x = sup_dir.x;
        block_i[pos_list].outputs.point.y = sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5);
        block_i[pos_list].input1.point.x = sup_esq.x;
        block_i[pos_list].input1.point.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.25));
        block_i[pos_list].input2.point.x = sup_esq.x;
        block_i[pos_list].input2.point.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.75));
    }
    else if (id == 2 || id == 14 || id == 15 || id == 16 || id == 17 || id == 18 || id == 19 || id == 20 || id == 21 || id == 22 || id == 23 || id == 24)
    {
        //1 input 1 output

        block_i[pos_list].id = id;
        block_i[pos_list].outputs.point.x = sup_dir.x;
        block_i[pos_list].outputs.point.y = sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5);
        block_i[pos_list].input1.point.x = sup_esq.x;
        block_i[pos_list].input1.point.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.5));
    }
    else if (id == 7 || id == 8)
    {
        //1 input

        block_i[pos_list].id = id;
        block_i[pos_list].input1.point.x = sup_esq.x;
        block_i[pos_list].input1.point.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.5));
    }
    else if (id == 9 || id == 10 || id == 11 || id == 12)
    {
        //1 output

        block_i[pos_list].id = id;
        block_i[pos_list].outputs.point.x = sup_dir.x;
        block_i[pos_list].outputs.point.y = sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5);
    }
    else if (id == 27)
    {
        //2 inputs 1 output 1 condition

        block_i[pos_list].id = id;
        block_i[pos_list].outputs.point.x = sup_dir.x;
        block_i[pos_list].outputs.point.y = sup_dir.y + ((inf_dir.y - sup_dir.y) * 0.5);
        block_i[pos_list].input1.point.x = sup_esq.x;
        block_i[pos_list].input1.point.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.25));
        block_i[pos_list].input2.point.x = sup_esq.x;
        block_i[pos_list].input2.point.y = sup_esq.y + ((inf_esq.y - sup_esq.y) * (0.75));
        block_i[pos_list].condition.point.x = ((inf_dir.x-inf_esq.x)*0.5)+inf_esq.x;
        block_i[pos_list].condition.point.y = inf_dir.y;
    }

    return block_i;
}


// Saves every Block's Corners and I/Os
std::vector <block> saving_coordinates(cv::InputOutputArray image, std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids, std::vector <block> block_i)
{
    if (ids.size() > 0)
    {
        current_ids_size = ids.size();

        //run through every detected aruco
        for (int i = 0; i < corners.size(); i++)
        {
            block_i = corners_blocks(ids[i], i, corners, block_i);

            block_i = save_in_out(image, block_i[i].b_sup_left, block_i[i].b_sup_right, block_i[i].b_inf_left, block_i[i].b_inf_right, i, ids[i], block_i);
        }
    }

    return block_i;
}


std::string check_function(int id)
{
    if (id == 0) return "Aruco SUM";
    else if (id == 1)return "Aruco PRODUCT";
    else if (id == 2) return "Aruco INVERSE";
    else if (id == 3) return "Aruco IF";
    else if (id == 4) return "Aruco LESS THAN";
    else if (id == 5) return "Aruco MORE THAN";
    else if (id == 6) return "Aruco EQUAL";
    else if (id == 7) return "Aruco LEFT MOTOR";
    else if (id == 8) return "Aruco RIGHT MOTOR";
    else if (id == 9) return "Aruco LEFT SENSOR";
    else if (id == 10) return "Aruco RIGHT SENSOR";
    else if (id == 11) return "Aruco FRONT SENSOR";
    else if (id == 12) return "Aruco REAR SENSOR";
    else if (id == 13) return "Aruco TIMER";
    else if (id == 14) return "Aruco NUMBER 0";
    else if (id == 15) return "Aruco NUMBER 1";
    else if (id == 16) return "Aruco NUMBER 2";
    else if (id == 17) return "Aruco NUMBER 3";
    else if (id == 18) return "Aruco NUMBER 4";
    else if (id == 19) return "Aruco NUMBER 5";
    else if (id == 20) return "Aruco NUMBER 6";
    else if (id == 21) return "Aruco NUMBER 7";
    else if (id == 22) return "Aruco NUMBER 8";
    else if (id == 23) return "Aruco NUMBER 9";
    else if (id == 24) return "Aruco DOT";
    else if (id == 25) return "Aruco AND";
    else if (id == 26) return "Aruco OR";
    else if (id == 27) return "Aruco MUX";
}


std::vector<block> put_arucos_order(std::vector<block> blocks )
{
    std::vector<block> new_order_block;

    //copy.assign(blocks.begin(),blocks.end());

    int min;
    int index_min;


    while(blocks.size()>0)
    {
        //cout  << "Iteration: " << blocks.size() << "\n";

        min=blocks[0].b_sup_left.x;
        index_min=0;
        for(int i=0; i<blocks.size(); i++)
        {
            //cout << blocks[i].id << "  " << check_function(blocks[i].id) << "\n";

            if(blocks[i].b_sup_left.x < min)
            {
                min = blocks[i].b_sup_left.x;
                index_min=i;
            }
        }
        //cout << min << "  IMHERE\n";

        new_order_block.push_back(blocks[index_min]);

        std::vector<block>::iterator it;
        it= blocks.begin()+index_min;
        blocks.erase(it);
    }

    return new_order_block;
}


void DebugBlocks(std::vector<block> block_in_order)
{
    for (int j = 0; j < block_in_order.size(); j++)
    {
        ROS_WARN_STREAM("ARUCO OF ID " <<  block_in_order[j].id << "(" << block_in_order[j].count << ") --> " << check_function(block_in_order[j].id) << " : Outputs-->" << block_in_order[j].outputs.point.x << "  " << block_in_order[j].outputs.point.y << "  " << "\n");
        ROS_WARN_STREAM("                                    "  << " : Input1-->" << block_in_order[j].input1.point.x << "  " << block_in_order[j].input1.point.y << "  " << "\n");
        ROS_WARN_STREAM("                                    "  << " : Input2-->" << block_in_order[j].input2.point.x << "  " << block_in_order[j].input2.point.y << "  " << "\n");
        ROS_WARN_STREAM("MASK    minx " << masks[j][0] << "    maxx " << masks[j][1] << "    miny " << masks[j][2] << "    maxy " << masks[j][3] << "\n");
        ROS_WARN_STREAM("-----------------------------------------------------------------------------------------\n");
    }
}

void Debugcombs(std::vector<combination> comb)
{
    for (int j = 0; j < comb.size(); j++)
    {
        ROS_WARN_STREAM("COMB " << j << ": (number --> " << comb[j].number <<  " / " << "matrix_pos --> " << comb[j].matrix_pos << " / " << "dest --> " << comb[j].dest << ")\n");
    }

}

void Debugmatrixlinks(std::vector<std::vector<int>> matrix)
{
    for (int j = 0; j < matrix.size() ; j++) {
        std::cout << "row:" << j << " ";
        for (int i= 0; i < matrix.size() ; i++) {
            std::cout << matrix[j][i]<< " ";
        }
        std::cout << "\n";
    }

}

void Debugmatrixvalues(std::vector<std::vector<float>> matrix)
{
    for (int j = 0; j < matrix.size() ; j++) {
        std::cout << "row:" << j << " ";
        for (int i= 0; i < matrix.size() ; i++) {
            std::cout << matrix[j][i]<< " ";
        }
        std::cout << "\n";
    }

}



// ---------- LINE DETECTION ---------- (add to separate library)


//position of I/Os in matrix
int position_matrix_input1(int id, int count){
    if(id==0){
        if(count==1){
            return 8;
        }
        else if(count==2){
            return 12;
        }
    }

    else if(id==1){
        if(count==1){
            return 20;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==2){
        if(count==1){
            return 6;
        }
        else if(count==2){
            return 15;
        }
    }

    else if(id==3){
        if(count==1){
            return 39;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==4){
        if(count==1){
            return 56;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==5){
        if(count==1){
            return 53;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==6){
        if(count==1){
            return 59;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==7){
        if(count==1){
            return 17;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==8){
        if(count==1){
            return 24;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==13){
        if(count==1){
            return 22;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==14){
        if(count==1){
            return 62;
        }
        else if(count==2){
            return 84;
        }
    }

    else if(id==15){
        if(count==1){
            return 64;
        }
        else if(count==2){
            return 86;
        }
    }

    else if(id==16){
        if(count==1){
            return 66;
        }
        else if(count==2){
            return 88;
        }
    }

    else if(id==17){
        if(count==1){
            return 68;
        }
        else if(count==2){
            return 90;
        }
    }

    else if(id==18){
        if(count==1){
            return 70;
        }
        else if(count==2){
            return 92;
        }
    }

    else if(id==19){
        if(count==1){
            return 72;
        }
        else if(count==2){
            return 94;
        }
    }

    else if(id==20){
        if(count==1){
            return 74;
        }
        else if(count==2){
            return 96;
        }
    }

    else if(id==21){
        if(count==1){
            return 76;
        }
        else if(count==2){
            return 98;
        }
    }

    else if(id==22){
        if(count==1){
            return 78;
        }
        else if(count==2){
            return 100;
        }
    }

    else if(id==23){
        if(count==1){
            return 80;
        }
        else if(count==2){
            return 102;
        }
    }

    else if(id==24){
        if(count==1){
            return 82;
        }
        else if(count==2){
            return 104;
        }
    }

    else if(id==25){
        if(count==1){
            return  47;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==26){
        if(count==1){
            return 50;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==27){
        if(count==1){
            return 26;
        }
        else if(count==2){
            return 31;
        }
    }


}

int position_matrix_input2(int id,int count){

    if(id==0){
        if(count==1){
            return 9;
        }
        else if(count==2){
            return 11;
        }
    }

    else if(id==1){
        if(count==1){
            return 38;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==3){
        if(count==1){
            return 40;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==4){
        if(count==1){
            return 57;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==5){
        if(count==1){
            return 54;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==6){
        if(count==1){
            return 60;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==25){
        if(count==1){
            return 48;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==26){
        if(count==1){
            return 51;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==27){
        if(count==1){
            return 27;
        }
        else if(count==2){
            return 32;
        }
    }



}

int position_matrix_output(int id, int count){
    if(id==0){
        if(count==1){
            return 10;
        }
        else if(count==2){
            return 13;
        }
    }

    else if(id==1){
        if(count==1){
            return 21;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==2){
        if(count==1){
            return 7;
        }
        else if(count==2){
            return 16;
        }
    }

    else if(id==3){
        if(count==1){
            return 41;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==4){
        if(count==1){
            return 58;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==5){
        if(count==1){
            return 55;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==6){
        if(count==1){
            return 61;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==9){

        return 2;

    }

    else if(id==10){

        return 4;

    }

    else if(id==11){

        return 3;

    }

    else if(id==12){

        return 35;

    }

    else if(id==13){
        if(count==1){
            return 23;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==14){
        if(count==1){
            return 63;
        }
        else if(count==2){
            return 85;
        }
    }

    else if(id==15){
        if(count==1){
            return 65;
        }
        else if(count==2){
            return 87;
        }
    }

    else if(id==16){
        if(count==1){
            return 67;
        }
        else if(count==2){
            return 89;
        }
    }

    else if(id==17){
        if(count==1){
            return 69;
        }
        else if(count==2){
            return 91;
        }
    }

    else if(id==18){
        if(count==1){
            return 71;
        }
        else if(count==2){
            return 93;
        }
    }

    else if(id==19){
        if(count==1){
            return 73;
        }
        else if(count==2){
            return 95;
        }
    }

    else if(id==20){
        if(count==1){
            return 75;
        }
        else if(count==2){
            return 97;
        }
    }

    else if(id==21){
        if(count==1){
            return 77;
        }
        else if(count==2){
            return 99;
        }
    }

    else if(id==22){
        if(count==1){
            return 79;
        }
        else if(count==2){
            return 101;
        }
    }

    else if(id==23){
        if(count==1){
            return 81;
        }
        else if(count==2){
            return 103;
        }
    }

    else if(id==24){
        if(count==1){
            return 83;
        }
        else if(count==2){
            return 105;
        }
    }

    else if(id==25){
        if(count==1){
            return 49;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==26){
        if(count==1){
            return 52;
        }
        else if(count==2){
            return -1;
        }
    }

    else if(id==27){
        if(count==1){
            return 29;
        }
        else if(count==2){
            return 34;
        }
    }



}

int position_matrix_condition(int id, int count){
    if(id==27){
        if(count==1){
            return 28;
        }
        else if(count==2){
            return 33;
        }
    }
}


bool isInside(int circle_x, int circle_y, int rad, int x, int y)
{
    // Compare radius of circle with distance
    // of its center from given point
    if ((x - circle_x) * (x - circle_x) +
            (y - circle_y) * (y - circle_y) <= rad * rad)
        return true;
    else
        return false;
}


std::vector<block> check_lines(cv::Vec4i lin, size_t j, int radius, std::vector<block> vec)
{
    for(int k=0;k<vec.size();k++)
    {
        // output points
        if(isInside(vec[k].outputs.point.x, vec[k].outputs.point.y,radius,lin[0], lin[1])==1)
        {
            //            ROS_WARN_STREAM("It is false that the point " << lin[0] << " " << lin[1] << " of the line " << j << " is near the aruco " << vec[k].id << " output\n");

            for(int j=0;j<vec.size();j++)
            {

                if((isInside(vec[j].input1.point.x, vec[j].input1.point.y,radius,lin[2], lin[3])==1))
                {
                    //                    ROS_WARN_STREAM("The output of aruco " << vec[k].id << " is connected to the top input of aruco " << vec[j].id << "\n\n");

                    vec[k].outputs.linked=true;
                    vec[k].outputs.order=position_matrix_output(vec[k].id,vec[k].count);
                    vec[k].outputs.link_end=position_matrix_input1(vec[j].id,vec[j].count);

                    vec[j].input1.linked=true;
                    vec[j].input1.order=position_matrix_input1(vec[j].id,vec[j].count);
                    vec[j].input1.link_end=position_matrix_output(vec[k].id,vec[k].count);



                }
                else if((isInside(vec[j].input2.point.x, vec[j].input2.point.y,radius,lin[2], lin[3])==1))
                {
                    //                    ROS_WARN_STREAM("The output of aruco " << vec[k].id << " is connected to the bottom input of aruco " << vec[j].id << "\n\n");

                    vec[k].outputs.linked=true;
                    vec[k].outputs.order=position_matrix_output(vec[k].id,vec[k].count);
                    vec[k].outputs.link_end=position_matrix_input2(vec[j].id,vec[j].count);

                    vec[j].input2.linked=true;
                    vec[j].input2.order=position_matrix_input2(vec[j].id,vec[j].count);
                    vec[j].input2.link_end=position_matrix_output(vec[k].id,vec[k].count);


                }
                else if((isInside(vec[j].condition.point.x, vec[j].condition.point.y,radius,lin[2], lin[3])==1))
                {
                    //                    ROS_WARN_STREAM("The output of aruco " << vec[k].id << " is connected to the bottom input of aruco " << vec[j].id << "\n\n");

                    vec[k].outputs.linked=true;
                    vec[k].outputs.order=position_matrix_output(vec[k].id,vec[k].count);
                    vec[k].outputs.link_end=position_matrix_condition(vec[j].id,vec[j].count);

                    vec[j].condition.linked=true;
                    vec[j].condition.order=position_matrix_condition(vec[j].id,vec[j].count);
                    vec[j].condition.link_end=position_matrix_output(vec[k].id,vec[k].count);


                }
                //                else ROS_WARN_STREAM("WRONG lINK\n");
            }
        }
        else if(isInside(vec[k].outputs.point.x, vec[k].outputs.point.y,radius,lin[2], lin[3])==1)
        {
            //std::cout <<"It is false that the point " << lin[2] << " " << lin[3] << " of the line " << j << " is near the aruco " << vec[k].id << " output\n";
            for(int j=0;j<vec.size();j++)
            {
                if((isInside(vec[j].input1.point.x, vec[j].input1.point.y,radius,lin[0], lin[1])==1))
                {
                    //                    std::cout << "The output of aruco " << vec[k].id << " is connected to the top input of aruco " << vec[j].id << "\n\n";

                    vec[k].outputs.linked=true;
                    vec[k].outputs.order=position_matrix_output(vec[k].id,vec[k].count);
                    vec[k].outputs.link_end=position_matrix_input1(vec[j].id,vec[j].count);

                    vec[j].input1.linked=true;
                    vec[j].input1.order=position_matrix_input1(vec[j].id,vec[j].count);
                    vec[j].input1.link_end=position_matrix_output(vec[k].id,vec[k].count);



                }
                else if((isInside(vec[j].input2.point.x, vec[j].input2.point.y,radius,lin[0], lin[1])==1))
                {
                    //                    std::cout << "The output of aruco " << vec[k].id << " is connected to the bottom input of aruco " << vec[j].id << "\n\n";

                    vec[k].outputs.linked=true;
                    vec[k].outputs.order=position_matrix_output(vec[k].id,vec[k].count);
                    vec[k].outputs.link_end=position_matrix_input2(vec[j].id,vec[j].count);

                    vec[j].input2.linked=true;
                    vec[j].input2.order=position_matrix_input2(vec[j].id,vec[j].count);
                    vec[j].input2.link_end=position_matrix_output(vec[k].id,vec[k].count);


                }
                else if((isInside(vec[j].condition.point.x, vec[j].condition.point.y,radius,lin[0], lin[1])==1))
                {
                    //                    std::cout << "The output of aruco " << vec[k].id << " is connected to the bottom input of aruco " << vec[j].id << "\n\n";

                    vec[k].outputs.linked=true;
                    vec[k].outputs.order=position_matrix_output(vec[k].id,vec[k].count);
                    vec[k].outputs.link_end=position_matrix_condition(vec[j].id,vec[j].count);

                    vec[j].condition.linked=true;
                    vec[j].condition.order=position_matrix_condition(vec[j].id,vec[j].count);
                    vec[j].condition.link_end=position_matrix_output(vec[k].id,vec[k].count);


                }
                //                else ROS_WARN_STREAM("WRONG lINK\n");
            }
        }
    }


    return vec;
}


std::vector<cv::Point2f> detectCrossings(cv::Mat image)
{
    // Skeleton

    //    cv::imshow("input", image);

    cv::Mat img = image.clone();
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do
    {
        cv::erode(img, eroded, element);
        cv::dilate(eroded, temp, element); // temp = open(img)
        cv::subtract(img, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(img);

        done = (cv::countNonZero(img) == 0);
    } while (!done);

    //    cv::imshow("Skeleton", skel);


    // Pre-Process Skeleton

    cv::morphologyEx(skel, skel, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    //    cv::imshow("Close", skel);


    // Detecting corners

    cv::Mat dst, dst_norm, dst_norm_scaled;

    dst = cv::Mat::zeros(skel.size(), CV_32FC1);
    cornerHarris(skel, dst, 10, 5, 0.05, cv::BORDER_DEFAULT);
    //    cv::imshow("dst", dst);


    // Normalizing

    normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    //    cv::imshow("dst_norm", dst_norm);how to draw cross in opnecv

    convertScaleAbs(dst_norm, dst_norm_scaled);
    //    cv::imshow("dst_norm_scaled", dst_norm_scaled);


    // Filtering

    cv::Mat crossingPoints;

    cv::threshold(dst_norm_scaled, crossingPoints, 120, 255, CV_THRESH_BINARY);
    //    cv::imshow("crossMask", crossingPoints);

    cv::dilate(crossingPoints, crossingPoints, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
    //    cv::imshow("Dilate After Thresh", crossingPoints);


    // Save corner points

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    // find contours
    cv::findContours( crossingPoints, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    // get the moments
    std::vector<cv::Moments> mu(contours.size());
    for( int i = 0; i<contours.size(); i++ )
    {
        mu[i] = cv::moments( contours[i], false );
    }

    // get the centroid of figures.
    std::vector<cv::Point2f> mc(contours.size());
    for( int i = 0; i<contours.size(); i++)
    {
        mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    }


    //cv::waitKey(1);

    return mc;
}


std::vector<cv::Vec4i> detectLines(cv::Mat paper)
{
    cv::Mat image;
    cv::cvtColor(paper, image, cv::COLOR_BGR2GRAY);

    //imshow("Input Image", image);


    // Mask To Eliminate ArUcos
    for(int q=0; q<masks.size(); q++)
    {
        for(int w=masks[q][2]-8; w<masks[q][3]+8; w++)
        {
            for(int e=masks[q][0]-5; e<masks[q][1]+5; e++)
            {
                image.at<uchar>(w,e)=255;
            }
        }
    }

    //   cv::imshow("Take Off Arucos", image);


    // Threshold to eliminate white background
    cv::threshold(image, image, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);

    //    cv::imshow("Otsu Threshold ", image);


    // Keep only paper area
    rectangle(image, cv::Rect(0, 0, image.cols, image.rows), cv::Scalar(255));
    floodFill(image, cv::Point(0, 0), cv::Scalar(0));

    //cv::imshow("Flood Fill After Rect", image);


    // Line dilation
    cv::Mat kernel=cv::Mat(cv::Size(3,3),CV_8UC1,cv::Scalar(255));
    morphologyEx(image,image,cv::MORPH_DILATE,kernel);

    //cv::imshow("Dilation", image);


    // Detect Crossings
    //crossingPoints.clear();
    //crossingPoints = detectCrossings(image);


    // Detect Lines
    std::vector<std::vector<cv::Point> > contours;
    findContours(image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Vec4i> linesP;

    cv::Mat paperCopy = paper.clone();

    for(size_t k=0; k<contours.size(); k++)
    {
        //        // See each countour being detected
        //cv::drawContours(paperCopy, contours, (int)k, cv::Scalar(0, 255, 0), 2, cv::LINE_8, cv::noArray(), 0);
        //cv::imshow("countours", paperCopy);
        //cv::waitKey(0);

        auto val=minmax_element(contours[k].begin(), contours[k].end(), [](cv::Point const& a, cv::Point const& b)
        {
                return a.x < b.x;
    });

        //circle(image, Point(val.first->x,val.first->y), 13, Scalar(0, 255, 0), 8, LINE_AA);
        //circle(image, Point(val.second->x,val.second->y), 13, Scalar(0, 255, 0), 8, LINE_AA);
        linesP.push_back(cv::Vec4i());
        linesP[k][0]=val.first->x;
        linesP[k][1]=val.first->y;
        linesP[k][2]=val.second->x;
        linesP[k][3]=val.second->y;

        //        ROS_WARN_STREAM(" leftMost [ " << val.first->x << ", " << val.first->y << " ]");
        //        ROS_WARN_STREAM(" RightMost [ " << val.second->x << ", " << val.second->y << " ]");
    }

    //    cv::imshow("countours", paperCopy);
    //    cv::waitKey(1);

    return linesP;
}


std::vector<block> saveLines(std::vector<cv::Vec4i> linesP, std::vector<block> blocks)
{
    // Draw Lines
    for (size_t i = 0; i < linesP.size(); i++)
    {
        cv::Vec4i l = linesP[i];

        blocks = check_lines(l,i,18,blocks);

    }
    return blocks;
}

coordinates findEndPoint(int begin, std::vector<block> blocks){
    for (int j = 0; j < blocks.size(); j++) {
        if(blocks[j].input1.order==begin){
            return blocks[j].input1.point;
        }
        else if(blocks[j].input2.order==begin){
            return blocks[j].input2.point;
        }
        else if(blocks[j].condition.order==begin){
            return blocks[j].condition.point;
        }

    }

}


void drawLines(cv::InputOutputArray paper, std::vector<block> blocks)
{
    // Draw Lines
    for (int j = 0; j < blocks.size(); j++)
    {
        if(blocks[j].outputs.linked==true)
        {
            line(paper, cv::Point(blocks[j].outputs.point.x,blocks[j].outputs.point.y), cv::Point(findEndPoint(blocks[j].outputs.link_end,blocks).x, findEndPoint(blocks[j].outputs.link_end,blocks).y), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }
    }

    // Draw Crossings
    for( int i = 0; i<crossingPoints.size(); i++ )
    {
        cv::circle(paper, crossingPoints[i], 2, cv::Scalar(0,255,0), -1, 8, 0);
    }

}

void drawMatrixLinks(int m_links[100][100],std::vector<block> block,std::vector<combination> comb){
    for (int j = 0; j < block.size(); j++) {
        if(block[j].id<14 || block[j].id>24){
            if(block[j].outputs.linked==true){
                m_links[block[j].outputs.order][block[j].outputs.link_end]=1;
                m_links[block[j].outputs.link_end][block[j].outputs.order]=1;
            }
        }

    }
    for (int j = 0; j < comb.size(); j++) {
        m_links[comb[j].matrix_pos][comb[j].dest]=1;
        m_links[comb[j].dest][comb[j].matrix_pos]=1;
    }


}

int get_k(int id){

    if(id == 14) return 0;
    else if (id == 15) return 1;
    else if (id == 16) return 2;
    else if (id == 17) return 3;
    else if (id == 18) return 4;
    else if (id == 19) return 5;
    else if (id == 20) return 6;
    else if (id == 21) return 7;
    else if (id == 22) return 8;
    else if (id == 23) return 9;
    else if (id == 24) return -1;

}

void drawMatrixValues(float m_values[100][100], std::vector<block> block, std::vector<combination> comb){

    for (int j = 0; j < comb.size(); j++) {
        m_values[comb[j].matrix_pos][comb[j].dest]=comb[j].number;
        m_values[comb[j].dest][comb[j].matrix_pos]=comb[j].number;
    }

}

std::vector<combination> getCombinations(std::vector<block> blocks, std::vector<combination> comb){
    int count=0;
    for (int j = 0; j < blocks.size(); j++) {
        if(blocks[j].id<14 || blocks[j].id>24){
            if(blocks[j].input1.link_end>61 && blocks[j].input1.link_end<106){
                count=count+1;
                comb.push_back(combination());
                comb[count-1].matrix_pos=62+count;
            }
            if(blocks[j].input2.link_end>61 && blocks[j].input2.link_end<106){
                count=count+1;
                comb.push_back(combination());
                comb[count-1].matrix_pos=62+count;
            }
        }
    }
    num_combinations=count;
    return comb;
}

std::vector<combination> makeCombinations(std::vector<block> blocks, std::vector<combination> comb){
    int comb_pos=0;
    bool found;
    int endpoint;


    for (int j = 0; j < blocks.size(); j++) {
        int combo_number=0;
        std::vector<int> numbers;
        int save_index;
        int divisor=0;

        if(blocks[j].id<14 || blocks[j].id>24){

            //INPUT1
            if(blocks[j].input1.link_end>61 && blocks[j].input1.link_end<106){

                found=blocks[j].input1.linked;
                endpoint=blocks[j].input1.link_end;

                do{
                    for (int k = 0; k < blocks.size(); k++) {
                        if(endpoint==blocks[k].outputs.order){
                            numbers.push_back(get_k(blocks[k].id));
                            save_index=k;

                        }
                    }
                    endpoint=blocks[save_index].input1.link_end;
                    found=blocks[save_index].input1.linked;
                }
                while(found==1);


                for (int k = 0; k < numbers.size(); k++) {
                    if(numbers[k]==-1){
                        divisor=k;
                        numbers.erase(numbers.begin()+k);
                    }
                }

                int N=numbers.size();
                for (int k = 0; k < numbers.size(); k++) {
                    combo_number += numbers[k]*pow(10, k);
                }

                //std::cout << "POW:" << pow(10,divisor) << "\n";
                comb[comb_pos].number=combo_number/pow(10,divisor);
                //std::cout << "NUM:" << comb[comb_pos].number << "\n";
                comb[comb_pos].dest=blocks[j].input1.order;
                comb_pos=comb_pos+1;
                combo_number=0;
                divisor=0;
                numbers.clear();
            }


            //INPUT2
            if(blocks[j].input2.link_end>61 && blocks[j].input2.link_end<106){

                found=blocks[j].input2.linked;
                endpoint=blocks[j].input2.link_end;

                do{
                    for (int k = 0; k < blocks.size(); k++) {
                        if(endpoint==blocks[k].outputs.order){

                            numbers.push_back(get_k(blocks[k].id));
                            save_index=k;

                        }
                    }
                    endpoint=blocks[save_index].input1.link_end;
                    found=blocks[save_index].input1.linked;
                }
                while(found==1);

                for (int k = 0; k < numbers.size(); k++) {
                    if(numbers[k]==-1){
                        divisor=k;
                        numbers.erase(numbers.begin()+k);
                    }
                }

                int N=numbers.size();
                for (int k = 0; k < numbers.size(); k++) {
                    combo_number += numbers[k]*pow(10, k);
                }

                comb[comb_pos].number=combo_number/(pow(10,divisor));
                comb[comb_pos].dest=blocks[j].input2.order;
                comb_pos=comb_pos+1;
                combo_number=0;
                divisor=0;
                numbers.clear();

            }


        }


    }
    return comb;
}

// ---------- LINE DETECTION LOGIC ----------
void detectAndInterpret_Lines(cv::Mat& frame, cv::Ptr<cv::aruco::Dictionary> dict, int m_links[100][100], float m_values[100][100], int& ready)
{

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(frame, dict, corners, ids);

    // Number of detections after Homography has to be equal to the number of detections before
    // -4 to account for loss of cropped orientation blocks
    if(ids.size() != (count_total_arucos - 4)) return;

    count_total_arucos=0;


    // Block Formation

    std::vector <block> block_i;
    masks.clear(); // used to eliminate the arucos during line detection

    block_i = saving_coordinates(frame, corners, ids, block_i);

    std::vector<block> block_in_order = put_arucos_order(block_i);

//    DebugBlocks(block_in_order);


    // Line Detection

    std::vector<cv::Vec4i> linesP = detectLines(frame);

    block_in_order=saveLines(linesP, block_in_order);


    // Draw detections

    drawing_functions(frame, corners, ids, block_i);

    drawLines(frame, block_in_order);


    // Get combinations
    std::vector<combination> combs;
    combs=getCombinations(block_in_order,combs);
    combs=makeCombinations(block_in_order,combs);
//    Debugcombs(combs);
//    std::cout << "Num of combinations " << num_combinations << "\n";

    drawMatrixLinks(m_links,block_in_order,combs);
    drawMatrixValues(m_values,block_in_order,combs);

//    Debugmatrixlinks(matrix_links);
//    Debugmatrixvalues(matrix_values);


    ready=1;
}



// ---------- PAPER LOGIC (MAIN FUNCTION) ----------
int detectAndInterpret_Paper(cv::Mat& frame, cv::Ptr<cv::aruco::Dictionary> dict, int m_links[100][100], float m_values[100][100], int ready)
{
    cv::Mat frameCopy;
    frameCopy = frame.clone();
    // ArUco Detection
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(frame, dict, corners, ids); // corners returned clockwise, starting with top left

    // Get orientation blocks
    orientation_block markers = detect_orientation_blocks(corners, ids);

    std::vector<cv::Point2f> new_points;

    // Compute extended dimensions to account for distortion
    if(markers.ids.size()==4 && orientation_check)
    {
        new_points = calculateNewDimensions(frameCopy, markers);
    }

    // Frame Validation
    validatePicture(corners, ids, frame);

    // Perspective correction
    if(markers.ids.size()==4 && orientation_check && pictureValidated)
    {
        frame = perspective_correction(frame, new_points);

        detectAndInterpret_Lines(frame, dict, m_links, m_values, ready);
    }

    return ready;

}



// ---------- MAIN FOR TESTS ----------
/*
int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_tests_node");
    ros::NodeHandle n_public;


    //    // Get Camera Calibration Parameters
    //    camera_parameters(1);

    // Create a VideoCapture object and open the input file
    // If the input is the web camera, pass 0 instead of the video file name
    cv::VideoCapture cap("../catkin_ws/src/SERP/serp/include/tests/teste_arucos.h264");

    // Check if camera opened successfully
    if(!cap.isOpened())
    {
        ROS_WARN_STREAM("Error opening video stream or file");
        return -1;
    }

    while(ros::ok())
    {
        cv::Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty()) break;

        // Display the resulting frame
        //        cv::imshow("Original Frame", frame);
        //        cv::waitKey(1);

        //        // Correct Frame Distortion
        //        cv::Mat undist_frame = correctImage(frame, 1);
        //        cv::imshow("Fixed Frame", undist_frame);
        //        cv::waitKey(1);

        // ArUco Identification
        detectAndInterpret_Paper(frame, dictionary);

        ros::spinOnce();
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}
*/
