#include "proximity_sensor.h"

const double pi = 3.1416;

//angle at -1/4*pi  aka  -45º
double convert_Left(double distance){
    //y = (-118.7824*x)/(-258.2981 + x)
    double aux = (-118.7824 * distance)/(-258.2981 + distance);
    aux = aux * 0.1  * 2;
    if (aux > 95.0)
        return 95.0;
    if (aux < 0.0)
        return 0.0;
    return aux;
}


//angle at 0  aka  0º
double convert_Front(double distance){
    //y = (-108.4156*x)/(-264.6905 + x)
    double aux = (-108.4156 * distance)/(-264.6905 + distance);
    aux = aux * 0.1 * 2;
    if (aux > 95.0)
        return 95.0;
    if (aux < 0.0)
        return 0.0;
    return aux;
}


//angle at 1/4*pi  aka  45º
double convert_Right(double distance){
	//y = (-104.0271*x)/(-256.1551 + x)
    double aux = (-104.0271 * distance)/(-256.1551 + distance);
    aux = aux * 0.1 * 2;
    if (aux > 95.0)
        return 95.0;
    if (aux < 0.0)
        return 0.0;
    return aux;
}


//angle at pi  aka  180º
double convert_Back(double distance){
    //y = (-127.9308*x)/(-220.4707 + x)
    double aux = (-127.9308 * distance)/(-220.4707 + distance);
    aux = aux * 0.1 * 2;
    aux = aux - 25;
    if (aux > 95.0)
        return 95.0;
    if (aux < 0.0)
        return 0.0;
    return aux;
}

	
	
int main(int argc, char** argv)
{
    ros::init(argc, argv, "proximity_sensor_node");
    ros::NodeHandle n_public("~");
    ros::Rate rate(20); //rate set at 20 Hz

    bool debug_mode = false;
    bool publish_raw_image = false;

    // SERP2 Sensors Values Publisher
    ros::Publisher sensor_pub = n_public.advertise<serp::ObjectDetection>("/sensors", 1);

    // Raw Image Publisher
    image_transport::ImageTransport it(n_public);
    image_transport::Publisher pub_img = it.advertise("/camera/image", 1);
    sensor_msgs::ImagePtr img_msg;

    // 200º Rasp Camera
    raspicam::RaspiCam_Cv Camera;
    Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    Camera.set(CV_CAP_PROP_FRAME_WIDTH, 600);
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 450);
    //shutter speed, 1-100 for 0-33ms

    // Check if camera opened successfully
    std::cout << "Opening Camera...";
    if (!Camera.open())
    {
        ROS_WARN_STREAM("Error opening the camera");
        return -1;
    }
    std::cout << " done successfully!" << std::endl;

    while(ros::ok())
    {
        cv::Mat frame; // = cv::imread("/home/percmap/catkin_ws/src/ROS_tutorial/fisheye_camera_simulated_sensors_ros1-master/images/test5.jpg", CV_LOAD_IMAGE_COLOR);
        //cv::resize(frame, frame, cv::Size(600, 450));

        // Get new frame
        Camera.grab();
        Camera.retrieve(frame);

        // If the frame is empty, break immediately
        //if (frame.empty()) break;



        if(publish_raw_image){
            img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub_img.publish(img_msg);
            cv::waitKey(1000);
        }

        // Display the original frame
        if(debug_mode){
            //cv::imshow("Original Frame", frame);
            //cv::waitKey(1000);
        }

        cv::Mat img_gray;
        cv::cvtColor(frame, img_gray, cv::COLOR_BGR2GRAY);
        if(debug_mode){
            //cv::imshow("Grayscale Frame", img_gray);
            //cv::waitKey(1000);
        }

        cv::Mat img_bilat;
        cv::bilateralFilter(img_gray, img_bilat, 11, 75, 75);
        if(debug_mode){
            //cv::imshow("Bilat Frame", img_bilat);
            //cv::waitKey(1000);
        }

        cv::Mat img_canny;
        cv::Canny(img_bilat, img_canny, 60, 60, 3);
        if(debug_mode){
            //cv::imshow("Canny Frame", img_canny);
            //cv::waitKey(0);
            //cv::destroyAllWindows();
        }

        int frame_width = frame.size().width;
        int frame_height = frame.size().height;

        if (debug_mode){
            ROS_INFO("Frame resolution: %dx%d", frame_width, frame_height);
        }

        
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img_canny, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        cv::Scalar color_left = cv::Scalar(0,0,255);      // Left  ==  Red
        cv::Scalar color_front = cv::Scalar(0,255,0);     // Front ==  Green
        cv::Scalar color_right = cv::Scalar(255,0,0);     // Right ==  Blue
        cv::Scalar color_back = cv::Scalar(0,127,255);    // Back  ==  Orange
        
        // Vectors are set up in the following order: {Left, Front, Right, Back}
        std::vector<double> distances(4, 300);
        std::vector<cv::Point> detected_points(4, cv::Point(0,0));

        double front_dist = frame_width/5;  // Thresh to ignore the robot front chassis
        double back_dist = frame_width/4;   // Thresh to ignore the robot back chassis
        double sensor_range = 30 * pi/180;  // Sensor range in radians

        for(size_t i = 0; i < contours.size(); i++){
            for(size_t j = 0; j < contours.at(i).size(); j++){
                int x_diff = contours.at(i).at(j).x - frame_width/2;
                int y_diff = contours.at(i).at(j).y - frame_height/2;
                double distance = cv::sqrt(x_diff*x_diff + y_diff*y_diff);
                double angle = acos((x_diff * -1 + y_diff * 0)/(distance * 1));
                
                if(y_diff > 0){angle = -angle;};

                // Check Sensors
                if(distance > front_dist){
                    int index = -1;
                    if(angle < (-1*pi/4 + sensor_range/2) && angle > (-1*pi/4 - sensor_range/2)){index = 0;}
                    else if(angle < (0 + sensor_range/2) && angle > (0 - sensor_range/2)){index = 1;}
                    else if(angle < (1*pi/4 + sensor_range/2) && angle > (1*pi/4 - sensor_range/2)){index = 2;}
                    else if(angle > (pi - sensor_range/2) || -angle > (pi - sensor_range/2)){
                      if(distance > back_dist){index = 3;}}

                    if (index == -1){continue;}
                    
                    if(distances.at(index) > distance){
                        distances.at(index) = distance;
                        detected_points.at(index) = contours.at(i).at(j);
                    };
                };
            }
        }
        

        if(debug_mode){
            cv::drawMarker(frame, detected_points.at(0), color_left, cv::MARKER_CROSS, 10, 2);
            std::cout << "Left sensor: " << distances.at(0) << " pixels." << std::endl;
            cv::drawMarker(frame, detected_points.at(1), color_front, cv::MARKER_CROSS, 10, 2);
            std::cout << "Front sensor: " << distances.at(1) << " pixels." << std::endl;
            cv::drawMarker(frame, detected_points.at(2), color_right, cv::MARKER_CROSS, 10, 2);
            std::cout << "Right sensor: " << distances.at(2) << " pixels." << std::endl;
            cv::drawMarker(frame, detected_points.at(3), color_back, cv::MARKER_CROSS, 10, 2);
            std::cout << "Back sensor: " << distances.at(3) << " pixels." << std::endl;
        }
        
        // Create VAR to Publish Sensor Values
        serp::ObjectDetection sensor_msg;
        sensor_msg.left = (uint8_t)(convert_Left(distances.at(0)));
        sensor_msg.front = (uint8_t)(convert_Front(distances.at(1)));
        sensor_msg.right = (uint8_t)(convert_Right(distances.at(2)));
        sensor_msg.back = (uint8_t)(convert_Back(distances.at(3)));


        // Publish Sensors Values
        sensor_pub.publish(sensor_msg);

        if(debug_mode){
          std::cout << "Sensor Front: " << (int)sensor_msg.front << " cm" << std::endl;
          std::cout << "Sensor Left: " << (int)sensor_msg.left << " cm" << std::endl;
          std::cout << "Sensor Right: " << (int)sensor_msg.right << " cm" << std::endl;
          std::cout << "Sensor Back: " << (int)sensor_msg.back << " cm" << std::endl;
        }


        if(debug_mode){
            //cv::drawMarker(frame, cv::Point(2,2), color, cv::MARKER_CROSS, 3);
            cv::namedWindow("Sensors", cv::WINDOW_NORMAL);
            cv::resizeWindow("Sensors", frame_width, frame_height);
            cv::imshow("Sensors", frame);
            cv::waitKey(50);
        }




        
        if(debug_mode){
            //cv::waitKey(0);
        }

        //ros::spinOnce();
        rate.sleep();

    }

    // When everything done, release the video capture object
    //Camera.release();
    //video.release();

    // Closes all the frames
    cv::destroyAllWindows();

    return 0;
}
