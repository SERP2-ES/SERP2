#include "rpi_camera.h"
using std::string;

int ready;
cv::Mat final;
cv::Mat frame_sensors;





std::vector<uint8_t> sensor_valores;

bool cb_read_programming_sheet(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    camera_state = ReadProgrammingSheet;
    res.success = true;
    return true;
}

void cb_robot_state(const std_msgs::StringConstPtr &state) {
    if(state->data == "Stopped" || state->data == "ManualControl") camera_state = NormalOperation;
    else if(state->data == "Executing") camera_state = DetectObstacles;
}

string find_sensorIMG_path(char sensor , int valor_sensor){
    string path = "/home/pi/catkin_ws/src/serp/include/images/";

    switch (sensor)
    {
    case 'f': //frente
        path.append("frente");
        break;

    case 't': //trás
        path.append("tras");
        break;

    case 'd': //direita
        path.append("direita");
        break;

    case 'e': //esquerda
        path.append("esquerda");
        break;
    }

    switch (valor_sensor)
    {
    case 1:
        path.append("1.png");
        break;
    case 2:
        path.append("2.png");
        break;
    case 3:
        path.append("3.png");
        break;
    case 4:
        path.append("4.png");
        break;

    }
    return path;
}


cv::Mat place_sensor(cv::Mat frame, cv::Mat sensor, int position_x, int position_y){

    cv::Mat mask;
    cv::Mat rgbLayer[4];
    cv::split(sensor,rgbLayer);

    if(sensor.channels() == 4)
    {
        split(sensor,rgbLayer);         // seperate channels
        cv::Mat cs[3] = { rgbLayer[0],rgbLayer[1],rgbLayer[2] };
        merge(cs,3,sensor);  // glue together again
        mask = rgbLayer[3];       // png's alpha channel used as mask
    }

    // Get the destination ROI (and make sure it is within the image)
    cv::Rect dstRC = cv::Rect(position_x, position_y, sensor.size().width, sensor.size().height);
    cv::Mat dstROI = frame(dstRC);
    // Copy the pixels from src to dst.
    sensor.copyTo(dstROI,mask);

    return frame;
}

cv::Mat process_frame(cv::Mat frame, std::vector<uint8_t> sensor_valores)
{

    for(;;)
    {

        // Import sensor images
        cv::Mat sensor_front = cv::imread(find_sensorIMG_path('f',sensor_valores[1]+1), cv::IMREAD_UNCHANGED);
        cv::Mat sensor_back = cv::imread(find_sensorIMG_path('t',sensor_valores[2]+1), cv::IMREAD_UNCHANGED);
        cv::Mat sensor_left = cv::imread(find_sensorIMG_path('e',sensor_valores[3]+1), cv::IMREAD_UNCHANGED);
        cv::Mat sensor_right = cv::imread(find_sensorIMG_path('d',sensor_valores[0]+1), cv::IMREAD_UNCHANGED);


        //Resize sensor images
        cv::resize(sensor_front,sensor_front,cv::Size(60,37));
        cv::resize(sensor_back,sensor_back,cv::Size(60,37));
        cv::resize(sensor_left,sensor_left,cv::Size(60,71));
        cv::resize(sensor_right,sensor_right,cv::Size(60,71));


        int cx = (frame.cols - sensor_front.size().width) / 2;
        if (!sensor_front.empty()) {
            frame = place_sensor(frame, sensor_front, cx, frame.rows/7);
        }
        if (!sensor_back.empty()) {
            frame = place_sensor(frame, sensor_back, cx, 5.2*frame.rows/7);

        }
        if (!sensor_left.empty()) {
            frame = place_sensor(frame, sensor_left, 7*cx/12, frame.rows/6);
        }
        if (!sensor_right.empty()) {
            frame = place_sensor(frame, sensor_right, 17*cx/12, frame.rows/6);
        }
        return frame;
    }

}


int main(int argc, char **argv)
{


    
    int count=0, j=0;
    ros::init(argc, argv, "rpi_camera_node");
    ros::NodeHandle n_public;

    cv::Mat copy;

    // Subscriber for Robot State
    ros::Subscriber sub_robot_state = n_public.subscribe("robot_state", 2, cb_robot_state);
    camera_state=NormalOperation;

    // Initialize variable(s)

    cv_bridge::CvImage img_bridge;
    //aruco dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    int angle=90;
    uint8_t pleft;
    uint8_t pright;
    uint8_t pback;
    uint8_t pfront;
    int matrix_size=100;
    int matrix_links[100][100]={0};
    float matrix_values[100][100]={0};


    // Matrixes publisher
    serp::Matrix data;
    ros::Publisher send_matrix = n_public.advertise<serp::Matrix>("send_matrix", 1);


    // Image publisher
    image_transport::ImageTransport it(n_public);
    image_transport::Publisher pub_img = it.advertise("camera", 1);
    image_transport::Publisher pub_last_detected_sheet = it.advertise("camera/sheet_detections", 1); //usar para mandar frame

    // Create ROS Servers
    ros::ServiceServer service_read_programming_sheet = n_public.advertiseService("srv_read_programming_sheet", cb_read_programming_sheet);

    raspicam::RaspiCam_Cv Camera;
    Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    Camera.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);

    std::cout << "Opening Camera..." << std::endl;
    if (!Camera.open())
    {
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }
    ros::Duration(1.0).sleep();
    cv::Mat frame;

    while (ros::ok())
    {

        // Get new frame
        Camera.grab();
        Camera.retrieve(frame);


        cv::Mat cropped_image = frame(cv::Rect(100, 0, 1300, 1200));

        // Resize image to 800x450 (to publish to the GUI)
        cv::Mat resized_frame;
        cv::resize(cropped_image, resized_frame, cv::Size(500, 462));


        //Rotate frame 90º
        cv::rotate(resized_frame, resized_frame, cv::ROTATE_90_CLOCKWISE);

        if(camera_state == DetectObstacles)
        {
            //get sensor values
            sensor_valores = frameProcessing(frame);
            if(count==1)
            {
                //publish image
                // Convert OpenCV image to ROS data
                sensor_msgs::Image img_msg;
                std_msgs::Header header;
                header.stamp = ros::Time::now();

                //overlay sensors
                resized_frame=process_frame(resized_frame, sensor_valores);


                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, resized_frame);
                img_bridge.toImageMsg(img_msg);

                pub_img.publish(img_msg);
            }
            ROS_INFO("LEFT: %d", sensor_valores[3]);
            ROS_INFO("RIGHT: %d", sensor_valores[0]);
            ROS_INFO("FRONT: %d", sensor_valores[1]);
            ROS_INFO("BACK: %d", sensor_valores[2]);
            ROS_INFO("--------------------------");


            //fill matrix_values with sensor values
            for(int i=0; i < matrix_size; i++)
            {
                //if left sensor is connected to any block
                if(matrix_links[i][2] == 1)
                {
                    matrix_values[i][2] = sensor_valores[3];
                    matrix_values[2][i] = sensor_valores[3];
                }

                //if front sensor is connected to any block
                if(matrix_links[i][3] == 1)
                {
                    matrix_values[i][3] = sensor_valores[1];
                    matrix_values[3][i] = sensor_valores[1];
                }

                //if right sensor is connected to any block
                if(matrix_links[i][4] == 1)
                {
                    matrix_values[i][4] = sensor_valores[0];
                    matrix_values[4][i] = sensor_valores[0];
                }

                //if back sensor is connected to any block
                if(matrix_links[i][5] == 1)
                {
                    matrix_values[i][5] = sensor_valores[2];
                    matrix_values[5][i] = sensor_valores[2];
                }
            }

            //publish matrixes
            for (int i=0; i < matrix_size; i++)
            {
                for (int j=0 ; j< matrix_size ; j++)
                {
                    data.matrix_links.push_back(matrix_links[i][j]);
                    data.matrix_values.push_back(matrix_values[i][j]);
                }
            }
            send_matrix.publish(data);
        }
        else if(camera_state == ReadProgrammingSheet)
        {
            if(count==1)
            {
                //publish image
                // Convert OpenCV image to ROS data
                sensor_msgs::Image img_msg;
                std_msgs::Header header;
                header.stamp = ros::Time::now();
                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, resized_frame);
                img_bridge.toImageMsg(img_msg);

                pub_img.publish(img_msg);
            }
            ready=detectAndInterpret_Paper(frame, dictionary, matrix_links, matrix_values, ready);
            if(ready == 1)
            {
                // Convert OpenCV image to ROS data
                sensor_msgs::Image img_msg;
                std_msgs::Header header;
                header.stamp = ros::Time::now();
                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
                img_bridge.toImageMsg(img_msg);
                //publish image to interface
                pub_last_detected_sheet.publish(img_msg);

                //return to normal operation state until user decides
                camera_state = NormalOperation;
                ready=0;

            }
        }
        else {
            if(count==1)
            {
                //publish image
                // Convert OpenCV image to ROS data
                sensor_msgs::Image img_msg;
                std_msgs::Header header;
                header.stamp = ros::Time::now();
                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, resized_frame);
                img_bridge.toImageMsg(img_msg);

                pub_img.publish(img_msg);
            }
        }

        ros::spinOnce();
        count++;
        if (count==2)
            count=0;
    }
    Camera.release();
    cv::destroyAllWindows();
    return 0;
}
