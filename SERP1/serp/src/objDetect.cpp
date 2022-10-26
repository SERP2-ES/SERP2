#include "objDetect.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "sensor_node");
    ros::NodeHandle n_public;

	raspicam::RaspiCam_Cv Camera;
	//open camera
  	std::cout<<"Opening Camera..."<<std::endl;
    	if (!Camera.open()) {std::cerr<<"Error opening the camera"<<std::endl;return -1;}
    	ros::Duration(1.0).sleep();

    string videoPath = "C:\\Users\\nuno-\\BrunerZ\\FEUP\\1º Semestre\\ES\\virtualSensor\\object_detection.h264";

    // Create a VideoCapture object and use camera to capture the video
    VideoCapture cap(videoPath);

    // Check if camera opened successfully
    if (!cap.isOpened()) {
        cout << "Error opening video stream" << endl;
        return -1;
    }

    // Default resolutions of the frame are obtained.The default resolutions are system dependent.
    int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
    VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(frame_width, frame_height));


    while (1) {

        Mat frame;

        int width = frame.cols;
        int height = frame.rows;

        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
            break;

        frame = frameProcessing(frame);

        // Write the frame into the file 'outcpp.avi'
        video.write(frame);
        waitKey(0);
        

        // Display the resulting frame    
        imshow("Frame", frame);
        //waitKey(0); 

        // Press  ESC on keyboard to  exit
        char c = (char)waitKey(1);
        if (c == 27)
            break;
    }

    // When everything done, release the video capture and write object
    cap.release();
    video.release();

    // Closes all the frames
    destroyAllWindows();
    return 0;
}
