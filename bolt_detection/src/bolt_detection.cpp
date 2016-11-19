#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <math.h>

#define DEFAULT_CAMERA_ID   1
#define ESC_KEY     27
#define CALIBRATION_FILE_PATH "/resources/out_camera_data.yml"
#define TRANSFORM_FILE_PATH  "/resources/out_transform_data.yml"
#define CAMERA_FRAME "my_camera_view_link" //TODO check name

#define CANNY_THRESH 80

#define IMAGE_WIDTH_PX  1030
#define IMAGE_WIDTH_MM   430
#define IMAGE_HEIGHT_PX 1080
#define IMAGE_HEIGHT_MM  450

using namespace std;
using namespace cv;

//function declaration
bool openVideo(int stream, VideoCapture * camera);
bool getFrame(VideoCapture * camera, Mat * image);
bool readCalibrationData();
bool readTransformData();

//object declaration
VideoCapture camera;
Mat cameraMatrix, distCoeffs;
Mat transformMatrix = Mat(3, 3, CV_32FC1);
Mat image, imageCropped;
geometry_msgs::PoseArray holes;
ros::Publisher pubHoles;

void addHole(int x, int y) {
    float mm_x = (float) x * ((float) IMAGE_WIDTH_MM / IMAGE_WIDTH_PX);
    float mm_y = (float) y * ((float) IMAGE_HEIGHT_MM / IMAGE_HEIGHT_PX);

    geometry_msgs::Pose point;

    /*point.header.stamp = ros::Time::now();
    point.header.frame_id = CAMERA_FRAME;*/
    point.orientation.w = 1.0;
    point.position.x = (mm_y / 1000);
    point.position.y = (mm_x / 1000);

    ROS_DEBUG("Added hole: x[%f], y[%f].", point.position.x, point.position.y);

    holes.poses.push_back(point);
}

Mat detectHoles(Mat * object) {
    Mat cannyOutput, imageGray;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    cvtColor(*object, imageGray, CV_BGR2GRAY);
    Canny(imageGray, cannyOutput, CANNY_THRESH, CANNY_THRESH * 3, 3);


    //HoughCircles(cannyOutput, contours, CV_HOUGH_GRADIENT, 1, canny_i.rows/9, 105, 18, 0, 20 );
    findContours(cannyOutput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    holes.poses.clear();

    // Get the moments
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        mu[i] = moments(contours[i], false);
    }

    ///  Get the mass centers:
    vector<Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
        //filter circles
    }

    /// Draw contours
    Mat drawing = Mat::zeros(cannyOutput.size(), CV_8UC3);

    for (int i = 0; i < contours.size(); i++) {
        if (mu[i].m00 > 10) {
            printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength(contours[i], true));
            Scalar color = Scalar(0, 255, 255);
            drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
            circle(drawing, mc[i], 4, color, -1, 8, 0);

            addHole(mc[i].x, mc[i].y);
        }

        //        Point center(cvRound(contours[i][0]), cvRound(contours[i][1]));
        //        int radius = cvRound(contours[i][2]);
        //        // circle center
        //        circle(drawing, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        //        // circle outline
        //        circle(drawing, center, radius, Scalar(0, 0, 255), 3, 8, 0);


    }
    return drawing;
}

void sendHoles(void) {
    //send holes
    holes.header.stamp = ros::Time::now();
    pubHoles.publish(holes);

    ROS_DEBUG("Sent %i holes.", (int) holes.poses.size());
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "bolt_detection_node");
    ros::NodeHandle n;

    pubHoles = n.advertise<geometry_msgs::PoseArray>("/holes", 1);
    ros::Timer timer = n.createTimer(ros::Duration(2.0), boost::bind(&sendHoles));

    ROS_INFO("Bolt Detection Node Started!");

    ros::Rate r(30);

    //load camera
    int cameraId = (argc > 1) ? atoi(argv[1]) : DEFAULT_CAMERA_ID;
    if (!openVideo(cameraId, &camera))
        exit(1);

    //create window
    //namedWindow("Camera", CV_WINDOW_NORMAL);
    //resizeWindow("Camera", 960, 540);
    namedWindow("CameraCropped", CV_WINDOW_NORMAL);
    resizeWindow("CameraCropped", 515, 540);

    if (!readCalibrationData()) return 1;
    if (!readTransformData()) return 1;

    while (ros::ok() && getFrame(&camera, &image)) {
        //undistort image
        Mat imageU = image.clone();
        undistort(image, imageU, cameraMatrix, distCoeffs);

        //transform image
        Mat perspectiveImage = Mat::zeros(imageU.rows, imageU.cols, imageU.type());
        warpPerspective(imageU, perspectiveImage, transformMatrix, perspectiveImage.size());

        imageCropped = perspectiveImage(Rect(0, 0, IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX));

        //detect holes
        Mat detected = detectHoles(&imageCropped);

        //imshow("Camera", imageU);
        imshow("CameraCropped", detected);

        if ((cvWaitKey(40)&0xff) == ESC_KEY) {
            cout << "---------------------------------------------" << endl;
            destroyWindow("Camera");
            destroyWindow("CameraCropped");

            ros::shutdown();
        }

        ros::spinOnce();
        r.sleep();
    }
}

bool readCalibrationData() {
    //load camera parameters from file
    string pathname = ros::package::getPath("bolt_detection") + CALIBRATION_FILE_PATH;
    FileStorage fs(pathname, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Failed to open " << CALIBRATION_FILE_PATH << endl;
        return false;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    cout << "Loaded calibration data from [" << CALIBRATION_FILE_PATH << "]." << endl;
    return true;
}

bool readTransformData() {
    string pathname = ros::package::getPath("bolt_detection") + TRANSFORM_FILE_PATH;
    FileStorage fs(pathname, FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Failed to open " << TRANSFORM_FILE_PATH << endl;
        return false;
    }
    fs["transform_matrix"] >> transformMatrix;

    cout << "Loaded transform data from [" << TRANSFORM_FILE_PATH << "]." << endl;
    return true;
}

//try to open an video with the given filename

bool openVideo(int stream, VideoCapture * camera) {
    camera->open(stream);
    if (!camera->isOpened()) {
        cout << "Could not open camera ID [" << stream << "]." << endl;
        return false;
    }

    //optional: set resolution
    camera->set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    camera->set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

    Size refS = Size((int) camera->get(CV_CAP_PROP_FRAME_WIDTH),
            (int) camera->get(CV_CAP_PROP_FRAME_HEIGHT));
    cout << "Loaded camera ID [" << stream << "], size [" << refS.width << "x" << refS.height << "]" << endl;
    return true;
}

bool getFrame(VideoCapture * camera, Mat * image) {
    camera->read(*image);
    return !image->empty();
}
