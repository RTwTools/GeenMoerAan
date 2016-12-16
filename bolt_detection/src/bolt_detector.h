#ifndef BOLTDETECTOR_H_
#define BOLTDETECTOR_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <math.h>

#define ESC_KEY             27
#define CANNY_THRESH        80

#define IMAGE_WIDTH_PX    1030
#define IMAGE_WIDTH_MM     430
#define IMAGE_HEIGHT_PX   1080
#define IMAGE_HEIGHT_MM    450

#define CALIBRATION_FILE_NAME "/resources/out_camera_data.yml"
#define TRANSFORM_FILE_NAME  "/resources/out_transform_data.yml"
#define CAMERA_FRAME "camera_view_link"
#define PACKAGE_NAME "bolt_detection"

class bolt_detector {
public:
    // members

    // methods
    bolt_detector();
    bool GetStatus();
    void DetectHoles();
    void SendHoles();

protected:
    // members
    ros::NodeHandle nh_, ph_;
    ros::Publisher pubHoles_;
    ros::Timer timer_;
    nav_msgs::Path holes_;

    int publish_rate_;
    int cameraId_;
    bool viewCamera_;
    bool status_;
    bool gui_;

    int iLowH = 70;
    int iHighH = 475;
    int iLowS = 0;
    int iHighS = 100;

    cv::VideoCapture camera_;
    cv::Mat cameraMatrix_, distCoeffs_;
    cv::Mat transformMatrix_;
    cv::Mat image_, imageUndistorted_, imageCropped_, imageDetected_;
    cv::Size usedResolution_, calibratedResolution_;

    // method
    bool OpenCamera();
    void CreateWindows();
    void ShowWindows();
    bool ReadCalibrationData(std::string fileName);
    bool ReadTransformData(std::string fileName);
    void AddHole(int px_x, int px_y);
    bool ProcessImage();
};


#endif
