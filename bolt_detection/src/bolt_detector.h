#ifndef BOLTDETECTOR_H_
#define BOLTDETECTOR_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/package.h>
#include <geometry_msgs/PoseArray.h>
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

#define CALIBRATION_FILE_NAME	"/resources/out_camera_data.yml"
#define TRANSFORM_FILE_NAME 	"/resources/out_transform_data.yml"
#define CAMERA_FRAME "camera_view_link"
#define PACKAGE_NAME "bolt_detection"

class BoltDetector
{
public:
  // members

  // methods
  BoltDetector();
  bool GetStatus();

protected:
  // members
  ros::NodeHandle nh_, ph_;
  ros::Publisher pubHoles_;
  geometry_msgs::PoseArray holes_;

  int rate_;
  int cameraID_;
  bool viewCamera_;
  bool initStatus_;

  cv::VideoCapture camera_;
  cv::Mat cameraMatrix_, distCoeffs_;
  cv::Mat transformMatrix_;
  cv::Mat image_, imageCropped_;
  cv::Size usedResolution_, calibratedResolution_;

  // method
  bool OpenCamera();
  void CreateWindows();
  bool ReadCalibrationData(std::string fileName);
  bool ReadTransformData(std::string fileName);
};


#endif
