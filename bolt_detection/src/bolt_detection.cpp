#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <math.h>

#define DEFAULT_CAMERA_ID		 1
#define ESC_KEY					27
#define CALIBRATION_FILE_PATH	"/resources/out_camera_data.yml"
#define TRANSFORM_FILE_PATH 	"/resources/out_transform_data.yml"
#define CAMERA_FRAME "camera_view" //TODO check name

#define CANNY_THRESH 80

#define IMAGE_WIDTH_PX  1030
#define IMAGE_WIDTH_MM   430
#define IMAGE_HEIGHT_PX 1080
#define IMAGE_HEIGHT_MM  450

//function declaration
bool openVideo(int stream, cv::VideoCapture * camera);
bool getFrame(cv::VideoCapture * camera, cv::Mat * image);
bool readCalibrationData();
bool readTransformData();

//object declaration
cv::VideoCapture camera;
cv::Mat cameraMatrix, distCoeffs;
cv::Mat transformMatrix = cv::Mat(3, 3, CV_32FC1);
cv::Mat image, imageCropped;
nav_msgs::Path path;
ros::Publisher pubHoles;

void addHole(int x, int y)
{
  float mm_x = (float) x * ((float)IMAGE_WIDTH_MM / IMAGE_WIDTH_PX);
  float mm_y = (float) y * ((float)IMAGE_HEIGHT_MM / IMAGE_HEIGHT_PX);

  geometry_msgs::PoseStamped point;

  point.header.stamp = ros::Time::now();
  point.header.frame_id = CAMERA_FRAME;
  point.pose.orientation.w = 1.0;
  point.pose.position.x = (mm_x / 1000);
  point.pose.position.y = (mm_y / 1000);

  ROS_DEBUG("Added hole: x[%f], y[%f].",  point.pose.position.x,  point.pose.position.y);

  path.poses.push_back(point);
}

cv::Mat detectHoles(cv::Mat * object)
{
  cv::Mat cannyOutput, imageGray;
  cv::vector<cv::vector<cv::Point> > contours;
  cv::vector<cv::Vec4i> hierarchy;

  cv::cvtColor(*object, imageGray, CV_BGR2GRAY);
  cv::Canny(imageGray, cannyOutput, CANNY_THRESH, CANNY_THRESH*3, 3);

  cv::findContours(cannyOutput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

  path.poses.clear();

  // Get the moments
  cv::vector<cv::Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
  { mu[i] = moments( contours[i], false ); }

  ///  Get the mass centers:
  cv::vector<cv::Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
  {
    mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    //filter circles

  }

  /// Draw contours
  cv::Mat drawing = cv::Mat::zeros( cannyOutput.size(), CV_8UC3 );

  for( int i = 0; i< contours.size(); i++ )
  {
    if (mu[i].m00 > 10)
    {
      //printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
      cv::Scalar color = cv::Scalar(0, 255, 255);
      cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
      cv::circle( drawing, mc[i], 4, color, -1, 8, 0 );

      addHole(mc[i].x, mc[i].y);
    }
  }
  return drawing;
}

void sendHoles(void)
{
  //send holes
  path.header.stamp = ros::Time::now();
  pubHoles.publish(path);

  ROS_DEBUG("Sent %i holes.", (int)path.poses.size());
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bolt_detection_node");
  ros::NodeHandle n;

  pubHoles = n.advertise<nav_msgs::Path>("/holes", 1);
  ros::Timer timer = n.createTimer(ros::Duration(2.0), boost::bind(&sendHoles));

  ROS_INFO("Bolt Detection Node Started!");

  ros::Rate r(30);

  //load camera
  int cameraId = (argc > 1) ? std::atoi(argv[1]) : DEFAULT_CAMERA_ID;
  if (!openVideo(cameraId, &camera))
    exit(1);

  //create window
  //cv::namedWindow("Camera", CV_WINDOW_NORMAL);
  //cv::resizeWindow("Camera", 960, 540);
  cv::namedWindow("CameraCropped", CV_WINDOW_NORMAL);
  cv::resizeWindow("CameraCropped", 515, 540);

  if(!readCalibrationData()) return 1;
  if(!readTransformData()) return 1;

  while(ros::ok() && getFrame(&camera, &image))
  {
    //undistort image
    cv::Mat imageU = image.clone();
    cv::undistort(image, imageU, cameraMatrix, distCoeffs);

    //transform image
    cv::Mat perspectiveImage = cv::Mat::zeros(imageU.rows, imageU.cols, imageU.type());
    warpPerspective(imageU, perspectiveImage, transformMatrix, perspectiveImage.size());

    imageCropped = perspectiveImage(cv::Rect(0, 0, IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX));

    //detect holes
    cv::Mat detected = detectHoles(&imageCropped);

    //cv::imshow("Camera", imageU);
    cv::imshow("CameraCropped", detected);

    if((cvWaitKey(40)&0xff)==ESC_KEY)
    {
      std::cout << "---------------------------------------------" << std::endl;
      cv::destroyWindow("Camera");
      cv::destroyWindow("CameraCropped");

      ros::shutdown();
    }

    ros::spinOnce();
    r.sleep();
  }
}

bool readCalibrationData()
{
  //load camera parameters from file
  std::string pathname = ros::package::getPath("bolt_detection") + CALIBRATION_FILE_PATH;
  cv::FileStorage fs(pathname, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cerr << "Failed to open " << CALIBRATION_FILE_PATH << std::endl;
    return false;
  }
  fs["camera_matrix"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;

  std::cout << "Loaded calibration data from [" << CALIBRATION_FILE_PATH << "]." << std::endl;
  return true;
}

bool readTransformData()
{
  std::string pathname = ros::package::getPath("bolt_detection") + TRANSFORM_FILE_PATH;
  cv::FileStorage fs(pathname, cv::FileStorage::READ);
  if (!fs.isOpened())
    {
        std::cerr << "Failed to open " << TRANSFORM_FILE_PATH << std::endl;
        return false;
    }
  fs["transform_matrix"] >> transformMatrix;

  std::cout << "Loaded transform data from [" << TRANSFORM_FILE_PATH << "]." << std::endl;
  return true;
}

//try to open an video with the given filename
bool openVideo(int stream, cv::VideoCapture * camera)
{
  camera->open(stream);
  if(!camera->isOpened())
  {
    std::cout << "Could not open camera ID [" << stream << "]." << std::endl;
    return false;
  }

  //optional: set resolution
  camera->set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  camera->set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

  cv::Size refS = cv::Size((int) camera->get(CV_CAP_PROP_FRAME_WIDTH),
  (int) camera->get(CV_CAP_PROP_FRAME_HEIGHT));
  std::cout << "Loaded camera ID [" << stream << "], size [" << refS.width << "x" << refS.height << "]" << std::endl;
  return true;
}

bool getFrame(cv::VideoCapture * camera, cv::Mat * image)
{
  camera->read(*image);
  return !image->empty();
}
