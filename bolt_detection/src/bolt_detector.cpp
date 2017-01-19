#include "bolt_detector.h"

using namespace cv;
using namespace std;

bolt_detector::bolt_detector() :
  ph_("~"),
  viewCamera_(false),
  cameraId_(1),
  status_(false),
  gui_(false),
  viewSize_(false)
{
  frameCounter = 0;

  //parameter for calibrating the camera
  ph_.param("view_camera", viewCamera_, viewCamera_);
  //use the right camera ID, 1 with build in webcam, 0 without
  ph_.param("camera_id", cameraId_, cameraId_);
  //gui support boolean
  ph_.param("gui", gui_, gui_);
  //init publisher topic for Path msg
  pubHoles_ = nh_.advertise<nav_msgs::Path>("/holes", 1);

  //only go to the next function if the previous one returned true
  if (OpenCamera() && ReadCalibrationData(CALIBRATION_FILE_NAME) && ReadTransformData(TRANSFORM_FILE_NAME))
  {
    CreateWindows();
    status_ = true;
  }

  //if gui is set, subscribe GuiCB
  if (gui_)
    subGUI_ = nh_.subscribe<bolt_detection::Detection>("/detect_cmd", 1, &bolt_detector::GuiCB, this);
}

//Read transform data of Logitech C920 camera and return if success

bool bolt_detector::ReadTransformData(string fileName)
{
  string pathname = ros::package::getPath(PACKAGE_NAME) + fileName;
  FileStorage fs(pathname, FileStorage::READ);
  if (!fs.isOpened())
  {
    ROS_ERROR("Failed to open [%s].", fileName.c_str());
    return false;
  }
  fs["transform_matrix"] >> transformMatrix_;
  ROS_INFO("Loaded transform data from [%s].", fileName.c_str());
  return true;
}

//Read calibration data of Logitech C920 camera and return if success

bool bolt_detector::ReadCalibrationData(string fileName)
{
  //load camera parameters from file
  string pathname = ros::package::getPath(PACKAGE_NAME) + fileName;
  //load camera parameters from file
  FileStorage fs(pathname, FileStorage::READ);
  if (!fs.isOpened())
  {
    ROS_ERROR("Failed to open [%s].", fileName.c_str());
    return false;
  }
  fs["camera_matrix"] >> cameraMatrix_;
  fs["distortion_coefficients"] >> distCoeffs_;

  calibratedResolution_.width = fs["image_width"];
  calibratedResolution_.height = fs["image_height"];

  ROS_INFO("Loaded calibration data from [%s].", fileName.c_str());
  return true;
}

bool bolt_detector::ok()
{
  return status_;
}

//Create the windows and resize them

void bolt_detector::CreateWindows()
{
  if (viewCamera_)
  {
    //show unfiltered images to calibrate camera location
    namedWindow("Camera", CV_WINDOW_NORMAL);
    resizeWindow("Camera", 960, 540);
    namedWindow("CameraCropped", CV_WINDOW_NORMAL);
    resizeWindow("CameraCropped", 515, 540);
  }
  namedWindow("DetectedHoles", CV_WINDOW_NORMAL);
  resizeWindow("DetectedHoles", 515, 540);
}

//open camera with camera_id and return true if success

bool bolt_detector::OpenCamera()
{
  //    camera_.open(ros::package::getPath(PACKAGE_NAME) + "/src/my_video-4.avi");

  if (!camera_.open(cameraId_))
  {
    ROS_INFO("Could not open camera ID [%d].", cameraId_);
    return false;
  }

  //set resolution to FullHD
  camera_.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  camera_.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

  //set the used resolution size
  usedResolution_ = Size((int) camera_.get(CV_CAP_PROP_FRAME_WIDTH),
    (int) camera_.get(CV_CAP_PROP_FRAME_HEIGHT));

  ROS_INFO("Loaded camera ID [%i], size [%ix%i]", cameraId_, usedResolution_.width, usedResolution_.height);
  return true;
}

//Show images on created windows

void bolt_detector::ShowWindows()
{
  if (viewCamera_)
  {
    imshow("Camera", imageUndistorted_);
    imshow("CameraCropped", imageCropped_);
  }
  imshow("DetectedHoles", imageDetected_);
}

// Add position of holes from image to nav_msg path

void bolt_detector::AddHole(int px_x, int px_y)
{
  float mm_x = (float) px_x * ((float) IMAGE_WIDTH_MM / IMAGE_WIDTH_PX);
  float mm_y = (float) px_y * ((float) IMAGE_HEIGHT_MM / IMAGE_HEIGHT_PX);

  geometry_msgs::PoseStamped point;

  //orientation downwards
  point.pose.orientation.x = 0.7071;
  point.pose.orientation.y = 0.0;
  point.pose.orientation.z = -0.7071;
  point.pose.orientation.w = 0.0;
  //position
  point.pose.position.x = (mm_y / 1000);
  point.pose.position.y = (mm_x / 1000);
  point.pose.position.z = 0;

  point.header.frame_id = CAMERA_FRAME;

  ROS_DEBUG("Added hole: x[%f], y[%f].", point.pose.position.x, point.pose.position.y);

  holes_.poses.push_back(point);
}

//get new image then undistort and warp for detection use.

bool bolt_detector::ProcessImage()
{
  //get new image
  if (!camera_.read(image_))
  {
    status_ = false;
    ROS_ERROR("Couldn't get a new image from the camera!");
    return false;
  }

  //undistort image
  imageUndistorted_ = image_.clone();
  undistort(image_, imageUndistorted_, cameraMatrix_, distCoeffs_);

  //warp the perspective of the image
  Mat perspectiveImage = Mat::zeros(imageUndistorted_.rows, imageUndistorted_.cols, imageUndistorted_.type());
  warpPerspective(imageUndistorted_, perspectiveImage, transformMatrix_, perspectiveImage.size());
  if (perspectiveImage.rows < IMAGE_WIDTH_PX || perspectiveImage.cols < IMAGE_HEIGHT_PX)
  {
    ROS_ERROR("Can't transform image, camera resolution is too small!");
    status_ = false;
    return false;
  }
  imageCropped_ = perspectiveImage(Rect(0, 0, IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX));
  return true;
}

//filter the most significant object to reduce noise checks

bool bolt_detector::FilterObject()
{
  bool filtered = false;
  Mat cannyImage, grayImage, threshImage;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  cvtColor(imageCropped_, grayImage, CV_BGR2GRAY);

  //filter object from background
  threshold(grayImage, threshImage, 130, 255, 0);

  //find contours
  Canny(threshImage, cannyImage, CANNY_THRESH, CANNY_THRESH * 3, 3);
  findContours(cannyImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  /// Draw contours
  Mat mask = Mat::zeros(imageCropped_.size(), CV_8UC3);

  //for each contour check size
  for (int i = 0; i < contours.size(); i++)
  {
    //if size is big enough add to mask
    if (contourArea(contours[i]) > 50000)
    {
      drawContours(mask, contours, i, Scalar(255, 255, 255), -1, 8, hierarchy, 0, Point());
      filtered = true;
    }
  }
  //copy largest object to imageObject
  imageObject_ = Mat();
  imageCropped_.copyTo(imageObject_, mask);
  return filtered;
}

//

void bolt_detector::DetectHoles()
{
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  if (frameCounter < 5)
  {
    //if process and filter is a success, continue
    if (!ProcessImage()) return;
    FilterObject();
    //Actual filtering of images
    //cvt to gray
    cvtColor(imageObject_, imageGray, CV_BGR2GRAY);
    //blur image
    GaussianBlur(imageGray, imageGray, Size(3, 3), 1);
    //canny edge detection
    Canny(imageGray, cannyTemp, 70, 270, 3);
    int dilation_size = 13;
    Mat element = getStructuringElement(MORPH_ELLIPSE,
      Size(2 * dilation_size + 1, 2 * dilation_size + 1),
      Point(dilation_size, dilation_size));

    //opening morphology
    for (int i = 0; i < 1; i++)
    {
      dilate(cannyTemp, cannyTemp, element);
      erode(cannyTemp, cannyTemp, element);
    }

    //if frame is empty, initialize it
    if (frameCounter == 0)
      cannyOutput = Mat::zeros(cannyTemp.size(), CV_8U);
    //add up to 5 frames to cannyoutput
    cannyOutput += cannyTemp;
  }
  //if all frames are added
  if (frameCounter >= 4)
  {
    //reset framecounter
    frameCounter = -1;
    //find the contours of the sum of the frames
    findContours(cannyOutput, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<Point2f>center(contours.size());
    vector<float>radius(contours.size());
    vector<Rect> boundRect(contours.size());

    //aproximate the the contours to increase stability without loss of accuracy
    //use min enclosing circle to get circle
    for (int i = 0; i < contours.size(); i++)
    {
      approxPolyDP(Mat(contours[i]), contours[i], 3, true);
      boundRect[i] = boundingRect(Mat(contours[i]));
      minEnclosingCircle((Mat) contours[i], center[i], radius[i]);
    }
    //clear previous message
    holes_.poses.clear();

    //Draw contours and add holes to nav_msg if hole sizes are within certain parameters
    imageDetected_ = Mat::zeros(cannyOutput.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++)
    {
      if (radius[i] > 8 && radius[i] < 20)
      {
        Scalar color = Scalar(0, 255, 255);
        circle(imageDetected_, center[i], (int) radius[i], color, 2, 8, 0);
        circle(imageDetected_, center[i], 1, color, 2, 8, 0);
        AddHole(center[i].x, center[i].y);
        if (viewSize_)
        {
          stringstream ss;
          ss << setprecision(4) << (radius[i]*2);
          Point2f p = center[i];
          p.x += radius[i];
          putText(imageDetected_, ss.str(), p,
            FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 0), 2);
        }
      }
    }

    //display holes
    ShowWindows();
  }
  //finally increment framecounter
  frameCounter++;
}

//Publish the current nav_msg to the topic

void bolt_detector::SendHoles()
{
  holes_.header.frame_id = CAMERA_FRAME;
  holes_.header.stamp = ros::Time::now();
  pubHoles_.publish(holes_);

  //save image  
  string pathname = ros::package::getPath("bolt_detection") + "/resources/image.png";
  imwrite(pathname, imageCropped_);

  ROS_INFO("Sent %i holes.", (int) holes_.poses.size());
}

//callback for GUI support

void bolt_detector::GuiCB(const bolt_detection::Detection::ConstPtr &gui_msg)
{
  if (!gui_msg->detect) return;
  SendHoles();
}

int main(int argc, char **argv)
{
  //Check openCV version
  cout << "OpenCV version : " << CV_VERSION << endl;
  ros::init(argc, argv, PACKAGE_NAME);
  bolt_detector boltDetector;
  ros::Rate r(50);

  //check if boltDetector loaded correctly
  if (!boltDetector.ok())
    return -1;

  ROS_INFO("Bolt Detection Node Started!");

  //while ros and boltdetection are ok
  while (ros::ok() && boltDetector.ok())
  {
    //detect the holes
    boltDetector.DetectHoles();
    char c = (char) cvWaitKey(1);

    //if space is pressed send holes & gui isn't set
    if (c == SPACE_KEY && !boltDetector.gui_)
      boltDetector.SendHoles();    
    //if space is pressed send holes & gui is set
    if (c == SPACE_KEY && boltDetector.gui_)
      boltDetector.viewSize_= !boltDetector.viewSize_;

    //close program if escape is pressed
    if (c == ESC_KEY)
    {
      cout << "---------------------------------------------" << endl;
      destroyWindow("Camera");
      destroyWindow("CameraCropped");
      ros::shutdown();
      break;
    }
    //spin and sleep
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

