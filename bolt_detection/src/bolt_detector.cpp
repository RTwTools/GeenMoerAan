#include "bolt_detector.h"

bolt_detector::bolt_detector() :
  ph_("~"),
  viewCamera_(false),
  cameraId_(1),
  status_(false)
{
  ph_.param("view_camera", viewCamera_, viewCamera_);
  ph_.param("camera_ID", cameraId_, cameraId_);

  pubHoles_ = nh_.advertise<geometry_msgs::PoseArray>("/holes", 1);

  //only go to the next function if the previous one returned true
  status_ = OpenCamera();
  if (status_) status_ = ReadCalibrationData(CALIBRATION_FILE_NAME);
  if (status_) status_ = ReadTransformData(TRANSFORM_FILE_NAME);
  if (status_) CreateWindows();

  //TODO add timer to send holes
  //timer_ = nh_.createTimer(ros::Duration(1.0/rate_), boost::bind(&CruiseBehavior::update, this));
}

bool bolt_detector::ReadTransformData(std::string fileName)
{
  std::string pathname = ros::package::getPath(PACKAGE_NAME) + fileName;
  cv::FileStorage fs(pathname, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    ROS_ERROR("Failed to open [%s].", fileName.c_str());
    return false;
  }
  //TODO check if needed:
  //transformMatrix_ = cv::Mat(3,3,CV_32FC1);

  fs["transform_matrix"] >> transformMatrix_;

  ROS_INFO("Loaded transform data from [%s].", fileName.c_str());
  return true;
}

bool bolt_detector::ReadCalibrationData(std::string fileName)
{
  //load camera parameters from file
  std::string pathname = ros::package::getPath(PACKAGE_NAME) + fileName;
  //load camera parameters from file
  cv::FileStorage fs(pathname, cv::FileStorage::READ);
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

bool bolt_detector::GetStatus()
{
  return status_;
}

void bolt_detector::CreateWindows()
{
  if (viewCamera_)
  {
    cv::namedWindow("Camera", CV_WINDOW_NORMAL);
    cv::resizeWindow("Camera", 960, 540);
    cv::namedWindow("CameraCropped", CV_WINDOW_NORMAL);
    cv::resizeWindow("CameraCropped", 515, 540);

  }
  cv::namedWindow("DetectedHoles", CV_WINDOW_NORMAL);
  cv::resizeWindow("DetectedHoles", 515, 540);
}

bool bolt_detector::OpenCamera()
{
  camera_.open(cameraId_);
  if(!camera_.isOpened())
  {
    ROS_INFO("Could not open camera ID [%d].", cameraId_);
    return false;
  }

  //set resolution to FullHD
  camera_.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  camera_.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

  usedResolution_ = cv::Size((int) camera_.get(CV_CAP_PROP_FRAME_WIDTH),
                            (int) camera_.get(CV_CAP_PROP_FRAME_HEIGHT));

  ROS_INFO("Loaded camera ID [%i], size [%ix%i]", cameraId_, usedResolution_.width, usedResolution_.height);
  return true;
}

void bolt_detector::ShowWindows()
{
  if (viewCamera_)
  {
    cv::imshow("Camera", imageUndistorted_);
    cv::imshow("CameraCropped", imageCropped_);
  }
  cv::imshow("DetectedHoles", imageDetected_);
}

void bolt_detector::AddHole(int px_x, int px_y)
{
  float mm_x = (float) px_x * ((float)IMAGE_WIDTH_MM / IMAGE_WIDTH_PX);
  float mm_y = (float) px_y * ((float)IMAGE_HEIGHT_MM / IMAGE_HEIGHT_PX);

  geometry_msgs::Pose point;

  //orientation
  point.orientation.x = 0.70711;
  point.orientation.z = -0.70711;

  //position
  point.position.x = (mm_y / 1000);
  point.position.y = (mm_x / 1000);
  point.position.z = 0.01;

  ROS_DEBUG("Added hole: x[%f], y[%f].",  point.position.x,  point.position.y);

  holes_.poses.push_back(point);
}

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
  cv::undistort(image_, imageUndistorted_, cameraMatrix_, distCoeffs_);

  //transform image
  cv::Mat perspectiveImage = cv::Mat::zeros(imageUndistorted_.rows, imageUndistorted_.cols, imageUndistorted_.type());
  warpPerspective(imageUndistorted_, perspectiveImage, transformMatrix_, perspectiveImage.size());
  if (perspectiveImage.rows < IMAGE_WIDTH_PX || perspectiveImage.cols < IMAGE_HEIGHT_PX)
  {
    ROS_ERROR("Can't transform image, camera resolution is too small!");
    return false;
  }
  imageCropped_ = perspectiveImage(cv::Rect(0, 0, IMAGE_WIDTH_PX, IMAGE_HEIGHT_PX));
}

void bolt_detector::DetectHoles()
{
  if (!ProcessImage()) return;

  cv::Mat cannyOutput, imageGray;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::cvtColor(imageCropped_, imageGray, CV_BGR2GRAY);
  cv::Canny(imageGray, cannyOutput, CANNY_THRESH, CANNY_THRESH*3, 3);

  cv::findContours(cannyOutput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

  holes_.poses.clear();

  // Get the moments
  std::vector<cv::Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
  { mu[i] = moments( contours[i], false ); }

  ///  Get the mass centers:
  std::vector<cv::Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
  {
    mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
  }

  /// Draw contours
  imageDetected_ = cv::Mat::zeros( cannyOutput.size(), CV_8UC3 );

  for( int i = 0; i< contours.size(); i++ )
  {
    if (mu[i].m00 > 10)
    {
      //printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
      cv::Scalar color = cv::Scalar(0, 255, 255);
      cv::drawContours( imageDetected_, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
      cv::circle( imageDetected_, mc[i], 4, color, -1, 8, 0 );

      AddHole(mc[i].x, mc[i].y);
    }
  }

  ShowWindows();
}

void bolt_detector::SendHoles()
{
  holes_.header.frame_id = CAMERA_FRAME;
  holes_.header.stamp = ros::Time::now();
  pubHoles_.publish(holes_);

  ROS_INFO("Sent %i holes.", (int)holes_.poses.size());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PACKAGE_NAME");
  bolt_detector boltDetector;
  ros::Rate r(30);

  //check if boltDetector loaded correctly
  if (!boltDetector.GetStatus())
    return 1;

  ROS_INFO("Bolt Detection Node Started!");

  while(ros::ok() && boltDetector.GetStatus())
  {
    boltDetector.DetectHoles();

    if((cvWaitKey(40)&0xff)==ESC_KEY)
    {
      //save image
      //std::string pathname = ros::package::getPath("bolt_detection") + "/image.jpg";
      //imwrite(pathname, imageCropped);

      std::cout << "---------------------------------------------" << std::endl;
      cv::destroyWindow("Camera");
      cv::destroyWindow("CameraCropped");

      ros::shutdown();
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
