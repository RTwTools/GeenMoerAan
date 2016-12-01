#include "bolt_detector.h"

bolt_detector::bolt_detector() :
  ph_("~"),
  viewCamera_(false),
  cameraID_(1),
  initStatus_(false)
{
  ph_.param("view_camera", viewCamera_, viewCamera_);
  ph_.param("camera_ID", cameraID_, cameraID_);

  pubHoles_ = nh_.advertise<geometry_msgs::PoseArray>("/holes", 1);

  //only go to the next function if the previous one returned true
  initStatus_ = OpenCamera();
  if (initStatus_) initStatus_ = ReadCalibrationData(CALIBRATION_FILE_NAME);
  if (initStatus_) initStatus_ = ReadTransformData(TRANSFORM_FILE_NAME);
  if (initStatus_) CreateWindows();
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
  return initStatus_;
}

void bolt_detector::CreateWindows()
{
  if (viewCamera_)
  {
    cv::namedWindow("Camera", CV_WINDOW_NORMAL);
    cv::resizeWindow("Camera", 960, 540);
  }
  cv::namedWindow("CameraCropped", CV_WINDOW_NORMAL);
  cv::resizeWindow("CameraCropped", 515, 540);
}

bool bolt_detector::OpenCamera()
{
  camera_.open(cameraID_);
  if(!camera_.isOpened())
  {
    ROS_INFO("Could not open camera ID [%d].", cameraID_);
    return false;
  }

  //set resolution to FullHD
  camera_.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  camera_.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

  usedResolution_ = cv::Size((int) camera_.get(CV_CAP_PROP_FRAME_WIDTH),
                            (int) camera_.get(CV_CAP_PROP_FRAME_HEIGHT));

  ROS_INFO("Loaded camera ID [%i], size [%ix%i]", cameraID_, usedResolution_.width, usedResolution_.height);
  return true;
}

void bolt_detector::ShowWindows()
{
  if (viewCamera_)
  {
    //TODO
  }
}

void bolt_detector::DetectHoles()
{
  //TODO
}

void bolt_detector::SendHoles()
{
  //TODO
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

  while(ros::ok())
  {
    boltDetector.DetectHoles();

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
