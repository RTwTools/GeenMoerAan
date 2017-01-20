#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <bolt_detection/BoltHoleInfo.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

#define IMAGE_WIDTH_PX    1030
#define IMAGE_WIDTH_MM     430
#define IMAGE_HEIGHT_PX   1080
#define IMAGE_HEIGHT_MM    450

#define NOT_FOUND       -1
#define CANNY_THRESH    80
#define ESC_KEY         27
#define PACKAGE_NAME "machine_learning"
#define CAMERA_FRAME "camera_view_link"
#define SAVE_OBJECT_FILE_NAME "/resources/out_save_object_data.yml"

Scalar COLOR_GREEN = Scalar(0, 255, 0);

struct Object_Data {
    vector<Point> contour;
    vector<Point2f> holes;
};

enum ShapeStatus {
  Different,
  Same,
  Turned
};

//functions
vector<Point> GetContourOfObject(Mat& object);
void SaveObjectsInFile(const string& filename , vector<Object_Data>& allobjects);
void GetAllDataOutFile(const string& filename , vector<Object_Data>& allobjects);
ShapeStatus IsSameShape(double difference);

void MouseCallback(int event, int x, int y, int flags, void* userdata);
int FindClosestHole(Point point,vector<Point2f> comparelist);
float DistanceBetweenPoints(Point2f p1, Point2f p2);

void MouseCB(int event, int x, int y, int flags, void* userdata);
void rosCallbackHoles(const bolt_detection::BoltHoleInfo boltdetectmsg);

void AddHole(int px_x, int px_y);
void SendHoles();

//parameters
ros::Publisher pubHoles_;
nav_msgs::Path holes_;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
vector<Object_Data> AllFoundObjectInFile;
vector<Point2f> receivedHoles;
bool gotHoles = false;
vector<Point2f> holesToSend;
bool reorderFinished;
Mat imageFromCamera;
Point MousePos;

vector<bool> selected;
vector<Point2f> holesOrdered;

int findSameShape(vector<Point> imageContour)
{
  for (int i=0;i<AllFoundObjectInFile.size(); i++)
  {
    double difference = matchShapes(imageContour, AllFoundObjectInFile[i].contour,1,0.0);
    ROS_INFO("ML: Shape difference = [%f]", difference);

    if (IsSameShape(difference) != Different)
      return i; //shape is known
  }
  return NOT_FOUND;
}

vector<Point2f> getRectArray(RotatedRect rect)
{
  Point2f rect_points[4];
  rect.points( rect_points );

  vector<Point2f> TempPoints;
  for( int j = 0; j < 4 ; j++ )
  {
    TempPoints.push_back(rect_points[j]);
  }
  return TempPoints;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, PACKAGE_NAME);
  ros::NodeHandle nh_;
  ros::Rate r(5);
  ros::Subscriber subHoles_ = nh_.subscribe("/holeData",1,&rosCallbackHoles);
  pubHoles_ = nh_.advertise<nav_msgs::Path>("/holes", 1);



  //create window to show image
  namedWindow("Camera_Image", CV_WINDOW_NORMAL);
  resizeWindow("Camera_Image", 515, 540);

  //read known shape data from file
  string pathname_savedata = ros::package::getPath(PACKAGE_NAME) + SAVE_OBJECT_FILE_NAME;
  FileStorage fs(pathname_savedata, FileStorage::READ);
  if (fs.isOpened())
    GetAllDataOutFile(pathname_savedata, AllFoundObjectInFile);

  while (ros::ok())
  {
    //wait for a message with holesInfo
    while(!gotHoles && ros::ok())
    {
      //exit program when user pressed ESC-key
      if((cvWaitKey(40)&0xff)==ESC_KEY)
      {
        ROS_INFO("---------------------------------------------");
        destroyAllWindows();
        return 0;
      }

      ros::spinOnce();
      r.sleep();
    }

    //get contour of received camera image
    vector<Point> cameraContour = GetContourOfObject(imageFromCamera);

    //check if shape is already known
    int index = findSameShape(cameraContour);
    if (index != NOT_FOUND)
    {
      ROS_INFO("ML: Found same shape at index [%i].", index);

      //get rectangle points
      RotatedRect foundShapeMinRect = minAreaRect(AllFoundObjectInFile[index].contour);
      RotatedRect cameraShapeMinRect = minAreaRect(cameraContour);
      vector<Point2f> foundShapeRectPoints = getRectArray(foundShapeMinRect);
      vector<Point2f> cameraShapeRectPoints = getRectArray(cameraShapeMinRect);

      //get rotation
      Mat warp_mat_inverse( 2, 3, CV_32FC1 );
      warp_mat_inverse = getPerspectiveTransform(foundShapeRectPoints, cameraShapeRectPoints);

      //transform holes
      vector<Point2f> transformedFileHoles;
      perspectiveTransform(AllFoundObjectInFile[index].holes, transformedFileHoles, warp_mat_inverse);

      //reorder holes
      holesToSend.clear();
      for (int i = 0; i < transformedFileHoles.size(); ++i)
      {
        int closestIndex = FindClosestHole(transformedFileHoles[i],receivedHoles);
        holesToSend.push_back(receivedHoles[closestIndex]);
      }
    }
    else
    {
      ROS_INFO("ML: Received new shape!");

      // get holes order from user
      reorderFinished = false;
      selected = vector<bool>(receivedHoles.size(), false);
      setMouseCallback("Camera_Image", MouseCB, NULL);

      imshow("Camera_Image", imageFromCamera);

      ROS_INFO("ML: Reording holes...");
      //wait for reording
      while (!reorderFinished && ros::ok()) { cvWaitKey(50); }
      setMouseCallback("Camera_Image", NULL, NULL);

      //Add shape to known shapes
      Object_Data newShape;
      newShape.contour = cameraContour;
      newShape.holes = holesToSend;
      AllFoundObjectInFile.push_back(newShape);
    }

    //show received image
    imshow("Camera_Image", imageFromCamera);

    //save known shapes in file
    SaveObjectsInFile(pathname_savedata, AllFoundObjectInFile);

    SendHoles();
    gotHoles = false;
  }
}

//send holes to the robot_mover
void SendHoles()
{
  holes_.header.frame_id = CAMERA_FRAME;
  holes_.header.stamp = ros::Time::now();
  holes_.poses.clear();

  //add each hole to a msg
  for (int i = 0; i < (int)holesToSend.size(); i++)
    AddHole(holesToSend[i].x, holesToSend[i].y);

  ROS_INFO("ML: Sent %d holes!", (int)holesToSend.size());
  pubHoles_.publish(holes_);
}

//add hole to the list of holes to send to the robot_mover
void AddHole(int px_x, int px_y)
{
    float mm_x = (float) px_x * ((float) IMAGE_WIDTH_MM / IMAGE_WIDTH_PX);
    float mm_y = (float) px_y * ((float) IMAGE_HEIGHT_MM / IMAGE_HEIGHT_PX);

    geometry_msgs::PoseStamped point;

    //orientation
    point.pose.orientation.x = 0.7071;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = -0.7071;
    point.pose.orientation.w = 0.0;
    //position
    point.pose.position.x = (mm_y / 1000);
    point.pose.position.y = (mm_x / 1000);
    point.pose.position.z = 0;

    point.header.frame_id = CAMERA_FRAME;

    ROS_DEBUG("ML: Added hole: x[%f], y[%f].", point.pose.position.x, point.pose.position.y);

    holes_.poses.push_back(point);
}

//Callback to receive holes from the bolt_detector
void rosCallbackHoles(const bolt_detection::BoltHoleInfo boltdetectmsg)
{
  if (!boltdetectmsg.path.poses.empty())
  {
    receivedHoles.clear();
    for (int i = 0; i < boltdetectmsg.path.poses.size(); i++)
      receivedHoles.push_back(Point2f(boltdetectmsg.path.poses[i].pose.position.x, boltdetectmsg.path.poses[i].pose.position.y));
    gotHoles = true;
  }
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(boltdetectmsg.image, sensor_msgs::image_encodings::BGR8);
  imageFromCamera = cv_ptr->image;
}

//calculate the distance between two points
float DistanceBetweenPoints(Point2f p1, Point2f p2)
{
  float x = p1.x - p2.x;
  float y = p1.y - p2.y;
  float distance = sqrt(pow(x,2) + pow(y,2));
  return distance;
}

//Handle user input to get an order for the holes
void MouseCB(int event, int x, int y, int flags, void* userdata)
{
  if ( event == EVENT_LBUTTONDOWN )
  {
    MousePos = Point(x,y);
    int holeIndex = FindClosestHole(MousePos,receivedHoles);
    if (!selected[holeIndex])
    {
      selected[holeIndex] = true;
      holesOrdered.push_back(receivedHoles[holeIndex]);
      putText(imageFromCamera, format("%d ",holesOrdered.size()),  Point(receivedHoles[holeIndex].x + 30 , receivedHoles[holeIndex].y - 30), FONT_HERSHEY_COMPLEX_SMALL, 3.0F, COLOR_GREEN, 2.0F);
      circle(imageFromCamera, receivedHoles[holeIndex], 4, COLOR_GREEN, 2);

      imshow("Camera_Image", imageFromCamera);
    }

    if (receivedHoles.size() == holesOrdered.size())
    {
      ROS_INFO("ML: Re-ordered holes...");
      holesToSend = holesOrdered;
      holesOrdered.clear();
      reorderFinished = true;
    }
  }
}

int FindClosestHole(Point point,vector<Point2f> comparelist)
{
  float smallestDist = 9999999;
  int index = 0;

  for( int i = 0; i < comparelist.size(); i++ )
  {
    float tempDist = DistanceBetweenPoints(point, comparelist[i]);

    if (tempDist <= smallestDist)
    {
      smallestDist = tempDist;
      index = i;
    }
  }
  return index;
}

vector<Point> GetContourOfObject(Mat& object)
{
  Mat cannyOutput, imageGray;
  cv::cvtColor(object, imageGray, CV_BGR2GRAY);
  Canny(imageGray, cannyOutput, CANNY_THRESH, CANNY_THRESH*3, 3);

  int morphological_smoothing =2;

  //morphological closing (removes small holes from the foreground)
  dilate(cannyOutput, cannyOutput, getStructuringElement(MORPH_ELLIPSE, Size(morphological_smoothing, morphological_smoothing)) ); 
  erode(cannyOutput, cannyOutput, getStructuringElement(MORPH_ELLIPSE, Size(morphological_smoothing, morphological_smoothing)) );
  
  findContours(cannyOutput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
  
  // Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
  {
    mu[i] = moments( contours[i], false );
  }

  ///  Get the mass centers:
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
  {
    mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
  }

  for( int i = 0; i< contours.size(); i++ )
  {
    if (mu[i].m00 > 10)
      return contours[i];
  }
}

//Save all the known shapes in a file
void SaveObjectsInFile(const string& filename , vector<Object_Data>& allobjects)
{
  FileStorage fs(filename, FileStorage::READ);
  fs.open(filename,FileStorage::WRITE);

  // WRITE ALL
  fs << "AllObjects" << "[";
  for( int i = 0; i < allobjects.size(); i++ )
    fs << "{:" << "Contour" << allobjects[i].contour << "Holes" << allobjects[i].holes << "}";
  fs << "]";

  fs.release();
}

//Read all the known shapes from a file
void GetAllDataOutFile(const string& filename , vector<Object_Data>& allobjects)
{
    FileStorage fs(filename, FileStorage::READ);
    if (fs.isOpened())
    {
      FileNode AllObjects = fs["AllObjects"];
      FileNodeIterator it = AllObjects.begin(), it_end = AllObjects.end();
      int idx = 0;

      // iterate through a sequence using FileNodeIterator
      for( ; it != it_end; ++it, idx++ )
      {
        Object_Data temp;
        (*it)["Contour"] >> temp.contour;
        (*it)["Holes"] >> temp.holes;
        allobjects.push_back(temp);
      } 
    }
    fs.release();
}

//check the value of the difference
ShapeStatus IsSameShape(double difference)
{
  if (difference == 0) //Same Shape, Same rotation
      return Same;
  else if (difference > 0.0 && difference < 0.015) //Same Shape, rotated
    return Turned;
  else //Not the same shape
    return Different;
}
