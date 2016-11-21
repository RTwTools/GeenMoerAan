#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>

#define DEFAULT_CAMERA_ID		 0
#define ESC_KEY					27
#define CALIBRATION_FILE_NAME	"out_camera_data.yml"
#define HSV_FILE_NAME 			"out_hsv_data.yml"

#define MAX_HUE					179
#define MAX_SATURATION			255
#define MAX_VALUE				255

//function declaration
bool openVideo(int stream, cv::VideoCapture * camera);
bool getFrame(cv::VideoCapture * camera, cv::Mat * image);
bool readCalibrationValues();
void createTrackBars();
void writeHSVValues();
void readHSVValues();

//object declaration
cv::VideoCapture camera;
cv::Mat image, imageHSV, thresholdHSV;
cv::Mat cameraMatrix, distCoeffs;

//trackbar values
int minHueSlider =			  0;
int maxHueSlider =			179;
int minSaturationSlider =	  0;
int maxSaturationSlider =	255;
int minValueSlider =		  0;
int maxValueSlider =		255;

int main(int argc, char *argv[])
{

	//load camera
	int cameraId = (argc > 1) ? std::atoi(argv[1]) : DEFAULT_CAMERA_ID;
	if (!openVideo(cameraId, &camera))
		exit(1);

	//create windows
	cv::namedWindow("Camera", CV_WINDOW_NORMAL);
	cv::resizeWindow("Camera", 960, 540);
	cv::namedWindow("Sliders", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("FilteredHSV", CV_WINDOW_NORMAL);
	cv::resizeWindow("FilteredHSV", 960, 540);
	
	if(!readCalibrationValues()) return 1;
	
	readHSVValues();
	createTrackBars();
	
	//select HSV parameters
	while(getFrame(&camera, &image))
	{
		//undistort image
		cv::Mat imageU = image.clone();
        cv::undistort(image, imageU, cameraMatrix, distCoeffs);
		
		//Convert to HSV colorSpectrum
		cv::cvtColor(imageU, imageHSV, CV_BGR2HSV);
		
		cv::inRange(imageHSV, cv::Scalar(minHueSlider, minSaturationSlider, minValueSlider), cv::Scalar(maxHueSlider, maxSaturationSlider, maxValueSlider), thresholdHSV);

		cv::imshow("Camera", image);
		cv::imshow("FilteredHSV", thresholdHSV);		
	
		if((cvWaitKey(40)&0xff)==ESC_KEY)
		{
			writeHSVValues();
			cv::destroyWindow("Sliders");
			cv::destroyWindow("FilteredHSV");
			
			std::cout << "Using selected HSV values." << std::endl;
			break;
		}
	}
	
	/*cv::namedWindow("UndistortedImage", CV_WINDOW_NORMAL);
	cv::resizeWindow("UndistortedImage", 960, 540);*/
	
	while(getFrame(&camera, &image))
	{
		//undistort image
		cv::Mat imageU = image.clone();
        cv::undistort(image, imageU, cameraMatrix, distCoeffs);
        
        //filter noise
        cv::medianBlur(imageU, imageU, 3);
        
        //filter object
        cv::cvtColor(imageU, imageHSV, CV_BGR2HSV);
		cv::inRange(imageHSV, cv::Scalar(minHueSlider, minSaturationSlider, minValueSlider), cv::Scalar(maxHueSlider, maxSaturationSlider, maxValueSlider), thresholdHSV);
		
		//morphological opening (remove small objects from the foreground)
		cv::erode(thresholdHSV, thresholdHSV, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
		cv::dilate(thresholdHSV, thresholdHSV, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

		//morphological closing (fill small holes in the foreground)
		cv::dilate(thresholdHSV, thresholdHSV, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
		cv::erode(thresholdHSV, thresholdHSV, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
		
		//find contours
		cv::vector<cv::vector<cv::Point> > contours;
		cv::vector<cv::Vec4i> hierarchy;
		
		cv::findContours( thresholdHSV, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
		
		cv::vector<cv::vector<cv::Point> > contours_poly( contours.size() );
		cv::vector<cv::Rect> boundRect( contours.size() );
		
		for(int i=0; i < contours.size(); i++)
		{
			cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
			boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
		}
		
		/// Draw polygonal contour + bonding rects
		for( int i = 0; i< contours.size(); i++ )
		{
			/*if (cv::contourArea(contours[i]) > 1000)
			{*/
				//printf(" * Contour[%d] - Area: %.2f - Length: %.2f \n", i, cv::contourArea(contours[i]), cv::arcLength( contours[i], true ) );
				cv::Scalar color = cv::Scalar(0, 0, 255);
				//cv::drawContours( imageU, contours_poly, i, color, 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point() );
				cv::rectangle( imageU, boundRect[i].tl(), boundRect[i].br(), color, 3, 8, 0 );
				
				//calculate distance // size of object
				float distance_mm = 500;
				float focalLength_mm = 3.67;
				float focalLength_px = 1392;
				float imageSensor_pxmm = focalLength_px / focalLength_mm;
				float objectWidth_px = boundRect[i].width;
				float objectWidth_Sensor = objectWidth_px / imageSensor_pxmm;
				float objectWidth_mm = (distance_mm * objectWidth_Sensor) /focalLength_mm;
				
				std::cout << "contourArea: [" << cv::contourArea(contours[i]) <<  "] Object_Width [" << objectWidth_mm << "] mm." << std::endl;
			//}
       
     	}
     	std::cout << std::endl;

		
		/*//calculate moments
		cv::Moments myMoments = moments(thresholdHSV);
		
		double x = myMoments.m01;
		double y = myMoments.m10;
		double area = myMoments.m00;
        
        //boundingbox arround object*/
        
        imshow("Camera", imageU);
		//imshow("UndistortedImage", thresholdHSV);
		
		if((cvWaitKey(40)&0xff)==ESC_KEY)
		{
			std::cout << "---------------------------------------------" << std::endl;
			break;
		}
	}
	return 0;
}
	
void createTrackBars()
{
	cv::createTrackbar("hueMin", "Sliders", &minHueSlider, MAX_HUE);
	cv::createTrackbar("hueMax", "Sliders", &maxHueSlider, MAX_HUE);
	
	cv::createTrackbar("saturationMin", "Sliders", &minSaturationSlider, MAX_SATURATION);
	cv::createTrackbar("saturationMax", "Sliders", &maxSaturationSlider, MAX_SATURATION);
	
	cv::createTrackbar("valueMin", "Sliders", &minValueSlider, MAX_VALUE);
	cv::createTrackbar("valueMax", "Sliders", &maxValueSlider, MAX_VALUE);
}

bool readCalibrationValues()
{
	//load camera parameters from file
	cv::FileStorage fs(CALIBRATION_FILE_NAME, cv::FileStorage::READ);
	if (!fs.isOpened())
    {
        std::cerr << "Failed to open " << CALIBRATION_FILE_NAME << std::endl;
        return false;
    }
	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	
	std::cout << "Loaded calibration data from [" << CALIBRATION_FILE_NAME << "]." << std::endl;
	return true;
}

void readHSVValues()
{
	cv::FileStorage fs(HSV_FILE_NAME , cv::FileStorage::READ);
	if (!fs.isOpened())
    {
        std::cerr << "Failed to open " << HSV_FILE_NAME << std::endl;
        std::cerr << "Starting with default values." << std::endl;
        return;
    }  
    //cv::FileNode fn = fs["minHue"];
    
    minHueSlider = fs["minHue"];
    maxHueSlider = fs["maxHue"];
    
    minSaturationSlider = fs["minSaturation"];
    maxSaturationSlider = fs["maxSaturation"];
    
    minValueSlider = fs["minValue"];
    maxValueSlider = fs["maxValue"];
    
    fs.release();
    std::cout << "Loaded HSV values from [" << HSV_FILE_NAME << "]." << std::endl;
}

void writeHSVValues()
{
	cv::FileStorage fs(HSV_FILE_NAME , cv::FileStorage::WRITE);
	if (!fs.isOpened())
    {
        std::cerr << "Failed to write to " << HSV_FILE_NAME << std::endl;
        return;
    }
    
    fs << "minHue" << minHueSlider;
    fs << "maxHue" << maxHueSlider;
    
    fs << "minSaturation" << minSaturationSlider;
    fs << "maxSaturation" << maxSaturationSlider;
    
    fs << "minValue" << minValueSlider;
    fs << "maxValue" << maxValueSlider;
    
    fs.release();
    
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
