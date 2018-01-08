/**
* @ file Volume_Counter.cpp
* @ counts objects by volume estimation
* @ method - first the algorithm finds the planes of the shelf and the wall, which represent the height direction and the length direction.
* @			 The width direction can be calculated from their cross product. We use those vectors to create a box around the relevant products, starting from the left separator.
* @			 The count is done 1 line at a time, and does not take in calculation any missing object in the middle of the line.
* @ arguments: depth image 
* @ output: number of products
*/
//#include "counter.h"
#include "interactive_counter.h"
#include "counterUtils.h"
// #include "opencv2/imgproc.hpp"  
// #include "opencv2/imgcodecs.hpp"  
// #include "opencv2/highgui.hpp" 
// #include <librealsense2/rs.hpp> 
// #include <iostream> 
// #include <stdio.h>
// #include <math.h>
#include "Volume_Counter.h"

//double countObjects(int objectType);
void createObjectsTypeList();
void createPopupImages();
void showResultImg(double count);
static void onMouse(int event, int x, int y, int flags, void* param);
static void onBtnClick(int event, int x, int y, int flags, void* param);

////////////////TODO: delete///////////////////
counter c;
std::vector<objectType> objTypes;
std::vector<cv::Rect> objButtons;
int objID = -1;
cv::Mat btnsImg;
const int buttonWidth = 100;
const int buttonHeight = 50;
bool capture = true;
cv::Mat src;
///////////////////////////////////////////////

int main(int argc, char** argv){

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH, -1, 640, 480, RS2_FORMAT_Z16, 30);
	rs2::frame depth;
	cv::Mat depthMat, HeatMap;
	int w = 0, h = 0;
	createObjectsTypeList();
	createPopupImages();

	while(true)
	{
		capture = true;
		// Start streaming with default recommended configuration
		pipe.start(cfg);
		// Create a window for the stream
		cv::namedWindow("yahav", cv::WINDOW_AUTOSIZE);

		while(capture)
		{
			rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
			depth = data.get_depth_frame(); // Find the depth data

			if(depth != NULL)// Render depth heatmap and choose the separator with the mouse
			{
				w = depth.as<rs2::video_frame>().get_width();
				h = depth.as<rs2::video_frame>().get_height();
				depthMat = cv::Mat(cv::Size(640,480),CV_16UC1,(void*)depth.get_data(),cv::Mat::AUTO_STEP);

				src = depthMat;
				HeatMap = counterUtils::getColorCodedDepth(depthMat, 200, 4000);
				setMouseCallback("yahav", onMouse, &HeatMap);
				
				imshow("yahav", HeatMap);
				char key = cv::waitKey(50);
				if(key == 27) //escape
					return EXIT_SUCCESS;
			}
		} 
		// Stop streaming
		pipe.stop();
		c.setImg(depthMat);
		std::string btnsWinName = "choose an object type";
		cv::namedWindow(btnsWinName);
		cv::setMouseCallback(btnsWinName, onBtnClick);
		objID = -1;
		cv::imshow(btnsWinName, btnsImg);
		while(objID == -1)
		{
			cv::waitKey(20);
		}
		cv::destroyWindow(btnsWinName);
		c.setType(objTypes[objID]._width, objTypes[objID]._height, objTypes[objID]._depth, objTypes[objID]._name);
		double cnt = c.countObjects();
		if (cnt >= 0)
			showResultImg(cnt);
		else
			std::cout << "bad count: " << cnt << std::endl;
	}
	
	return EXIT_SUCCESS; 
}

void showResultImg(double count)
{
	cv::Mat resultImg;
	resultImg.create(80, 500, CV_8UC3);
	std::string text1 = "There are " + std::to_string(ROUND(count)) + " objects on the shelf";
	std::string text2 = "Press any key to continue...";
	resultImg.setTo(cv::Scalar(200, 200, 200));
	putText(resultImg, text1, cv::Point((resultImg.cols - text1.length() * 10)/2, resultImg.rows*0.2), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
	putText(resultImg, text2, cv::Point((resultImg.cols - text2.length() * 10)/2, resultImg.rows*0.6), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
	std::string resultWinName = "result";
	cv::imshow(resultWinName, resultImg);
	cv::waitKey(0);
	cv::destroyWindow(resultWinName);
}

void createPopupImages()
{
	std::vector<cv::Rect> buttons;
	btnsImg.create(buttonHeight, objTypes.size() * buttonWidth, CV_8UC3);
	for (int i = 0 ; i < objTypes.size(); i++)
	{
		cv::Rect button = cv::Rect(buttonWidth * i, 0, buttonWidth, buttonHeight);
		objButtons.push_back(button);
		btnsImg(button).setTo(cv::Scalar(215, 215, 215));
		cv::rectangle(btnsImg, button, cv::Scalar(169,169,169), 2);
		putText(btnsImg(button), objTypes[i]._name, cv::Point((button.width - objTypes[i]._name.length() * 10)/2, button.height*0.6), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
	}
}

void createObjectsTypeList()
{
	objTypes.push_back(objectType(150, -50, 40, "box"));
	//objTypes.push_back(objectType(80, -240, 60, "shampoo"));
	//objTypes.push_back(objectType(50, -135, 55, "deodorant"));
	objTypes.push_back(objectType(60, -210, 45, "shampoo"));
	objTypes.push_back(objectType(40, -150, 45, "deodorant"));
}


static void onMouse(int event, int x, int y, int flags, void* param)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{	
		cv::Mat &img = *((cv::Mat*)param);
		cv::Mat heatmap, source;
		img.convertTo(heatmap, CV_32FC1);
		src.convertTo(source, CV_32FC1);
		double u = (float)x;
		double v = (float)y;
		if(source.at<float>(ROUND(v), ROUND(u)) == 0)
		{
			std::cout << "You chose a bad coordinate for the separator. Please try again!" << std::endl;
		} else
		{
			// EPV::CameraIntrinsics intrinsics = { 671.062439,671.062439,679.713806,369.511169,source.cols,source.rows };
            EPV::CameraIntrinsics intrinsics = { 402.6374816894531,402.6374816894531,343.8282775878906,245.70669555664062,source.cols,source.rows };
			const cv::Point offset = cv::Point(0, 0);
			capture = false;
			double z = source.at<float>(ROUND(v), ROUND(u));
			double x_world = z *  (u + offset.x - intrinsics.px) / intrinsics.fx;
			double y_world = z *  (v + offset.y - intrinsics.py) / intrinsics.fy;
			std::cout << "for (" << u << "," << v << "), real coordinates are: " << "x = " << x_world << ", y = " << y_world << ", z = " << z << std::endl;
			cv::Point3d sep(x_world, y_world, z);
			c.setSeparator(sep);
			// separator.x = x_world;
			// separator.y = y_world;
			// separator.z = z;
		}
	}
}

static void onBtnClick(int event, int x, int y, int flags, void* param)
{
	
	//TODO: EVENT_LBUTTONUP
	if (event == cv::EVENT_LBUTTONDOWN)
    {
		for(int i = 0 ; i < objButtons.size() ; i++)
		{
			if (objButtons[i].contains(cv::Point(x, y)))
			{
				objID = i;
			}
		}
    }
}

// projecting point p to the plane defined by the normal and a point
// equation from https://stackoverflow.com/questions/8942950/how-do-i-find-the-orthogonal-projection-of-a-point-onto-a-plane 
cv::Point3d projection(cv::Point3d p, cv::Point3d planePnt, cv::Point3d normal)
{
cv::Point3d projectedPnt = p - (p - planePnt).dot(normal) * normal;
return projectedPnt;
}

// if need to rotate 180 deg and convert blue to red in color image
void adjustColorImage(cv::Mat& src)
{
	cv::cvtColor(src, src, CV_BGR2RGB);
	cv::rotate(src, src, 1);
	return;
}