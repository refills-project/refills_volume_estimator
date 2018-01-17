/**
* @ file Volume_Counter.cpp
* @ counts objects by volume estimation
* @ method - first the algorithm finds the planes of the shelf and the wall, which represent the height direction and the length direction.
* @			 The width direction can be calculated from their cross product. We use those vectors to create a box around the relevant products, starting from the left separator.
* @			 The count is done 1 line at a time, and does not take in calculation any missing object in the middle of the line.
* @ arguments: depth image 
* @ output: number of products
*/
#include "counter.h"
#include "counterUtils.h"

void createObjectsTypeList();
void createPopupImages();
void showResultImg(double count);
static void onMouse(int event, int x, int y, int flags, void* param);
static void onBtnClick(int event, int x, int y, int flags, void* param);

////////////////Demo Vars//////////////////////
counter c;
bool pf = true;
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
    std::cout << "sssssssssssssssssssss" << std::endl;
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
		c.setIntrinsics(402.6374816894531,402.6374816894531,343.8282775878906,245.70669555664062,depthMat.cols,depthMat.rows);
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
		if(!pf)
		{
			///// Ferenc - you need to give here the rotation matrix from the robot (c._rotation)
		}
		double cnt = c.countObjects(pf);
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
            EPV::CameraIntrinsics intrinsics = { 402.6374816894531,402.6374816894531,343.8282775878906,245.70669555664062,source.cols,source.rows };
			const cv::Point offset = cv::Point(0, 0);
			capture = false;
			double z = source.at<float>(ROUND(v), ROUND(u));
			double x_world = z *  (u + offset.x - intrinsics.px) / intrinsics.fx;
			double y_world = z *  (v + offset.y - intrinsics.py) / intrinsics.fy;
			std::cout << "for (" << u << "," << v << "), real coordinates are: " << "x = " << x_world << ", y = " << y_world << ", z = " << z << std::endl;
			cv::Point3d sep(x_world, y_world, z);
			c.setSeparator(sep);
		}
	}
}

static void onBtnClick(int event, int x, int y, int flags, void* param)
{
	
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