#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <time.h>
#include <ros/ros.h>
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ctime>
#include <chrono>
#include <ObjectPose.h>
#include <DominantPlane.h>
namespace enc = sensor_msgs::image_encodings;
static int Frame_count = 0;

class SystemHandler 
{
public:
	SystemHandler(Plane::DominantPlane* _plane, ObjectPose* _pose)
	{
		//Topic you want to publish
		// pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);
        pose_obj = _pose;
        plane_obj = _plane;
		//Topic you want to subscribe
		sub_color = nh.subscribe("/camera/color/image_raw", 100, &SystemHandler::ImageCallback, this);
		sub_depth = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 100, &SystemHandler::DepthCallback, this);
	}

	void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		// ROS_INFO("Color image height = %d, width = %d", msg->height, msg->width);
		cv_bridge::CvImagePtr cv_ptrRGB;
		try
		{
			cv_ptrRGB = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			imRGB = cv_ptrRGB->image;
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

	}

	void DepthCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		Frame_count++;
		// ROS_INFO("Depth image height = %d, width = %d", msg->height, msg->width);
		cv_bridge::CvImagePtr cv_ptrD;
		try 
		{
			cv_ptrD = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
			imDepth = cv_ptrD->image;
			if (imRGB.empty() || imDepth.empty()) {
				cerr << endl << "Failed to load image at: " << endl;
				return;
			} 
			else if(Frame_count%6 == 0)
				Run_pipeline(imRGB, imDepth);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	void Run_pipeline(cv::Mat& image_RGB, cv::Mat& image_Depth);

private:
	ros::NodeHandle nh;
	ros::Publisher pub_;
	ros::Subscriber sub_color;
	ros::Subscriber sub_depth;
    cv::Mat imRGB;
	cv::Mat imDepth;
    Plane::DominantPlane* plane_obj;
    ObjectPose* pose_obj;
};//End of class SubscribeAndPublish
