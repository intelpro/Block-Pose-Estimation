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
#include <block_pose/RedBlock.h>
#include <block_pose/YellowBlock.h>
#include <block_pose/BlueBlock.h>
#include <block_pose/GreenBlock.h>
#include <block_pose/BrownBlock.h>
#include <block_pose/OrangeBlock.h>
#include <block_pose/PurpleBlock.h>
#include <geometry_msgs/Point.h>

namespace enc = sensor_msgs::image_encodings;
static int Frame_count = 0;

class SystemHandler 
{
public:

    SystemHandler(const string &strConfig)
    {
        cv::FileStorage fconfig(strConfig.c_str(), cv::FileStorage::READ);
        if(!fconfig.isOpened())
        {
           cerr << "Failed to open Configuration file at: " << strConfig << endl;
           exit(-1);
        }
        // Camera parameters
        fx = fconfig["Camera.fx"];
        fy = fconfig["Camera.fy"];
        cx = fconfig["Camera.cx"];
        cy = fconfig["Camera.cy"];
        width = fconfig["Camera.width"];
        height = fconfig["Camera.height"];
        scale = fconfig["DepthMapFactor"];
        // Pose Finder parameters 
        Depth_Accum_iter = fconfig["Pose.CloudAccumIter"];
        unit_length = fconfig["Pose.GridUnitLength"];
        Threshold_for_occgrid = fconfig["Pose.ThresholdForOccGrid"];
        // Plane Finder parameters
        Distance_theshold = fconfig["Plane.DistanceThresh"];
        max_iter = fconfig["Plane.Maxiter"];
        // Crop parameters
        Crop_x_min = fconfig["Crop.xmin"];
        Crop_x_max = fconfig["Crop.xmax"];
        Crop_y_min = fconfig["Crop.ymin"];
        Crop_y_max = fconfig["Crop.ymax"];
        cout << "------------------------- Parameter -------------------------" << endl;
        cout << "fx: " << fx << endl;
        cout << "fy: " << fy << endl;
        cout << "cx: " << cx << endl;
        cout << "cy: " << cy << endl;
        cout << "scale: " << scale << endl;
        cout << "unit length: " << unit_length << endl;
        cout << "Threshold for occ grid: " << Threshold_for_occgrid << endl;
        cout << "Cloud Accum iter: " << Depth_Accum_iter << endl;
        cout << "Dist Thresh: " << Distance_theshold << endl;
        cout << "max iter: " << max_iter << endl;
        cout << "width: " << width << endl;
        cout << "height: " << height << endl;
        cout << "Crop x min: " << Crop_x_min << endl;
        cout << "Crop x max: " << Crop_x_max << endl;
        cout << "Crop y min: " << Crop_y_min << endl;
        cout << "Crop y max: " << Crop_y_max << endl;
        // Object pointer declaration
        PlaneFinder = new Plane::DominantPlane(fx,fy,cx,cy, scale, Distance_theshold, max_iter, width, height);
        PoseFinder = new ObjectPose(height, width, Depth_Accum_iter, fx, fy, cx, cy, unit_length, Threshold_for_occgrid, PlaneFinder);
        //Topic you want to publish
        pub_red = nh.advertise<block_pose::RedBlock>("/block_info/red", 100);
        pub_yellow = nh.advertise<block_pose::YellowBlock>("/block_info/yellow", 100);
        pub_green = nh.advertise<block_pose::GreenBlock>("/block_info/green", 100);
        pub_blue = nh.advertise<block_pose::BlueBlock>("/block_info/blue", 100);
        pub_brown = nh.advertise<block_pose::BrownBlock>("/block_info/brown", 100);
        pub_orange = nh.advertise<block_pose::OrangeBlock>("/block_info/orange", 100);
        pub_purple = nh.advertise<block_pose::PurpleBlock>("/block_info/purple", 100);
        // image declear
        imRGB = cv::Mat::zeros(height, width, CV_8UC3);
        imDepth = cv::Mat::zeros(height, width, CV_8UC3);
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
            {
                preprocess_image(imRGB, imDepth);
                Run_pipeline(imRGB_processed, imDepth_processed);
                Publish_Message();
            }
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	void preprocess_image(cv::Mat& image_RGB, cv::Mat& image_Depth);
	void Run_pipeline(cv::Mat& image_RGB, cv::Mat& image_Depth);
    void Publish_Message();

private:
    // hyper parameter
    float fx;
    float fy;
    float cx;
    float cy;
    float scale;
    float Distance_theshold;
    float unit_length;
    float Threshold_for_occgrid;
    int width;
    int height;
    int max_iter;
    int Depth_Accum_iter;
    int Crop_x_max;
    int Crop_x_min;
    int Crop_y_max;
    int Crop_y_min;
    // Node Handler
    ros::NodeHandle nh;
    // Subscriber
    ros::Subscriber sub_color;
    ros::Subscriber sub_depth;
    // Publisher
    ros::Publisher pub_red;
    ros::Publisher pub_yellow;
    ros::Publisher pub_green;
    ros::Publisher pub_blue;
    ros::Publisher pub_brown;
    ros::Publisher pub_orange;
    ros::Publisher pub_purple;
    // Block message
    block_pose::RedBlock Red_msg;
    block_pose::YellowBlock Yellow_msg;
    block_pose::GreenBlock Green_msg;
    block_pose::BlueBlock Blue_msg;
    block_pose::BrownBlock Brown_msg;
    block_pose::OrangeBlock Orange_msg;
    block_pose::PurpleBlock Purple_msg;
    // RGB image and Depth image
    cv::Mat imRGB;
	cv::Mat imDepth;
    cv::Mat imRGB_processed;
    cv::Mat imDepth_processed;
    // Plane and pose object
    Plane::DominantPlane* PlaneFinder;
    ObjectPose* PoseFinder;
};//End of class SubscribeAndPublish
