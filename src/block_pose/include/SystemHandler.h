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
#include <block_pose/IndigoBlock.h>
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
        // Visualization 
        cv::Mat imColorDebug = cv::Mat::zeros(height, width, CV_8UC3); 
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
        // Image show flag
        HSVImgShow_flag = fconfig["SystemHandler.imshowHSV"];
        SegImgShow_flag = fconfig["SystemHandler.imshowSegImg"];
        DepthImgShow_flag = fconfig["SystemHandler.imshowDepthImg"];
        ColorDebug_flag = fconfig["SystemHandler.ColorDebug"];
        // color parameter
        // Red color
        lower_Red_value1 = cv::Scalar(fconfig["Red_color.lower_value_0_0"],fconfig["Red_color.lower_value_1_0"],fconfig["Red_color.lower_value_2_0"]);
        lower_Red_value2 = cv::Scalar(fconfig["Red_color.lower_value_0_1"],fconfig["Red_color.lower_value_1_1"],fconfig["Red_color.lower_value_2_1"]);
        upper_Red_value1 = cv::Scalar(fconfig["Red_color.upper_value_0_0"],fconfig["Red_color.upper_value_1_0"],fconfig["Red_color.upper_value_2_0"]);
        upper_Red_value2 = cv::Scalar(fconfig["Red_color.upper_value_0_1"],fconfig["Red_color.upper_value_1_1"],fconfig["Red_color.upper_value_2_1"]);
        Red_imshow_flag = fconfig["Red_color.imshow"];
        // Orange color
        Orange_value1 = cv::Scalar(fconfig["Orange_color.value_0_0"],fconfig["Orange_color.value_1_0"],fconfig["Orange_color.value_2_0"]);
        Orange_value2 = cv::Scalar(fconfig["Orange_color.value_0_1"],fconfig["Orange_color.value_1_1"],fconfig["Orange_color.value_2_1"]);
        Orange_imshow_flag = fconfig["Orange_color.imshow"];
        // Yellow color
        Yellow_value1 = cv::Scalar(fconfig["Yellow_color.value_0_0"],fconfig["Yellow_color.value_1_0"],fconfig["Yellow_color.value_2_0"]);
        Yellow_value2 = cv::Scalar(fconfig["Yellow_color.value_0_1"],fconfig["Yellow_color.value_1_1"],fconfig["Yellow_color.value_2_1"]);
        Yellow_imshow_flag = fconfig["Yellow_color.imshow"];
        // Green color
        Green_value1 = cv::Scalar(fconfig["Green_color.value_0_0"],fconfig["Green_color.value_1_0"],fconfig["Green_color.value_2_0"]);
        Green_value2 = cv::Scalar(fconfig["Green_color.value_0_1"],fconfig["Green_color.value_1_1"],fconfig["Green_color.value_2_1"]);
        Green_imshow_flag = fconfig["Green_color.imshow"];
        // Blue color
        Blue_value1 = cv::Scalar(fconfig["Blue_color.value_0_0"],fconfig["Blue_color.value_1_0"],fconfig["Blue_color.value_2_0"]);
        Blue_value2 = cv::Scalar(fconfig["Blue_color.value_0_1"],fconfig["Blue_color.value_1_1"],fconfig["Blue_color.value_2_1"]);
        Blue_imshow_flag = fconfig["Blue_color.imshow"];
        // Indigo color
        lower_Indigo_value1 = cv::Scalar(fconfig["Indigo_color.lower_value_0_0"],fconfig["Indigo_color.lower_value_1_0"],fconfig["Indigo_color.lower_value_2_0"]);
        lower_Indigo_value2 = cv::Scalar(fconfig["Indigo_color.lower_value_0_1"],fconfig["Indigo_color.lower_value_1_1"],fconfig["Indigo_color.lower_value_2_1"]);
        upper_Indigo_value1 = cv::Scalar(fconfig["Indigo_color.upper_value_0_0"],fconfig["Indigo_color.upper_value_1_0"],fconfig["Indigo_color.upper_value_2_0"]);
        upper_Indigo_value2 = cv::Scalar(fconfig["Indigo_color.upper_value_0_1"],fconfig["Indigo_color.upper_value_1_1"],fconfig["Indigo_color.upper_value_2_1"]);
        Indigo_imshow_flag = fconfig["Indigo_color.imshow"];
        // Brown color
        lower_Brown_value1 = cv::Scalar(fconfig["Brown_color.lower_value_0_0"],fconfig["Brown_color.lower_value_1_0"],fconfig["Brown_color.lower_value_2_0"]);
        lower_Brown_value2 = cv::Scalar(fconfig["Brown_color.lower_value_0_1"],fconfig["Brown_color.lower_value_1_1"],fconfig["Brown_color.lower_value_2_1"]);
        upper_Brown_value1 = cv::Scalar(fconfig["Brown_color.upper_value_0_0"],fconfig["Brown_color.upper_value_1_0"],fconfig["Brown_color.upper_value_2_0"]);
        upper_Brown_value2 = cv::Scalar(fconfig["Brown_color.upper_value_0_1"],fconfig["Brown_color.upper_value_1_1"],fconfig["Brown_color.upper_value_2_1"]);
        Brown_imshow_flag = fconfig["Brown_color.imshow"];

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
        cout << "------------------------- Color space value -------------------------" << endl;
        cout << "Red value(lower): [" << lower_Red_value1[0] << " " << lower_Red_value1[1] << " " << lower_Red_value1[2] << "]  ["  <<
                lower_Red_value2[0] << " " << lower_Red_value2[1] << " " << lower_Red_value2[2] << "]" <<  endl;
        cout << "Red value(upper): [" << upper_Red_value1[0] << " " << upper_Red_value1[1] << " " << upper_Red_value1[2] << "]  ["  <<
                upper_Red_value2[0] << " " << upper_Red_value2[1] << " " << upper_Red_value2[2] << "]" <<  endl;
        cout << "Orange value: [" << Orange_value1[0] << " " << Orange_value1[1] << " " << Orange_value1[2] << "]  ["  <<
                Orange_value2[0] << " " << Orange_value2[1] << " " << Orange_value2[2] << "]" <<  endl;
        cout << "Yellow value: [" << Yellow_value1[0] << " " << Yellow_value1[1] << " " << Yellow_value1[2] << "]  ["  <<
                Yellow_value2[0] << " " << Yellow_value2[1] << " " << Yellow_value2[2] << "]" <<  endl;
        cout << "Green value: [" << Green_value1[0] << " " << Green_value1[1] << " " << Green_value1[2] << "]  ["  <<
                Green_value2[0] << " " << Green_value2[1] << " " << Green_value2[2] << "]" <<  endl;
        cout << "Blue value: [" << Blue_value1[0] << " " << Blue_value1[1] << " " << Blue_value1[2] << "]  ["  <<
                Blue_value2[0] << " " << Blue_value2[1] << " " << Blue_value2[2] << "]" <<  endl;
        cout << "Indigo value(lower): [" << lower_Indigo_value1[0] << " " << lower_Indigo_value1[1] << " " << lower_Indigo_value1[2] << "]  ["  <<
                lower_Indigo_value2[0] << " " << lower_Indigo_value2[1] << " " << lower_Indigo_value2[2] << "]" <<  endl;
        cout << "Indigo value(upper): [" << upper_Indigo_value1[0] << " " << upper_Indigo_value1[1] << " " << upper_Indigo_value1[2] << "]  ["  <<
                upper_Indigo_value2[0] << " " << upper_Indigo_value2[1] << " " << upper_Indigo_value2[2] << "]" <<  endl;
        cout << "Brown value(lower): [" << lower_Brown_value1[0] << " " << lower_Brown_value1[1] << " " << lower_Brown_value1[2] << "]  ["  <<
                lower_Brown_value2[0] << " " << lower_Brown_value2[1] << " " << lower_Brown_value2[2] << "]" <<  endl;
        cout << "Brown value(upper): [" << upper_Brown_value1[0] << " " << upper_Brown_value1[1] << " " << upper_Brown_value1[2] << "]  ["  <<
                upper_Brown_value2[0] << " " << upper_Brown_value2[1] << " " << upper_Brown_value2[2] << "]" <<  endl;

        // Object pointer declaration
        PlaneFinder = new Plane::DominantPlane(fx,fy,cx,cy, scale, Distance_theshold, max_iter, width, height);
        PoseFinder = new ObjectPose(height, width, Depth_Accum_iter, fx, fy, cx, cy, unit_length, Threshold_for_occgrid, PlaneFinder, fconfig);
        //Topic you want to publish
        pub_red = nh.advertise<block_pose::RedBlock>("/block_info/red", 100);
        pub_yellow = nh.advertise<block_pose::YellowBlock>("/block_info/yellow", 100);
        pub_green = nh.advertise<block_pose::GreenBlock>("/block_info/green", 100);
        pub_blue = nh.advertise<block_pose::BlueBlock>("/block_info/blue", 100);
        pub_brown = nh.advertise<block_pose::BrownBlock>("/block_info/brown", 100);
        pub_orange = nh.advertise<block_pose::OrangeBlock>("/block_info/orange", 100);
        pub_Indigo = nh.advertise<block_pose::IndigoBlock>("/block_info/Indigo", 100);
        // image declear
        imRGB = cv::Mat::zeros(height, width, CV_8UC3);
        imDepth = cv::Mat::zeros(height, width, CV_16UC1);
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
                preprocess_image(imRGB);
                Run_pipeline(imRGB_processed, imDepth);
                Publish_Message();
            }
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

    void preprocess_image(cv::Mat& image_RGB);
    void Run_pipeline(cv::Mat& image_RGB, cv::Mat& image_Depth);
    void Publish_Message();
    void ColorSegmenation(cv::Mat& RGB_image, std::vector<cv::Mat>& Mask_vector);
    void Show_Results(cv::Mat& pointCloud, cv::Mat RGB_image_original, cv::Mat RGB_masked, std::string window_name);

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
    int Red_imshow_flag;
    int Orange_imshow_flag;
    int Yellow_imshow_flag;
    int Green_imshow_flag;
    int Blue_imshow_flag;
    int Indigo_imshow_flag;
    int Brown_imshow_flag;
    // Imshow flag
    int SegImgShow_flag;
    int HSVImgShow_flag;
    int ColorDebug_flag;
    int DepthImgShow_flag;
    // color parameter
    cv::Scalar lower_Red_value1;
    cv::Scalar lower_Red_value2;
    cv::Scalar upper_Red_value1;
    cv::Scalar upper_Red_value2;
    cv::Scalar Orange_value1;
    cv::Scalar Orange_value2;
    cv::Scalar Yellow_value1;
    cv::Scalar Yellow_value2;
    cv::Scalar Green_value1;
    cv::Scalar Green_value2;
    cv::Scalar Blue_value1;
    cv::Scalar Blue_value2;
    cv::Scalar lower_Indigo_value1;
    cv::Scalar lower_Indigo_value2;
    cv::Scalar upper_Indigo_value1;
    cv::Scalar upper_Indigo_value2;
    cv::Scalar lower_Brown_value1;
    cv::Scalar lower_Brown_value2;
    cv::Scalar upper_Brown_value1;
    cv::Scalar upper_Brown_value2;
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
    ros::Publisher pub_Indigo;
    // Block message
    block_pose::RedBlock Red_msg;
    block_pose::YellowBlock Yellow_msg;
    block_pose::GreenBlock Green_msg;
    block_pose::BlueBlock Blue_msg;
    block_pose::BrownBlock Brown_msg;
    block_pose::OrangeBlock Orange_msg;
    block_pose::IndigoBlock Indigo_msg;
    // RGB image and Depth image
    cv::Mat imRGB;
	cv::Mat imDepth;
    cv::Mat imRGB_processed;
    // Image for Color debug
    cv::Mat imColorDebug;
    // Plane and pose object
    Plane::DominantPlane* PlaneFinder;
    ObjectPose* PoseFinder;
};//End of class SubscribeAndPublish
