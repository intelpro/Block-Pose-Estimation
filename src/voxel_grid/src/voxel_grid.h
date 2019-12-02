#include <string>
#include <fstream>
#include <opencv2/viz.hpp>
#include <time.h>
#include "DominantPlane.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ctime>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <sstream>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <librealsense2/rs.hpp>
#include <opencv2/line_descriptor.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/features2d.hpp>
#ifndef PCL_NO_RECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE
 struct KeyLine3D
{
    float startPointX; 
    float startPointY;
    float startPointD;
    float endPointX;
    float endPointY;
    float endPointD;
    float directionVectorX;
    float directionVectorY;
    float directionVectorD;
};
struct GridXYZ3D
{
    cv::Vec3f GridX;
    cv::Vec3f GridY;
    cv::Vec3f GridZ;
};
int LinesContactChecker(KeyLine3D, KeyLine3D, float);
   
using namespace cv;
using namespace cv::line_descriptor;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
void DepthCallback(const sensor_msgs::ImageConstPtr& msg);
//void pCloudCallback(const sensor_msgs::ImageConstPtr& msg);

void Get_RGB(cv_bridge::CvImageConstPtr& cv_ptr);
void Get_Depth(cv_bridge::CvImageConstPtr& cv_ptr);

void Segmentation(cv::Mat& image_RGB, cv::Mat& image_Depth);

void Show_Results(cv::Mat& pointCloud, cv::Mat RGB_image_original);

cv::Mat imageCb(cv::Mat& RGB_image);
cv::Mat RGB2pcl(cv::Mat red_image, cv::Mat pointCloud);

void registration(cv::Mat& pointCloud_output);

pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat OpencVPointCloud);

cv::Mat line_segment(cv::Mat);
cv::Mat color_segment(cv::Mat, cv::Mat &redMask, cv::Mat &orangeMask, cv::Mat &yellowMask, cv::Mat &greenMask, cv::Mat &blueMask, cv::Mat &purpleMask, cv::Mat &brownMask, cv::Mat Mask); 
void Get_keyLines(std::vector<KeyLine> &keyLines, cv::Mat imRGB, cv::Mat mask);
cv::Mat line_visualize(std::vector<KeyLine> keyLines, cv::Mat imRGB);

void Get3DLines(std::vector<KeyLine>, cv::Mat, std::vector<KeyLine3D>&);
int GetGridXYZ(std::vector<KeyLine3D>, cv::Point3f, std::vector<cv::Point3f>&, cv::Point3f&);
void OriginVisualize(cv::Point3f Origin, std::vector<cv::Point3f> GRIDXYZ, cv::Mat imRGB);
cv::Point2d Project2Img(cv::Point3f Point);


void VoxelGridVisualize(cv::Mat, cv::Point3f, std::vector<cv::Point3f>, cv::Mat);
int FillVoxelGrid(vector<vector<int> > &, cv::Point3f Origin, std::vector<cv::Point3f> GRIDXYZ, Plane::Plane_model DominantPlane, cv::Mat FirstLevel, cv::Mat SecondLevel, cv::Mat ThirdLevel, cv::Mat redmask, cv::Mat orangemask, cv::Mat yellowmask, cv::Mat greenmask, cv::Mat bluemask, cv::Mat brownmask, cv::Mat purplemask, cv::Mat, int&, int&, int&, int&, int&, int&);
int ArgMax(int, int, int, int, int, int, int);
void GetGridPos(vector<vector<vector<vector<int>>>>, int VoxelGridSize, int col, int row, int level,  int &GridX, int &GridY);
