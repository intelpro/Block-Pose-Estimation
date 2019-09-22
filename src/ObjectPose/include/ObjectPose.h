#ifndef OBJECTPOSE_OBJECTPOSE_H
#define OBJECTPOSE_OBJECTPOSE_H
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <thread>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <DominantPlane.h>

using namespace std;
using namespace cv; 


class ObjectPose{
    public:
        ObjectPose(int _height, int _width, int _Accum_iter,float _fx, float _fy, float _cx, float _cy, Plane::DominantPlane* plane);
        void Accumulate_PointCloud(cv::Mat& pcd_outlier, cv::Mat& Mask);
        void CloudView(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector);
        void XYZRGB2XYZnFiltering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);
        void ProjectedCloudToImagePlane();
        void ProjectToImage();
        void fitRectangle(cv::Mat image);
        void ProjectToDominantPlane();
        void BackProjectToDominatPlane(std::vector<Point2f> Rect_points);
    
    protected:
        // cv::Mat Red_Mask;
        int Accum_iter; 
        int Accum_idx; 
        int height; 
        int width; 
        int index; 
        int box_flag;
        float fx; 
        float fy;
        float cx; 
        float cy;
        pcl::PointCloud<pcl::PointXYZRGB> merged_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud;
        Plane::DominantPlane* plane_object;
        Plane::Plane_model best_plane; 
};

#endif 
