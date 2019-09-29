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
        ObjectPose(int _height, int _width, int _Accum_iter,float _fx, float _fy, float _cx, float _cy, float _unit_length, float _dist_thresh, Plane::DominantPlane* plane);
        void Accumulate_PointCloud(cv::Mat& pcd_outlier, std::vector<cv::Mat>& Mask);
        void CloudView(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector);
        void ProjectToDominantPlane(pcl::PointCloud<pcl::PointXYZRGB> in_cloud, std::string _color_string);
        void ProjectedCloudToImagePlane();
        void fitRectangle(cv::Mat image);
        void BackProjectToDominatPlane(std::vector<Point> Rect_points);
        float FindBlockHeight(pcl::PointCloud<pcl::PointXYZRGB> in_cloud, float a, float b, float c, float d);
        float FindBlockMeanHeight(pcl::PointCloud<pcl::PointXYZRGB> in_cloud, float a, float b, float c, float d);
        void FindOccGrid(std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector, float max_height);
        void MeasureOccupany(std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector, std::vector<int> Grid_size, pcl::PointCloud<pcl::PointXYZRGB> ref_cloud);
        void CheckOccGridWithKnownShape(std::vector<int> Grid_size, std::vector<int> occ_grid);
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
        float unit_length;
        float dist_thresh;
        std::string color_string;
        pcl::PointCloud<pcl::PointXYZRGB> red_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> yellow_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> green_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> blue_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> brown_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> orange_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> purple_cloud;
        std::vector<int> red_Grid;
        std::vector<int> yellow_Grid;
        std::vector<int> green_Grid; 
        std::vector<int> blue_Grid;
        std::vector<int> brown_Grid;
        std::vector<int> orange_Grid;
        std::vector<int> purple_Grid;
        std::vector<int> red_occ_Grid;
        std::vector<int> yellow_occ_Grid;
        std::vector<int> green_occ_Grid;
        std::vector<int> blue_occ_Grid;
        std::vector<int> brown_occ_Grid;
        std::vector<int> orange_occ_Grid;
        std::vector<int> purple_occ_Grid;
        std::vector<Point> _RectPoints = std::vector<Point> (4);
        cv::Mat _Projected_image;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud;
        Plane::DominantPlane* plane_object;
        Plane::Plane_model best_plane; 
};

#endif 
