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

struct Point3D{ float x; float y; float z; };


class ObjectPose{
    public:
        ObjectPose(int _height, int _width, int _Accum_iter,float _fx, float _fy, float _cx, float _cy, float _unit_length, float _dist_thresh, Plane::DominantPlane* plane, cv::FileStorage fconfig);
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
        void GenerateRealSyntheticCloud(std::vector<int> Grid_size, std::vector<int> occ_Grid, std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector);
        void ClearVariable();
        // Occupany grid size
        std::vector<int> red_Grid;
        std::vector<int> yellow_Grid;
        std::vector<int> green_Grid; 
        std::vector<int> blue_Grid;
        std::vector<int> brown_Grid;
        std::vector<int> orange_Grid;
        std::vector<int> Indigo_Grid;
        std::vector<int> red_occ_Grid;
        std::vector<int> yellow_occ_Grid;
        std::vector<int> green_occ_Grid;
        std::vector<int> blue_occ_Grid;
        std::vector<int> brown_occ_Grid;
        std::vector<int> orange_occ_Grid;
        std::vector<int> Indigo_occ_Grid;
        // Bounding box vertex information
        std::vector<Point3D> BB_info_red;
        std::vector<Point3D> BB_info_yellow;
        std::vector<Point3D> BB_info_green;
        std::vector<Point3D> BB_info_blue;
        std::vector<Point3D> BB_info_brown;
        std::vector<Point3D> BB_info_orange;
        std::vector<Point3D> BB_info_Indigo;
        // Block center point cloud infomation
        std::vector<Point3D> Block_center_red;
        std::vector<Point3D> Block_center_yellow;
        std::vector<Point3D> Block_center_green;
        std::vector<Point3D> Block_center_blue;
        std::vector<Point3D> Block_center_brown;
        std::vector<Point3D> Block_center_orange;
        std::vector<Point3D> Block_center_Indigo;
        // RectPoint of each individual block
        std::vector<Point> Red_RectPoints = std::vector<Point>(4);
        std::vector<Point> Yellow_RectPoints = std::vector<Point>(4);
        std::vector<Point> Green_RectPoints = std::vector<Point>(4);
        std::vector<Point> Blue_RectPoints = std::vector<Point>(4);
        std::vector<Point> Brown_RectPoints = std::vector<Point>(4);
        std::vector<Point> Orange_RectPoints = std::vector<Point>(4);
        std::vector<Point> Indigo_RectPoints = std::vector<Point>(4);
        std::vector<Point> _RectPoints = std::vector<Point>(4);
        // Projected image 
        cv::Mat Total_Projected_image;
        cv::Mat _Projected_image;

      protected:
        int Accum_iter; 
        int Accum_idx; 
        int height; 
        int width; 
        int index; 
        int box_flag;
        // flag for test
        int Test_all_flag;
        int Test_Individual_flag;
        int Test_red_flag;
        int Test_yellow_flag;
        int Test_green_flag;
        int Test_blue_flag;
        int Test_brown_flag;
        int Test_orange_flag;
        int Test_indigo_flag;
        // Debug flag
        int Debug_Object;
        float fx; 
        float fy;
        float cx; 
        float cy;
        float Unit_Cube_L;
        float dist_thresh;
        bool red_change_flag;
        // color string value during pose estimation of block 
        std::string color_string;
        // Synthetic point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Red_Synthetic;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Yellow_Synthetic;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Green_Synthetic;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Blue_Synthetic;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Brown_Synthetic;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Orange_Synthetic;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Indigo_Synthetic;
        // Accumulated point cloud
        pcl::PointCloud<pcl::PointXYZRGB> red_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> yellow_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> green_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> blue_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> brown_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> orange_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> Indigo_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> projected_cloud;
        std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> BBinfo_temp; 
        pcl::PointCloud<pcl::PointXYZRGB> Block_center_temp;
        Plane::DominantPlane* plane_object;
        Plane::Plane_model best_plane; 
};

#endif 
