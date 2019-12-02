//
// Created by intelpro on 3/26/19.
//

//#ifndef DOMINANTPLANE_DOMINANTPLANE_H
//#define DOMINANTPLANE_DOMINANTPLANE_H
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <librealsense2/rs.hpp>
#include <opencv2/line_descriptor.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/features2d.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/viz.hpp>
#include "opencv2/core/core.hpp"

using namespace std;
namespace Plane {
    struct Plane_model
    {
        float a; // normal vector x element
        float b; // normal vector y element
        float c; // normal vector z element
        float d; // plane constant ( ax+by+cz+d = 0 )
        float denominator; // used for calculating distance
        float avg_distance;
    };
    class DominantPlane {
    public:
        DominantPlane(float _fx, float _fy, float _cx, float _cy, float _DepthMapFactor, float _Distance_thresh,  int _max_iter, int _width, int _height);

        ~DominantPlane(void) {}

        cv::Mat Depth2pcd(cv::Mat &depth);
        Plane::Plane_model RunRansac(cv::Mat &pcd_inlier);
        void ResetValue()
        {
            N_pcds = 0;
            frame_id ++;
            pcd_concat.clear();
        };
        void FindPlaneFromSampledPcds(std::vector<cv::Vec3f> &sampled_pcds, Plane_model& Plane);
        void compute_inlier_cost(Plane_model& plane, cv::Mat& pcd_inlier, cv::Mat& PointCloud);
        void Object_Segmentation(cv::Mat& pcd_inlier, cv::Mat& pcd_object);

        void ObjectSegmentation(Plane_model& plane, cv::Mat& pcd_object);

        void FaceSegmentation(Plane_model, cv::Mat &, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&);
        cv::Mat DistantPlaneSegmentation(Plane_model, float);
        static cv::Mat mK;
        static float DepthMapFactor;
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float Distance_threshold;
        // number of point cloud
        static int max_iter;
        static int N_pcds;
        static int frame_id;
        static int N_inlier;
        static int width;
        static int height;
        static cv::Mat Pointcloud;
        std::vector<cv::Vec3f> pcd_concat;
    };
}
//#endif //DOMINANTPLANE_DOMINANTPLANE_H

Plane::Plane_model Segmentation(cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&);
void Show_Results(cv::Mat&, cv::Mat, std::string);
cv::Mat RGB2pcl(cv::Mat, cv::Mat);
void registration(cv::Mat&);
cv::Mat imageCb(cv::Mat&);
pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat);
