#include <math.h>       /* atan */
#include <opencv2/viz.hpp>

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

//#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "matching_algorithm.h"

# include <message_filters/subscriber.h>
# include <message_filters/time_synchronizer.h>
# include "sensor_msgs/image_encodings.h"
# include "cv_bridge/cv_bridge.h"

#include "ros/ros.h"
#include "matching_algorithm/Data1d.h"
#include "matching_algorithm/Data2d.h"

#define PI 3.14159265

using namespace cv;
using namespace std;

Rect lineFitting(cv::Mat image);
double GetDist(Plane::Plane_model plane, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier);
cv::Mat DistantPlaneSegmentation(Plane::Plane_model plane, cv::Mat red_pcloud, float distance);
cv::Mat dominant_plane_projection(Plane::Plane_model plane, cv::Mat pCloud);
cv::Mat get_projected_image(Plane::Plane_model plane, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
cv::Mat RectFitting(cv::Mat projected_image, std::string string);
void LetsFindBox(cv::Mat projected_image);
void FindBoundingBox(cv::Mat image_RGB, cv::Mat image_depth);
void CloudViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2);
void Merge(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> color_vector);
void vizPointCloud(cv::Mat image_RGB, pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud);
pcl::PointXYZ GetUnitY(pcl::PointXYZ origin, pcl::PointXYZ unit_x, Plane::Plane_model plane);
pcl::PointCloud<pcl::PointXYZ> makeSyntheticCloud(std::string color_string);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_dominant_plane_projection(Plane::Plane_model plane, cv::Mat pCloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr back_projection(cv::Mat image, std::string string);
pcl::PointCloud<pcl::PointXYZ>::Ptr makeForm (pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr cv2pcl(cv::Mat image);

bool colorCheck(cv::Mat color_image, cv::Mat& color_pCloud, cv::Mat pCloud);

double getDistant(pcl::PointXYZ pt_1, pcl::PointXYZ pt_2);
int getStep(float number);

void BoundingBox(Plane::Plane_model dominant_plane, cv::Mat pCloud_outlier, std::string color_string);

int GetPose(std::string color_string, pcl::PointCloud<pcl::PointXYZ> color_synthetic, std::vector<double> &array);
pcl::PointCloud<pcl::PointXYZ> Rotate(pcl::PointCloud<pcl::PointXYZ> source, int x, int y, int z, int division, Eigen::Matrix4f &trans_rot);
Eigen::Matrix4f MatrixMul(Eigen::Matrix4f obj_mat, Eigen::Matrix4f to_mat);
double ICP(pcl::PointCloud<pcl::PointXYZ> source, pcl::PointCloud<pcl::PointXYZ> target, Eigen::Matrix4f &trans_icp);
std::vector<double> GetCenter(pcl::PointCloud<pcl::PointXYZ> point_cloud);
pcl::PointCloud<pcl::PointXYZ> Translation(pcl::PointCloud<pcl::PointXYZ> source, std::vector<double> move, Eigen::Matrix4f &translation);
double alignment(pcl::PointCloud<pcl::PointXYZ> source, pcl::PointCloud<pcl::PointXYZ> target, Eigen::Matrix4f &trans);

cv::Scalar fitEllipseColor = Scalar(255,  0,  0);
cv::Mat processImage(cv::Mat image, std::string string);
int sliderPos = 70;

void GetBoundingBox();  
void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
void DepthCallback(const sensor_msgs::ImageConstPtr& msg);

void Get_RGB(cv_bridge::CvImagePtr& cv_ptr);
void Get_Depth(cv_bridge::CvImagePtr& cv_ptr);

bool Red_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr red_cloud, pcl::PointXYZ point_one, pcl::PointXYZ point_two);
bool Orange_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr orange_cloud, pcl::PointXYZ point_one, pcl::PointXYZ point_two);
pcl::PointXYZ GivePoint(std::vector<std::vector<float> > vector, pcl::PointXYZ origin, double x, double y, double z);
bool btwCheck(pcl::PointXYZ point, pcl::PointXYZ start, pcl::PointXYZ end);
bool Green_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr green_cloud, pcl::PointXYZ point_one, pcl::PointXYZ point_two);
void ArrayInit(int *array, int num);
bool Blue_btwCheck(pcl::PointXYZ point, pcl::PointXYZ cloud_point);
void Blue_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr blue_cloud, pcl::PointXYZ& dier, pcl::PointXYZ& diyi_one, pcl::PointXYZ& diyi_two);
void Purple_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr purple_cloud, pcl::PointXYZ& dier, pcl::PointXYZ& diyi_one, pcl::PointXYZ& diyi_two);
void Brown_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr brown_cloud, pcl::PointXYZ& dier, pcl::PointXYZ& diyi_one, pcl::PointXYZ& diyi_two);


pcl::PointCloud<pcl::PointXYZ> GetRedSynthetic();
pcl::PointCloud<pcl::PointXYZ> GetOrangeSynthetic();
pcl::PointCloud<pcl::PointXYZ> GetYellowSynthetic();
pcl::PointCloud<pcl::PointXYZ> GetGreenSynthetic();
pcl::PointCloud<pcl::PointXYZ> GetBlueSynthetic();
pcl::PointCloud<pcl::PointXYZ> GetPurpleSynthetic();
pcl::PointCloud<pcl::PointXYZ> GetBrownSynthetic();

int number;

cv::Mat image_RGB;
cv::Mat image_depth;

pcl::PointXYZ origin;
pcl::PointXYZ unit_x;
pcl::PointXYZ unit_y;

std::vector<pcl::PointXYZ> keypoints = std::vector<pcl::PointXYZ> (4);
std::vector<pcl::PointXYZ> red_keypoints = std::vector<pcl::PointXYZ> (4);
std::vector<pcl::PointXYZ> orange_keypoints = std::vector<pcl::PointXYZ> (4);
std::vector<pcl::PointXYZ> yellow_keypoints = std::vector<pcl::PointXYZ> (4);
std::vector<pcl::PointXYZ> green_keypoints = std::vector<pcl::PointXYZ> (4);
std::vector<pcl::PointXYZ> blue_keypoints = std::vector<pcl::PointXYZ> (4);
std::vector<pcl::PointXYZ> purple_keypoints = std::vector<pcl::PointXYZ> (4);
std::vector<pcl::PointXYZ> brown_keypoints = std::vector<pcl::PointXYZ> (4);

cv::Point rect_points[4];
std::vector<Point2f> rectpoints = std::vector<Point2f> (4);

std::vector<float> unit_vector = std::vector<float> (3);

float a;
float b;
float c;
float d;

bool minus_x;
bool minus_y; 

int dim_w;
int dim_h;

int min_x = 0;
int min_y = 0;

double block_height;

double red_block_height;
double orange_block_height;
double yellow_block_height;
double green_block_height;
double blue_block_height;
double purple_block_height;
double brown_block_height;

cv::Mat red_pCloud;
cv::Mat orange_pCloud;
cv::Mat yellow_pCloud;
cv::Mat green_pCloud;
cv::Mat blue_pCloud;
cv::Mat purple_pCloud;
cv::Mat brown_pCloud;

bool bool_red = false;
bool bool_orange = false;
bool bool_yellow = false;
bool bool_green = false;
bool bool_blue = false;
bool bool_purple = false;
bool bool_brown = false;

// color cloud

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> red_vector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> (2);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> orange_vector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> (2);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> yellow_vector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> (2);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> green_vector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> (2);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> blue_vector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> (2);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> purple_vector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> (2);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> brown_vector = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> (2);
pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud (new pcl::PointCloud<pcl::PointXYZ>);

// row declaration

ros::Publisher pub_red;
ros::Publisher pub_orange;
ros::Publisher pub_yellow;
ros::Publisher pub_green;
ros::Publisher pub_blue;
ros::Publisher pub_purple;
ros::Publisher pub_brown;

ros::Subscriber sub_color;
ros::Subscriber sub_depth;

struct Rectangle
{   
    cv::Point u_l;
    cv::Point u_r;
    cv::Point l_l;
    cv::Point l_r;

    Rectangle(int x=0, int y=0)
    {   
        int x_val = x;
        int y_val = y;

        u_l.x = x_val, u_l.y = y_val;
        u_r.x = x_val + dim_w, u_r.y = y_val;
        l_l.x = x_val, l_l.y =  y_val + dim_h;
        l_r.x = x_val + dim_w, l_r.y = y_val + dim_h;
    }
};


class canvas{
public:
    bool setupQ;
    cv::Point origin;
    cv::Point corner;
    int minDims,maxDims;
    double scale;
    int rows, cols;
    cv::Mat img;
    void init(int minD, int maxD){
        // Initialise the canvas with minimum and maximum rows and column sizes.
        minDims = minD; maxDims = maxD;
        origin = cv::Point(0,0);
        corner = cv::Point(0,0);
        scale = 1.0;
        rows = 0;
        cols = 0;
        setupQ = false;
    }
    void stretch(cv::Point2f min, cv::Point2f max){
        // Stretch the canvas to include the points min and max.
        if(setupQ){
            if(corner.x < max.x){corner.x = (int)(max.x + 1.0);};
            if(corner.y < max.y){corner.y = (int)(max.y + 1.0);};
            if(origin.x > min.x){origin.x = (int) min.x;};
            if(origin.y > min.y){origin.y = (int) min.y;};
        } else {
            origin = cv::Point((int)min.x, (int)min.y);
            corner = cv::Point((int)(max.x + 1.0), (int)(max.y + 1.0));
        }
        int c = (int)(scale*((corner.x + 1.0) - origin.x));
        if(c<minDims){
            scale = scale * (double)minDims/(double)c;
        } else {
            if(c>maxDims){
                scale = scale * (double)maxDims/(double)c;
            }
        }
        int r = (int)(scale*((corner.y + 1.0) - origin.y));
        if(r<minDims){
            scale = scale * (double)minDims/(double)r;
        } else {
            if(r>maxDims){
                scale = scale * (double)maxDims/(double)r;
            }
        }
        cols = (int)(scale*((corner.x + 1.0) - origin.x));
        rows = (int)(scale*((corner.y + 1.0) - origin.y));
        setupQ = true;
    }
    void stretch(vector<Point2f> pts)
    {   // Stretch the canvas so all the points pts are on the canvas.
        cv::Point2f min = pts[0];
        cv::Point2f max = pts[0];
        for(size_t i=1; i < pts.size(); i++){
            Point2f pnt = pts[i];
            if(max.x < pnt.x){max.x = pnt.x;};
            if(max.y < pnt.y){max.y = pnt.y;};
            if(min.x > pnt.x){min.x = pnt.x;};
            if(min.y > pnt.y){min.y = pnt.y;};
        };
        stretch(min, max);
    }
    void stretch(cv::RotatedRect box)
    {   // Stretch the canvas so that the rectangle box is on the canvas.
        cv::Point2f min = box.center;
        cv::Point2f max = box.center;
        cv::Point2f vtx[4];
        box.points(vtx);
        for( int i = 0; i < 4; i++ ){
            cv::Point2f pnt = vtx[i];
            if(max.x < pnt.x){max.x = pnt.x;};
            if(max.y < pnt.y){max.y = pnt.y;};
            if(min.x > pnt.x){min.x = pnt.x;};
            if(min.y > pnt.y){min.y = pnt.y;};
        }
        stretch(min, max);
    }
    void drawEllipseWithBox(cv::RotatedRect box, cv::Scalar color, int lineThickness)
    {
        if(img.empty()){
            stretch(box);
            img = cv::Mat::zeros(rows,cols,CV_8UC3);
        }
        box.center = scale * cv::Point2f(box.center.x - origin.x, box.center.y - origin.y);
        box.size.width  = (float)(scale * box.size.width);
        box.size.height = (float)(scale * box.size.height);
        //ellipse(img, box, color, lineThickness, LINE_AA);
        Point2f vtx[4];
        box.points(vtx);
        for( int j = 0; j < 4; j++ ){
            line(img, vtx[j], vtx[(j+1)%4], color, lineThickness, LINE_AA);
        }
    }
    void drawPoints(vector<Point2f> pts, cv::Scalar color)
    {
        if(img.empty()){
            stretch(pts);
            img = cv::Mat::zeros(rows,cols,CV_8UC3);
        }
        for(size_t i=0; i < pts.size(); i++){
            Point2f pnt = scale * cv::Point2f(pts[i].x - origin.x, pts[i].y - origin.y);
            img.at<cv::Vec3b>(int(pnt.y), int(pnt.x))[0] = (uchar)color[0];
            img.at<cv::Vec3b>(int(pnt.y), int(pnt.x))[1] = (uchar)color[1];
            img.at<cv::Vec3b>(int(pnt.y), int(pnt.x))[2] = (uchar)color[2];
        };
    }
    void drawLabels( std::vector<std::string> text, std::vector<cv::Scalar> colors)
    {
        if(img.empty()){
            img = cv::Mat::zeros(rows,cols,CV_8UC3);
        }
        int vPos = 0;
        for (size_t i=0; i < text.size(); i++) {
            cv::Scalar color = colors[i];
            std::string txt = text[i];
            Size textsize = getTextSize(txt, FONT_HERSHEY_COMPLEX, 1, 1, 0);
            vPos += (int)(1.3 * textsize.height);
            Point org((img.cols - textsize.width), vPos);
            cv::putText(img, txt, org, FONT_HERSHEY_COMPLEX, 1, color, 1, LINE_8);
        }
    }
    void MergeImage( cv::Mat image )
    {
      for (int i = 0; i < image.rows; i++)
      {
        for (int j = 0; j < image.cols; j++)
        { 
          if (image.at<uchar>(i, j) == 0)
          {}
          else
          {
            img.at<cv::Vec3b>(i, j)[0] = 255;
            img.at<cv::Vec3b>(i, j)[1] = 255;
            img.at<cv::Vec3b>(i, j)[2] = 255;
          }
        }
      }
    }
};


namespace Plane {
    float DominantPlane::fx, DominantPlane::fy, DominantPlane::cx, DominantPlane::cy;
    float DominantPlane::DepthMapFactor;
    float DominantPlane::Distance_threshold;
    int DominantPlane::max_iter;
    int DominantPlane::N_pcds;
    int DominantPlane::frame_id;
    int DominantPlane::N_inlier;
    int DominantPlane::width;
    int DominantPlane::height;
    std::vector<cv::Vec3f> pcd_concat;
    cv::Mat DominantPlane::mK = cv::Mat::zeros(3, 3, CV_32FC1);
    cv::Mat DominantPlane::Pointcloud;

    DominantPlane::DominantPlane(float _fx, float _fy, float _cx, float _cy, float _DepthMapFactor, float _Distance_thresh,  int _max_iter,
                                 int _width, int _height) {
        mK.at<float>(0,0) = _fx;
        mK.at<float>(1,1) = _fy;
        mK.at<float>(0,2) = _cx;
        mK.at<float>(1,2) = _cy;
        fx = _fx;
        fy = _fy;
        cx = _cx;
        cy = _cy;
        DepthMapFactor = _DepthMapFactor;
        max_iter = _max_iter;
        frame_id = 0;
        N_inlier = 0;
        width = _width;
        height = _height;
        Distance_threshold = _Distance_thresh;
        Pointcloud = cv::Mat::zeros(height, width, CV_32FC3);
    }

    cv::Mat DominantPlane::Depth2pcd(cv::Mat &depth){
        float X, Y, Z;
        for (int y = 0; y < depth.rows; y++)
        {
            for(int x = 0; x < depth.cols; x++)
            {
                Z = depth.at<uint16_t>(y,x) / DepthMapFactor;
                if(Z > 0)
                {
                    X = (x - cx) * Z / fx;
                    Y = (y - cy) * Z / fy;
                    Pointcloud.at<cv::Vec3f>(y, x) = cv::Vec3f(X, Y, Z);
                    pcd_concat.push_back(cv::Vec3f(X,Y,Z));
                }
                else{
                     Pointcloud.at<cv::Vec3f>(y, x) = cv::Vec3f(0.f, 0.f, 0.f);
                }
            }
        }

        return Pointcloud;
    }
    

    
    Plane_model DominantPlane::RunRansac(cv::Mat &pcd_inlier, cv::Mat point_cloud) {
        unsigned long rand_idx;
        float cost = 0;
        float best_cost = std::numeric_limits<float>::infinity();
        unsigned int best_N_inlier = 0;
        Plane_model best_plane;
        std::srand(time(NULL));

        Plane_model plane;

        for (int i = 0; i < max_iter; i++)
        {
            cv::Mat pcd_temp_inlier = cv::Mat::zeros(height, width, CV_32FC3);
            std::vector<cv::Vec3f> sampled_pcd;
            for(int j =0; j < 3; j++)
            {
                rand_idx = int(pcd_concat.size() * (std::rand()/(double)RAND_MAX));
                sampled_pcd.push_back(pcd_concat.at(rand_idx));
            }
            FindPlaneFromSampledPcds(sampled_pcd, plane);
            compute_inlier_cost(plane, pcd_temp_inlier, point_cloud);
            cost = plane.avg_distance / N_inlier;
            if(best_cost > cost)
            {
                best_cost = cost;
                best_plane = plane;
                pcd_inlier = pcd_temp_inlier;
            }
            N_inlier = 0;
            pcd_temp_inlier.release();
            sampled_pcd.clear();
        }

        

        ResetValue();
        return best_plane;
    }
    

    void DominantPlane::FindPlaneFromSampledPcds(std::vector<cv::Vec3f> &sampled_pcds, Plane_model& plane)
    {
        cv::Vec3f A = sampled_pcds[0], B = sampled_pcds[1], C = sampled_pcds[2];
        // cout<< "A" << A << "B" << B << endl;
        cv::Vec3f AB = A - B;
        cv::Vec3f BC = B - C;
        cv::Vec3f normal = AB.cross(BC);

        plane.a = normal[0], plane.b = normal[1], plane.c = normal[2];
        plane.d = -plane.a * A[0] - plane.b * A[1] - plane.c * A[2];
        plane.denominator = sqrt(pow(plane.a, 2) + pow(plane.b, 2) + pow(plane.c, 2));
    }

    void DominantPlane::compute_inlier_cost(Plane_model& plane, cv::Mat& pcd_inlier, cv::Mat& pcd_input)
    {
        float dist_all = 0;
        for (int y = 0; y < pcd_input.rows; y++)
        {
            for(int x = 0; x < pcd_input.cols; x++)
            {
                cv::Vec3f temp = pcd_input.at<cv::Vec3f>(y,x);
                float dist = abs(plane.a * temp[0] + plane.b * temp[1] + plane.c * temp[2] + plane.d) / plane.denominator;
                if(dist < Distance_threshold)
                {
                    pcd_inlier.at<cv::Vec3f>(y,x) = temp;
                    N_inlier++;
                    dist_all += dist;
                }
                else
                {
                    pcd_inlier.at<cv::Vec3f>(y,x) = cv::Vec3f(0.f, 0.f, 0.f);
                }

            }
        }
        plane.avg_distance = dist_all / N_inlier;
    }
}

cv::Mat DistantPlaneSegmentation(Plane::Plane_model plane, cv::Mat pCloud_outlier, float distance)
{
    /*
    object_cloud->width  = red_cloud->width;
    object_cloud->height = red_cloud->height;
    object_cloud->points.resize (object_cloud->width * object_cloud->height);
    */
    cv::Mat pcd_object = cv::Mat::zeros(pCloud_outlier.rows, pCloud_outlier.cols, CV_32FC3);

    float Threshold = 0.001;

    for (int i=0; i<pcd_object.rows; i++)
    {
        for(int j=0; j<pcd_object.cols; j++)
        {
            float x = pCloud_outlier.at<cv::Vec3f>(i, j)[0];
            float y = pCloud_outlier.at<cv::Vec3f>(i, j)[1];
            float z = pCloud_outlier.at<cv::Vec3f>(i, j)[2];

            float dist = abs(plane.a * x + plane.b * y + plane.c * z + plane.d) / plane.denominator;

            if ( -Threshold + distance < dist && dist < Threshold + distance)
            {
                pcd_object.at<cv::Vec3f>(i, j) = pCloud_outlier.at<cv::Vec3f>(i, j);

            }
        }

    }
    return pcd_object;
    
}

void Segmentation(cv::Mat& image_RGB, cv::Mat& image_depth, Plane::Plane_model &best_plane, cv::Mat &pCloud_outlier)
{
    float fx = 615.6707153320312;
    float fy = 615.962158203125;
    float cx = 328.0010681152344;
    float cy = 241.31031799316406;

    float scale = 1000;
    float Distance_theshold = 0.0005;

    int width = 640;
    int height = 480;
    int max_iter = 300;

    Plane::DominantPlane plane(fx, fy, cx, cy, scale, Distance_theshold, max_iter, width, height);
    
    cv::Mat pCloud_inlier(height, width, CV_32FC3); 
    cv::Mat pCloud(height, width, CV_32FC3);
    
    pCloud = plane.Depth2pcd(image_depth);

    Plane::Plane_model dominant_plane;
    dominant_plane = plane.RunRansac(pCloud_inlier, pCloud);

    best_plane = dominant_plane;

    for (int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            pCloud_outlier.at<cv::Vec3f>(y, x) = pCloud.at<cv::Vec3f>(y, x) - pCloud_inlier.at<cv::Vec3f>(y, x);
        }
    }
}

void Color_Segmentation(cv::Mat& RGB_image, cv::Mat& pCloud)
{   
    Mat im = RGB_image.clone();
    Mat hsv_image;
    cvtColor(im, hsv_image, COLOR_BGR2HSV); // convert BGR2HSV

    Mat lower_red_hue;
    Mat upper_red_hue;

    inRange(hsv_image, Scalar(0, 100, 50), Scalar(3, 255, 255), lower_red_hue);
    inRange(hsv_image, Scalar(150, 100, 50), Scalar(179, 255, 255), upper_red_hue);

    Mat red;
    addWeighted(lower_red_hue, 1.0, upper_red_hue, 1.0, 0.0, red);

        // Threshold for orange color
    Mat orange;
    inRange(hsv_image, Scalar(4, 100, 30), Scalar(10, 255,200), orange);

        // Threshold for yellow color
    Mat yellow;
    inRange(hsv_image, Scalar(11, 200, 50), Scalar(25, 255, 255), yellow);

        // Threshold for green color
    Mat green;
    inRange(hsv_image, Scalar(50, 40, 40), Scalar(90, 255, 255), green);

        // Threshold for blue color
    Mat blue;
    inRange(hsv_image, Scalar(102, 100, 40), Scalar(130, 255, 255), blue);

        // Threshold for purple color. the hue for purple is the same as red. Only difference is value.
    Mat purple;
    inRange(hsv_image, Scalar(131, 50, 30), Scalar(179, 255, 140), purple);

        // Threshold for brown color. the hue for brown is the same as red and orange. Only difference is value.
    Mat brown;
    inRange(hsv_image, Scalar(0, 50, 10), Scalar(15, 200, 100), brown);

    bool_red = colorCheck(red, red_pCloud, pCloud);
    bool_orange = colorCheck(orange, orange_pCloud, pCloud);
    bool_yellow = colorCheck(yellow, yellow_pCloud, pCloud);
    bool_green = colorCheck(green, green_pCloud, pCloud);
    bool_blue = colorCheck(blue, blue_pCloud, pCloud);
    bool_purple = colorCheck(purple, purple_pCloud, pCloud);
    bool_brown = colorCheck(brown, brown_pCloud, pCloud);
}

bool colorCheck(cv::Mat color_image, cv::Mat& color_pCloud, cv::Mat pCloud)
{   
    bool bool_color = false;
    color_pCloud = pCloud.clone();

    int color_num = 0;
    for (int i = 0; i < color_image.rows; i ++)
    {
        for (int j = 0; j < color_image.cols; j++)
        {
            if(color_image.at<uchar>(i, j) == 255)
                { color_num++; }
        }
    }
    if(color_num > 0 && color_num < color_image.rows * color_image.cols)
    {   
        bool_color = true;
        for (int i = 0; i < color_image.rows; i++)
        {
            for (int j = 0; j < color_image.cols; j++)
            {
                if (color_image.at<uchar>(i, j) == 0) // ******choose color****** //
                {
                    for (int k = 0; k < 3 ; k++)
                    color_pCloud.at<cv::Vec3f>(i, j)[k] = 0;
                } 
            }
        }
    }

    return bool_color;
}

void LoadImages(const string &association_dir, vector<string> &FilenameRGB, vector<string> &FilenameDepth)
{
    ifstream association;
    association.open(association_dir.c_str());
    while(!association.eof())
    {
        string s;
        getline(association, s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string sRGB, sDepth;
            ss >> sRGB;
            FilenameRGB.push_back(sRGB);
            ss >> sDepth;
            FilenameDepth.push_back(sDepth);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"matching_algorithm");
    ros::NodeHandle nh;

    //Mat imRGB, imDepth
    sub_color = nh.subscribe("/camera/color/image_raw", 1, ImageCallback);
    sub_depth = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, DepthCallback);

    pub_red = nh.advertise<matching_algorithm::Data2d>("/color_matrix/red_matrix", 1);
    pub_orange = nh.advertise<matching_algorithm::Data2d>("/color_matrix/orange_matrix", 1);
    pub_yellow = nh.advertise<matching_algorithm::Data2d>("/color_matrix/yellow_matrix", 1);
    pub_green = nh.advertise<matching_algorithm::Data2d>("/color_matrix/green_matrix", 1);
    pub_blue = nh.advertise<matching_algorithm::Data2d>("/color_matrix/blue_matrix", 1);
    pub_purple = nh.advertise<matching_algorithm::Data2d>("/color_matrix/purple_matrix", 1);
    pub_brown = nh.advertise<matching_algorithm::Data2d>("/color_matrix/brown_matrix", 1);

    ros::spin();
    return 0;

}

void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{   
    cv_bridge::CvImagePtr cv_ptrRGB;

    cv_ptrRGB = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Get_RGB(cv_ptrRGB);
    /*
    try
    {
        cv_ptrRGB = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Get_RGB(cv_ptrRGB);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    */

}

void DepthCallback(const sensor_msgs::ImageConstPtr& msg)
{   

    cv_bridge::CvImagePtr cv_ptrD;
    cv_ptrD = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    Get_Depth(cv_ptrD);

    if (image_RGB.empty() || image_depth.empty()) {
        cerr << endl << "Failed to load image at: " << endl;
        return;
    } else {//if(number%2 == 0){
       
        GetBoundingBox();
        
    }
    /*
    try {
        cv_ptrD = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

        cout << " Come on! " << endl;
        Get_Depth(cv_ptrD);

        cout << " What?! " << endl;

        if (image_RGB.empty() || image_depth.empty()) {
            cerr << endl << "Failed to load image at: " << endl;
            return;
        } else {//if(number%2 == 0){
           
            GetBoundingBox();
            
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    */
}


void Get_RGB (cv_bridge::CvImagePtr& cv_ptr)
{
    image_RGB = cv_ptr->image;
    if(!image_RGB.empty())
    {   
        ROS_INFO("RGB image height = %d, width = %d", image_RGB.rows, image_RGB.cols);
        cv::imshow("RGB image", image_RGB);
        cv::waitKey(1);
    }
}

void Get_Depth (cv_bridge::CvImagePtr& cv_ptr)
{
    image_depth = cv_ptr->image;
    if(!image_depth.empty())
    {   
        number++;
        ROS_INFO("Depth image height = %d, width = %d", image_depth.rows, image_depth.cols);
        cv::imshow("Depth image", image_depth);
        cv::waitKey(1);
    }
}


void GetBoundingBox()  
{ 

    Plane::Plane_model dominant_plane;

    cv::Mat pCloud_outlier = cv::Mat::zeros(image_RGB.rows, image_RGB.cols, CV_32FC3);

    Segmentation(image_RGB, image_depth, dominant_plane, pCloud_outlier);

    Color_Segmentation(image_RGB, pCloud_outlier);

    if(bool_red)
        { BoundingBox(dominant_plane, red_pCloud, "red"); }
    if(bool_orange)
        { BoundingBox(dominant_plane, orange_pCloud, "orange"); }
    if(bool_yellow)
        { BoundingBox(dominant_plane, yellow_pCloud, "yellow"); }
    if(bool_green)
        { BoundingBox(dominant_plane, green_pCloud, "green"); }
    if(bool_blue)
        { BoundingBox(dominant_plane, blue_pCloud, "blue"); }
    if(bool_purple)
        { BoundingBox(dominant_plane, purple_pCloud, "purple"); }
    if(bool_brown)
        { BoundingBox(dominant_plane, brown_pCloud, "brown"); }
    
    pcl::PointCloud<pcl::PointXYZ> red_synthetic;
    pcl::PointCloud<pcl::PointXYZ> orange_synthetic;
    pcl::PointCloud<pcl::PointXYZ> yellow_synthetic;
    pcl::PointCloud<pcl::PointXYZ> green_synthetic;
    pcl::PointCloud<pcl::PointXYZ> blue_synthetic;
    pcl::PointCloud<pcl::PointXYZ> purple_synthetic;
    pcl::PointCloud<pcl::PointXYZ> brown_synthetic;

    std::vector<double> red_array = std::vector<double> (16);
    std::vector<double> orange_array = std::vector<double> (16);
    std::vector<double> yellow_array = std::vector<double> (16);
    std::vector<double> green_array = std::vector<double> (16);
    std::vector<double> blue_array = std::vector<double> (16);
    std::vector<double> purple_array = std::vector<double> (16);
    std::vector<double> brown_array = std::vector<double> (16);

    
    if(bool_red)
        { red_synthetic = makeSyntheticCloud("red"); }
    if(bool_orange)
        { orange_synthetic = makeSyntheticCloud("orange"); }
    if(bool_yellow)
        { yellow_synthetic = makeSyntheticCloud("yellow"); }
    if(bool_green)
        { green_synthetic = makeSyntheticCloud("green"); }
    if(bool_blue)
        { blue_synthetic = makeSyntheticCloud("blue"); }
    if(bool_purple)
        { purple_synthetic = makeSyntheticCloud("purple"); }
    if(bool_brown)
        { brown_synthetic = makeSyntheticCloud("brown"); }
    

    if(bool_red && !red_synthetic.empty())
    { 
        if(red_synthetic.points.size() == 3)
        {
            if(GetPose("red", red_synthetic, red_array) == 0)
            {   
                matching_algorithm::Data2d msg;
                msg.data.resize(4);
                for (int i = 0; i < 4; i++)
                {   msg.data[i].data.resize(4);
                    for (int j = 0; j < 4; j++)
                    { msg.data[i].data[j] = red_array[j + i*4]; } 
                }
                pub_red.publish(msg);  
            }
        }
    }
    if(bool_orange && !orange_synthetic.empty())
    {   
        if(orange_synthetic.points.size() == 4)
        {
            if(GetPose("orange", orange_synthetic, orange_array) == 0)
            {
                matching_algorithm::Data2d msg;
                msg.data.resize(4);
                for (int i = 0; i < 4; i++)
                {   msg.data[i].data.resize(4);
                    for (int j = 0; j < 4; j++)
                    { msg.data[i].data[j] = orange_array[j + i*4]; } 
                }
                pub_red.publish(msg);   
            }
        }    
    }
    if(bool_yellow && !yellow_synthetic.empty())
    {   
        if(yellow_synthetic.points.size() == 4)
        {
            if(GetPose("yellow", yellow_synthetic, yellow_array) == 0)
            {   

                matching_algorithm::Data2d msg;
                msg.data.resize(4);
                for (int i = 0; i < 4; i++)
                {   msg.data[i].data.resize(4);
                    for (int j = 0; j < 4; j++)
                    { msg.data[i].data[j] = yellow_array[j + i*4]; } 
                }
                pub_red.publish(msg);  
                
            }
        }
    }
    if(bool_green && !green_synthetic.empty())
    { 
        if(green_synthetic.points.size() == 4)
        {
            if(!GetPose("green", green_synthetic, green_array) == 1)
            {   
                
                matching_algorithm::Data2d msg;
                msg.data.resize(4);
                for (int i = 0; i < 4; i++)
                {   msg.data[i].data.resize(4);
                    for (int j = 0; j < 4; j++)
                    { msg.data[i].data[j] = green_array[j + i*4]; } 
                }
                pub_red.publish(msg);  
            }
        }
    }
    if(bool_blue && !blue_synthetic.empty())
    {   
        if(blue_synthetic.points.size() == 4)
        {
            if(GetPose("blue", blue_synthetic, blue_array) == 0)
            {

                matching_algorithm::Data2d msg;
                msg.data.resize(4);
                for (int i = 0; i < 4; i++)
                {   msg.data[i].data.resize(5);
                    for (int j = 0; j < 4; j++)
                    { msg.data[i].data[j] = blue_array[j + i*4]; } 
                    // msg.data[i].data[4] = "\n";
                }
                pub_red.publish(msg);   

            }
        }
    }
    if(bool_purple && !purple_synthetic.empty())
    {   
        if(purple_synthetic.points.size() == 4)
        {
            if(GetPose("purple", purple_synthetic, purple_array) == 0 && purple_array.size() == 16)
            {
                matching_algorithm::Data2d msg;
                msg.data.resize(4);
                for (int i = 0; i < 4; i++)
                {   msg.data[i].data.resize(4);
                    for (int j = 0; j < 4; j++)
                    { msg.data[i].data[j] = purple_array[j + i*4]; } 
                }
                pub_red.publish(msg);  
            }
        }
    }
    if(bool_brown && !brown_synthetic.empty())
    { 
        if(brown_synthetic.points.size() == 4)
        {
            if((GetPose("brown", brown_synthetic, brown_array)) == 0)
            {   

                matching_algorithm::Data2d msg;
                msg.data.resize(4);
                for (int i = 0; i < 4; i++)
                {   msg.data[i].data.resize(4);
                    for (int j = 0; j < 4; j++)
                    { msg.data[i].data[j] = brown_array[j + i*4]; } 
                }
                pub_red.publish(msg);  
            }
        }
    }
    
    
    if(bool_red)
        { Merge(red_vector); }
    if(bool_orange)
        { Merge(orange_vector); }
    if(bool_yellow)
        { Merge(yellow_vector); }
    if(bool_green)
        { Merge(green_vector); }
    if(bool_blue)
        { Merge(blue_vector); }
    if(bool_purple)
        { Merge(purple_vector); }
    if(bool_brown)
        { Merge(brown_vector); }

    vizPointCloud(image_RGB, merged_cloud);
    merged_cloud->points.clear();

}


void BoundingBox(Plane::Plane_model dominant_plane, cv::Mat pCloud_outlier, std::string color_string)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    outlier_cloud = cv2pcl(pCloud_outlier);

    block_height = GetDist(dominant_plane, outlier_cloud); 
    
    if(color_string == "red")
        { red_block_height = block_height;}
    else if(color_string == "orange")
        { orange_block_height = block_height;}
    else if(color_string == "yellow")
        { yellow_block_height = block_height;}
    else if(color_string == "green")
        { green_block_height = block_height;}
    else if(color_string == "blue")
        { blue_block_height = block_height;}
    else if(color_string == "purple")
        { purple_block_height = block_height;}
    else
        { brown_block_height = block_height;}

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_projected_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_projected_cloud = pcl_dominant_plane_projection(dominant_plane, pCloud_outlier);

    cv::Mat projected_image = get_projected_image(dominant_plane, pcl_projected_cloud);

    cv::Mat box_image = processImage(projected_image, color_string);
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    box_cloud = back_projection(box_image, color_string);

    pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    bbox_cloud = makeForm(box_cloud);


    if(color_string == "red")
    {
        red_vector.at(0) = outlier_cloud;
        red_vector.at(1) = bbox_cloud;
    }
    else if(color_string == "orange")
    {
        orange_vector.at(0) = outlier_cloud;
        orange_vector.at(1) = bbox_cloud;        
    }
    else if(color_string == "yellow")
    {
        yellow_vector.at(0) = outlier_cloud;
        yellow_vector.at(1) = bbox_cloud;        
    }
    else if(color_string == "green")
    {
        green_vector.at(0) = outlier_cloud;
        green_vector.at(1) = bbox_cloud;        
    }
    else if(color_string == "blue")
    {
        blue_vector.at(0) = outlier_cloud;
        blue_vector.at(1) = bbox_cloud;        
    }
    else if(color_string == "purple")
    {
        purple_vector.at(0) = outlier_cloud;
        purple_vector.at(1) = bbox_cloud;        
    }
    else
    {
        brown_vector.at(0) = outlier_cloud;
        brown_vector.at(1) = bbox_cloud;        
    }
}


pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_dominant_plane_projection(Plane::Plane_model plane, cv::Mat pCloud)
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_projected_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    float a = plane.a; 
    float b = plane.b; 
    float c = plane.c; 
    float d = plane.d;

    for(int i = 0; i < pCloud.rows; i++)
    {
        for(int j = 0; j < pCloud.cols; j++)
        {
            if (pCloud.at<cv::Vec3f>(i, j)[0] == 0 && pCloud.at<cv::Vec3f>(i, j)[1] == 0 && pCloud.at<cv::Vec3f>(i, j)[2] == 0)
            {}
            else   
            {
                double x = pCloud.at<cv::Vec3f>(i, j)[0];
                double y = pCloud.at<cv::Vec3f>(i, j)[1];
                double z = pCloud.at<cv::Vec3f>(i, j)[2];
        
                double t = (- a*x - b*y - c*z - d)/(a*a + b*b + c*c);

                pcl::PointXYZ point;
                point.x = x + t*a;
                point.y = y + t*b;
                point.z = z + t*c;

                pcl_projected_cloud->points.push_back(point);
            }
        }
    }

    return pcl_projected_cloud;
}


cv::Mat get_projected_image(Plane::Plane_model plane, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{       
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (point_cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
    
    double x_min = 100;
    double y_x;
    double z_x;

    double y_min = 100;
    double x_y;
    double z_y;

    double z_min = 100;
    double x_z;
    double y_z;

    int num = 0;

    for (int i = 0; i < cloud_filtered->points.size(); i++)
    {
        pcl::PointXYZ point = cloud_filtered->points[i];

        double x = point.x;
        double y = point.y;
        double z = point.z;

        if ( x < x_min )
        {
            x_min = x;
            y_x = y;
            z_x = z;
        }
        if ( y < y_min )
        {
            y_min = y;
            x_y = x;
            z_y = z;
        }
        if ( z < z_min )
        {
            z_min = z;
            x_z = x;
            y_z = y;
        }

        
    }

    origin.x = x_min;
    origin.y = y_min; 
    origin.z = (- x_min*plane.a - y_min*plane.b - plane.d) / plane.c; 

    pcl::PointXYZ x_dir;
    x_dir.x = x_min - origin.x;
    x_dir.y = y_x - origin.y;
    x_dir.z = z_x - origin.z;

    double norm = sqrt(pow(x_dir.x, 2) + pow(x_dir.y, 2) + pow(x_dir.z, 2)) + 1e-10;

    unit_x.x = x_dir.x / norm;
    unit_x.y = x_dir.y / norm;
    unit_x.z = x_dir.z / norm;

    a = plane.a;
    b = plane.b;
    c = plane.c;
    d = plane.d;  

    if(c > 0)
    {
        a = - a; 
        b = - b; 
        c = - c; 
        d = - d; 
    }

    unit_y = GetUnitY(origin, unit_x, plane);
 
    
    double max_x = 0;
    double max_y = 0;
    double min_x = 10;
    double min_y = 10;

    for (int i = 0; i < cloud_filtered->points.size(); i++)
    {
        pcl::PointXYZ point = cloud_filtered->points[i];
        point.x -= origin.x;
        point.y -= origin.y;
        point.z -= origin.z;

        int x = (unit_x.x*point.x + unit_x.y*point.y + unit_x.z*point.z)*1000;
        int y = (unit_y.x*point.x + unit_y.y*point.y + unit_y.z*point.z)*1000;

        if(max_x < x) {max_x = x;};
        if(max_y < y) {max_y = y;};
        if(min_x > x) {min_x = x;};
        if(min_y > y) {min_y = y;};
    }

    if(((max_x - min_x) > 0) && ((max_y - min_y) > 0) && ((max_y - min_y) < 1e+5))
    { 
        cv::Mat projected_image = cv::Mat::zeros(max_x - min_x + 10, max_y - min_y + 10, CV_8UC1); 
        for (int i = 0; i < cloud_filtered->points.size(); i++)
        {
            pcl::PointXYZ point = cloud_filtered->points[i];
            point.x -= origin.x;
            point.y -= origin.y;
            point.z -= origin.z;

            int x = (unit_x.x*point.x + unit_x.y*point.y + unit_x.z*point.z)*1000 - min_x + 5;
            int y = (unit_y.x*point.x + unit_y.y*point.y + unit_y.z*point.z)*1000 - min_y + 5;
            
 
            
            if(x > 0 && y > 0)
                { projected_image.at<uchar>(x, y) = 255; }
        }

        return projected_image;
    }
    else
    {
        cv::Mat projected_image = cv::Mat::zeros(10, 10, CV_8UC1);
        return projected_image;

    }
    
}   



void CloudViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2)
{
    // viewer

    pcl::visualization::PCLVisualizer viewer ("Cloud Viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_1_color_handler (cloud_1, 255, 255, 255);
    viewer.addPointCloud (cloud_1, cloud_1_color_handler, "cloud_1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_2_color_handler (cloud_2, 230, 20, 20); // Red
    viewer.addPointCloud (cloud_2, cloud_2_color_handler, "cloud_2");

    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_1");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_2");
    
    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
      viewer.spinOnce ();
    }
    
}

cv::Mat processImage(cv::Mat image, std::string string)
{
    int thresh = 100;
    RNG rng(12345);

    blur( image, image, Size(3,3) );

    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    threshold( image, threshold_output, thresh, 255, THRESH_BINARY );

    findContours( threshold_output, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    int max_num = 0;
    int max_index = 0;

    Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC1);


    if(!contours.empty())
    {   
        for(int i = 0; i < contours.size(); i++)
        {
            if(contours[i].size() > max_num)
               { max_num = contours[i].size(); max_index = i;}
        }


        RotatedRect minRect = minAreaRect( Mat(contours[max_index]) );


        Point2f rect_points[4];
        minRect.points( rect_points );

        for ( int j = 0; j < 4; j++ )
        {   
            rectpoints.at(j) = rect_points[j];
        }
        
        for ( int j = 0; j < 4; j++ )
        {   
            line( drawing, rectpoints.at(j), rectpoints.at((j+1)%4), 255 );
        } 
    }
    
    return drawing;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr back_projection(cv::Mat image, std::string string)
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            if(image.at<uchar>(i, j) == 0)
            {}
            else 
            {   
                
                double i_ = i + min_x - 5;
                double j_ = j + min_y - 5;

                i_ /= 1000;
                j_ /= 1000;
                


                double x = i_*(unit_x.x) + j_*(unit_y.x) + origin.x;
                double y = i_*(unit_x.y) + j_*(unit_y.y) + origin.y;
                double z = i_*(unit_x.z) + j_*(unit_y.z) + origin.z;

                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;

                box_cloud->points.push_back(point);
            }
        }
    }

    for (int i = 0; i < 4; i++)
    {
        double i_ = rectpoints.at(i).y + min_x - 5;
        double j_ = rectpoints.at(i).x + min_y - 5;

        i_ /= 1000;
        j_ /= 1000;
                

        double x = i_*(unit_x.x) + j_*(unit_y.x) + origin.x;
        double y = i_*(unit_x.y) + j_*(unit_y.y) + origin.y;
        double z = i_*(unit_x.z) + j_*(unit_y.z) + origin.z;

        keypoints.at(i).x = x;
        keypoints.at(i).y = y;
        keypoints.at(i).z = z;

        if(string == "red")
        {
            red_keypoints.at(i).x = x;
            red_keypoints.at(i).y = y;
            red_keypoints.at(i).z = z;
    
        }
        else if (string == "orange")
        {
            orange_keypoints.at(i).x = x;
            orange_keypoints.at(i).y = y;
            orange_keypoints.at(i).z = z;        
        }
        else if (string == "yellow")
        {
            yellow_keypoints.at(i).x = x;
            yellow_keypoints.at(i).y = y;
            yellow_keypoints.at(i).z = z;        
        }
        else if (string == "green")
        {
            green_keypoints.at(i).x = x;
            green_keypoints.at(i).y = y;
            green_keypoints.at(i).z = z;        
        }
        else if (string == "blue")
        {
            blue_keypoints.at(i).x = x;
            blue_keypoints.at(i).y = y;
            blue_keypoints.at(i).z = z;        
        }
        else if (string == "purple")
        {
            purple_keypoints.at(i).x = x;
            purple_keypoints.at(i).y = y;
            purple_keypoints.at(i).z = z;        
        }
        else
        {
            brown_keypoints.at(i).x = x;
            brown_keypoints.at(i).y = y;
            brown_keypoints.at(i).z = z;        
        }
    }

    return box_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr makeForm (pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud)
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
    float norm = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));

    unit_vector.at(0) = a / norm;
    unit_vector.at(1) = b / norm;
    unit_vector.at(2) = c / norm;

    for (int i = 0; i < box_cloud->points.size(); i++)
    {
        pcl::PointXYZ point = box_cloud->points[i];
        double x = point.x;
        double y = point.y;
        double z = point.z;

        double scalar = block_height;

        pcl::PointXYZ pt;
        pt.x = x + scalar*unit_vector.at(0);
        pt.y = y + scalar*unit_vector.at(1);
        pt.z = z + scalar*unit_vector.at(2);

        temp->points.push_back(pt);
    }

    int num = 0;

    for(int i = 0; i < box_cloud->points.size(); i++)
    {
        pcl::PointXYZ point = box_cloud->points[i];
        temp->points.push_back(point);
    }

    double val = 1e-8; 

    for (int i = 0; i < 4; i++)
    {   
        pcl::PointXYZ pts;
        pts.x = keypoints.at(i).x;
        pts.y = keypoints.at(i).y;
        pts.z = keypoints.at(i).z;

        double intval = block_height / 20;

        for (int j = 0; j < 20; j++)
        {   
            pcl::PointXYZ pt;
            pt.x = intval*j*unit_vector.at(0) + pts.x;
            pt.y = intval*j*unit_vector.at(1) + pts.y;
            pt.z = intval*j*unit_vector.at(2) + pts.z;

            temp->points.push_back(pt);
        }
    }    

    return temp;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cv2pcl(cv::Mat image)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {   
            pcl::PointXYZ point;
            point.x = image.at<cv::Vec3f>(i, j)[0];
            point.y = image.at<cv::Vec3f>(i, j)[1];
            point.z = image.at<cv::Vec3f>(i, j)[2];

            cloud->points.push_back(point);
        }
    }

    return cloud;
}

pcl::PointXYZ GetUnitY(pcl::PointXYZ origin, pcl::PointXYZ unit_x, Plane::Plane_model plane)
{   
    pcl::PointXYZ unit_y;

    if( !b == 0 )
    {
        float K = - c - origin.x * a - origin.y * b - origin.z * c - d;

        unit_y.x = (- unit_x.z - unit_x.y * K / b) / (unit_x.x + unit_x.y * (- a / b));
        unit_y.y = (K - a * unit_y.x) / b;
        unit_y.z = 1;
   
    }else if ( !a == 0 )
    {
        float K = - c - origin.x * a - origin.z * c - d;
        unit_y.x = K / a;
        unit_y.y = ( - unit_x.z - unit_x.x * unit_y.x ) / unit_x.y;
        unit_y.z = 1;
    }else
    {   
        unit_y.x = 1;
        unit_y.z = (- d / c) - origin.z;
        unit_y.y = (- unit_x.z * unit_y.z - unit_x.x) / unit_x.y;
    }

    
    float norm = sqrt(pow(unit_y.x, 2) + pow(unit_y.y, 2) + pow(unit_y.z, 2));

    unit_y.x /= norm;
    unit_y.y /= norm;
    unit_y.z /= norm;

    if(unit_y.x < 0)
    {
        unit_y.x = - unit_y.x;
        unit_y.y = - unit_y.y;
        unit_y.z = - unit_y.z;
    }

    return unit_y;
}


double GetDist(Plane::Plane_model plane, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier)
{   
    double dist;
    double max_dist = 0;

    double a = plane.a;
    double b = plane.b;
    double c = plane.c;
    double d = plane.d;

    for(int i = 0; i < cloud_outlier->points.size(); i++)
    {
        double x = cloud_outlier->points[i].x;
        double y = cloud_outlier->points[i].y;
        double z = cloud_outlier->points[i].z;

        dist = abs((a * x + b * y + c * z + d) / plane.denominator);

        if(dist > 0.07)
            { }
        else if (dist > max_dist)
            { max_dist = dist; }
    }


    if(max_dist < 0.045 && max_dist > 0.005)
        { return 0.025; }
    else if(max_dist < 0.07 && max_dist > 0.03)
        { return 0.05; }
    else{ return 0; }
}

void Merge(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> color_vector)
{
    for (int i = 0; i < color_vector.at(0)->points.size(); i++)
    {
        pcl::PointXYZ pt = color_vector.at(0)->points[i];
        merged_cloud->points.push_back(pt);
    }

    for (int i = 0; i < color_vector.at(1)->points.size(); i++)
    {
        pcl::PointXYZ pt = color_vector.at(1)->points[i];
        merged_cloud->points.push_back(pt);
    }
} 

void vizPointCloud(cv::Mat image_RGB, pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud)
{   
    float fx = 612.86083984375;
    float fy = 613.0430908203125;
    float cx = 316.27764892578125; 
    float cy = 250.1717071533203;

    cv::Mat cloud_image = cv::Mat::zeros(image_RGB.rows, image_RGB.cols, CV_8UC1);
    for(int i = 0; i < merged_cloud->points.size(); i++)
    {
        pcl::PointXYZ pt = merged_cloud->points[i];
        float X = pt.x;
        float Y = pt.y;
        float Z = pt.z;

        int x = cx + fx * X / Z;
        int y = cy + fy * Y / Z;

        if( x > 0 && x < image_RGB.cols && y > 0 && y < image_RGB.rows)
        {
             cloud_image.at<uchar>(y, x) = 255;        
        }
    }

    cv::imshow("cloud image visualization", cloud_image);
    cv::waitKey(2);
}

pcl::PointCloud<pcl::PointXYZ> makeSyntheticCloud(std::string color_string)
{   

    if(color_string == "red")
    {   
        pcl::PointCloud<pcl::PointXYZ> output_cloud;
        output_cloud = GetRedSynthetic();

        return output_cloud;
    }
    else if(color_string == "orange")
    {
        pcl::PointCloud<pcl::PointXYZ> output_cloud;
        output_cloud = GetOrangeSynthetic();

        return output_cloud;
    }
    else if(color_string == "yellow")
    {
        pcl::PointCloud<pcl::PointXYZ> output_cloud;
        output_cloud = GetYellowSynthetic();

        return output_cloud;        
    }
    else if(color_string == "green")
    {
        pcl::PointCloud<pcl::PointXYZ> output_cloud;
        output_cloud = GetGreenSynthetic();

        return output_cloud;        
    }
    else if(color_string == "blue")
    {   
        pcl::PointCloud<pcl::PointXYZ> output_cloud;
        output_cloud = GetBlueSynthetic();

        return output_cloud; 
    }
    else if(color_string == "purple")
    {   
        pcl::PointCloud<pcl::PointXYZ> output_cloud;
        output_cloud = GetPurpleSynthetic();

        return output_cloud;
    }
    else if(color_string == "brown")
    {
        pcl::PointCloud<pcl::PointXYZ> output_cloud;
        output_cloud = GetBrownSynthetic();

        return output_cloud;
    }

}   

double getDistant(pcl::PointXYZ pt_1, pcl::PointXYZ pt_2)
{   
    double distance = sqrt(pow((pt_1.x - pt_2.x), 2) + pow((pt_1.y - pt_2.y), 2) + pow((pt_1.z - pt_2.z), 2));
    return distance;
}

int getStep(float number)
{
    if(number > 0.02 && number < 0.03)
        { return 1; }
    else if(number > 0.045 && number < 0.055)
        { return 2; }
    else if(number > 0.07 && number < 0.08)
        { return 3; }
    else 
        { return 0; }
}


pcl::PointCloud<pcl::PointXYZ> GetRedSynthetic()
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr red_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    red_cloud = red_vector.at(0);

    pcl::PointCloud<pcl::PointXYZ> output_cloud;

    float block_width = getDistant(red_keypoints.at(0), red_keypoints.at(1));
    float block_depth = getDistant(red_keypoints.at(0), red_keypoints.at(3));

    int width_step = getStep(block_width);
    int depth_step = getStep(block_depth);
    int height_step = getStep(red_block_height);

    if(width_step * depth_step * height_step == 4)
    {   
        std::vector<float> width_vector = std::vector<float> (3);
        width_vector.at(0) = red_keypoints.at(1).x - red_keypoints.at(0).x;
        width_vector.at(1) = red_keypoints.at(1).y - red_keypoints.at(0).y;
        width_vector.at(2) = red_keypoints.at(1).z - red_keypoints.at(0).z;

        float norm = sqrt(pow(width_vector.at(0), 2) + pow(width_vector.at(1),2) + pow(width_vector.at(2), 2));

        width_vector.at(0) = width_vector.at(0) / norm;
        width_vector.at(1) = width_vector.at(1) / norm;
        width_vector.at(2) = width_vector.at(2) / norm;

        std::vector<float> depth_vector = std::vector<float> (3);
        depth_vector.at(0) = red_keypoints.at(3).x - red_keypoints.at(0).x;
        depth_vector.at(1) = red_keypoints.at(3).y - red_keypoints.at(0).y;
        depth_vector.at(2) = red_keypoints.at(3).z - red_keypoints.at(0).z;

        norm = sqrt(pow(depth_vector.at(0), 2) + pow(depth_vector.at(1),2) + pow(depth_vector.at(2), 2));

        depth_vector.at(0) = depth_vector.at(0) / norm;
        depth_vector.at(1) = depth_vector.at(1) / norm;
        depth_vector.at(2) = depth_vector.at(2) / norm;
        
        std::vector<float> height_vector = std::vector<float> (3);
        height_vector.at(0) = unit_vector.at(0);
        height_vector.at(1) = unit_vector.at(1);
        height_vector.at(2) = unit_vector.at(2);

        std::vector<std::vector<float> > vector = std::vector<std::vector<float> > (3);
        vector.at(0) = height_vector;
        vector.at(1) = width_vector;
        vector.at(2) = depth_vector; 

        if(height_step == 2)
        {
            pcl::PointXYZ point_one;
            pcl::PointXYZ point_two;

            if(width_step == 2)
            {
                point_one.x = 2;
                point_one.y = 0.5;
                point_one.z = 0.5;
                
                point_two.x = 2;
                point_two.y = 1.5;
                point_two.z = 0.5;
    
            }
            else if(depth_step == 2)
            {
                point_one.x = 2;
                point_one.y = 0.5;
                point_one.z = 0.5;
                
                point_two.x = 2;
                point_two.y = 0.5;
                point_two.z = 1.5;
    
            }

            pcl::PointXYZ dier;
            pcl::PointXYZ diyi;

            if(Red_CheckDist(vector, red_cloud, point_one, point_two))
                { dier = point_one; diyi = point_two; }
            else
                { dier = point_two; diyi = point_one; }

            output_cloud.points.push_back(GivePoint(vector, red_keypoints.at(0), dier.x - 1/2, dier.y, dier.z));
            output_cloud.points.push_back(GivePoint(vector, red_keypoints.at(0), dier.x - 3/2, dier.y, dier.z));
            output_cloud.points.push_back(GivePoint(vector, red_keypoints.at(0), diyi.x - 3/2, diyi.y, diyi.z));

            if(output_cloud.points.size() == 3)
                { pcl::io::savePLYFileASCII ("output_cloud/red_output_cloud.ply", output_cloud); return output_cloud; }
            
        }
        else if(height_step == 1)
        {   
            int output_array[width_step][depth_step][height_step] = {};

            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                     output_array[j][k][i] = 0;
                    }
                }
            }

            for (int n = 0; n < red_cloud->points.size(); n++)
            {
                std::vector<int> index = std::vector<int> (3);

                pcl::PointXYZ pt;
                pt.x = red_cloud->points[n].x;
                pt.y = red_cloud->points[n].y;
                pt.z = red_cloud->points[n].z;


                float min_dist = 100;
                for(int i = 0; i < height_step; i++)
                {
                    for(int j = 0; j < width_step; j++)
                    {
                        for(int k = 0; k < depth_step; k++)
                        {   
                            if(pt.x == 0 && pt.y == 0 && pt.z == 0)
                            {}
                            else
                            {   pcl::PointXYZ point;
                                point.x = red_keypoints.at(0).x + height_vector.at(0)*(0.025)*i + width_vector.at(0)*(0.025)*j + depth_vector.at(0)*(0.025)*k + (height_vector.at(0)*(0.025) + width_vector.at(0)*(0.025) + depth_vector.at(0)*(0.025)) / 2; 
                                point.y = red_keypoints.at(0).y + height_vector.at(1)*(0.025)*i + width_vector.at(1)*(0.025)*j + depth_vector.at(1)*(0.025)*k + (height_vector.at(1)*(0.025) + width_vector.at(1)*(0.025) + depth_vector.at(1)*(0.025)) / 2; 
                                point.z = red_keypoints.at(0).z + height_vector.at(2)*(0.025)*i + width_vector.at(2)*(0.025)*j + depth_vector.at(2)*(0.025)*k + (height_vector.at(2)*(0.025) + width_vector.at(2)*(0.025) + depth_vector.at(2)*(0.025)) / 2;

                                double dist = getDistant(pt, point);

                                if(dist < min_dist)
                                {
                                    min_dist = dist;
                                    index.at(0) = j;
                                    index.at(1) = k;
                                    index.at(2) = i;
                                }
                            }
                        }
                    }
                }

                output_array[index.at(0)][index.at(1)][index.at(2)] += 1;
            }
            
            int min_value = 10000;
            std::vector<int> min_index = std::vector<int> (3);
            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(output_array[j][k][i] < min_value)
                        {   
                            min_value = output_array[j][k][i];
                            min_index.at(0) = j;
                            min_index.at(1) = k;
                            min_index.at(2) = i;
                        }
                    }
                }
            }
            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(min_index.at(0) == j && min_index.at(1) == k && min_index.at(2) == i)
                        {}
                        else
                        {   

                            pcl::PointXYZ point;
                            point.x = red_keypoints.at(0).x + height_vector.at(0)*(0.025)*i + width_vector.at(0)*(0.025)*j + depth_vector.at(0)*(0.025)*k + (height_vector.at(0)*(0.025) + width_vector.at(0)*(0.025) + depth_vector.at(0)*(0.025)) / 2; 
                            point.y = red_keypoints.at(0).y + height_vector.at(1)*(0.025)*i + width_vector.at(1)*(0.025)*j + depth_vector.at(1)*(0.025)*k + (height_vector.at(1)*(0.025) + width_vector.at(1)*(0.025) + depth_vector.at(1)*(0.025)) / 2; 
                            point.z = red_keypoints.at(0).z + height_vector.at(2)*(0.025)*i + width_vector.at(2)*(0.025)*j + depth_vector.at(2)*(0.025)*k + (height_vector.at(2)*(0.025) + width_vector.at(2)*(0.025) + depth_vector.at(2)*(0.025)) / 2;

                            output_cloud.points.push_back(point); 
                        }
                    }
                }
            }
            if(output_cloud.points.size() == 3)
                { pcl::io::savePLYFileASCII ("output_cloud/red_output_cloud.ply", output_cloud); return output_cloud; }
        }
    }
}

bool Red_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr red_cloud, pcl::PointXYZ point_one, pcl::PointXYZ point_two)
{   
    pcl::PointXYZ box_ones;
    pcl::PointXYZ box_onee;

    pcl::PointXYZ box_twos;
    pcl::PointXYZ box_twoe;

    box_ones = GivePoint(vector, red_keypoints.at(0), point_one.x - 1, point_one.y - 1/2, point_one.z - 1/2);
    box_onee = GivePoint(vector, red_keypoints.at(0), point_one.x, point_one.y + 1/2, point_one.z + 1/2);

    box_twos = GivePoint(vector, red_keypoints.at(0), point_two.x - 1, point_two.y - 1/2, point_two.z - 1/2);
    box_twoe = GivePoint(vector, red_keypoints.at(0), point_two.x, point_two.y + 1/2, point_two.z + 1/2);

    int num_one = 0;
    int num_two = 0;

    for(int i = 0; i < red_cloud->points.size(); i++)
    {   
        pcl::PointXYZ point = red_cloud->points[i];

        if(btwCheck(point, box_ones, box_onee))
            { num_one++; }
        if(btwCheck(point, box_twos, box_twoe))
            { num_two++; }
    }

    if(num_one > num_two)
        { return true; }
    else{ return false; }
}

pcl::PointXYZ GivePoint(std::vector<std::vector<float> > vector, pcl::PointXYZ origin, double x, double y, double z)
{   
    std::vector<float> height_vector, width_vector, depth_vector;
    height_vector = vector.at(0);
    width_vector = vector.at(1);
    depth_vector = vector.at(2);

    pcl::PointXYZ point;
    point.x = origin.x + (0.025)*x*(height_vector.at(0)) + (0.025)*y*(width_vector.at(0)) + (0.025)*z*(depth_vector.at(0)); 
    point.y = origin.y + (0.025)*x*(height_vector.at(1)) + (0.025)*y*(width_vector.at(1)) + (0.025)*z*(depth_vector.at(1));
    point.z = origin.z + (0.025)*x*(height_vector.at(2)) + (0.025)*y*(width_vector.at(2)) + (0.025)*z*(depth_vector.at(2));

    return point;
}

bool btwCheck(pcl::PointXYZ point, pcl::PointXYZ start, pcl::PointXYZ end)
{   
    pcl::PointXYZ center;
    center.x = (start.x + end.x) / 2;
    center.y = (start.y + end.y) / 2;
    center.z = (start.z + end.z) / 2;

    double dist = getDistant(center, point);

    if(dist <= sqrt(pow(0.025, 2) + pow(0.025, 2) + pow(0.025, 2)) / 2)
        { return true; }
    else{ return false; }
}


pcl::PointCloud<pcl::PointXYZ> GetOrangeSynthetic()
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr orange_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    orange_cloud = orange_vector.at(0);

    pcl::PointCloud<pcl::PointXYZ> output_cloud;

    float block_width = getDistant(orange_keypoints.at(0), orange_keypoints.at(1));
    float block_depth = getDistant(orange_keypoints.at(0), orange_keypoints.at(3));

    int width_step = getStep(block_width);
    int depth_step = getStep(block_depth);
    int height_step = getStep(orange_block_height);


    if(width_step * depth_step * height_step == 6)
    {   

        std::vector<float> width_vector = std::vector<float> (3);
        width_vector.at(0) = orange_keypoints.at(1).x - orange_keypoints.at(0).x;
        width_vector.at(1) = orange_keypoints.at(1).y - orange_keypoints.at(0).y;
        width_vector.at(2) = orange_keypoints.at(1).z - orange_keypoints.at(0).z;

        float norm = sqrt(pow(width_vector.at(0), 2) + pow(width_vector.at(1),2) + pow(width_vector.at(2), 2));

        width_vector.at(0) = width_vector.at(0) / norm;
        width_vector.at(1) = width_vector.at(1) / norm;
        width_vector.at(2) = width_vector.at(2) / norm;

        std::vector<float> depth_vector = std::vector<float> (3);
        depth_vector.at(0) = orange_keypoints.at(3).x - orange_keypoints.at(0).x;
        depth_vector.at(1) = orange_keypoints.at(3).y - orange_keypoints.at(0).y;
        depth_vector.at(2) = orange_keypoints.at(3).z - orange_keypoints.at(0).z;

        norm = sqrt(pow(depth_vector.at(0), 2) + pow(depth_vector.at(1),2) + pow(depth_vector.at(2), 2));

        depth_vector.at(0) = depth_vector.at(0) / norm;
        depth_vector.at(1) = depth_vector.at(1) / norm;
        depth_vector.at(2) = depth_vector.at(2) / norm;

        std::vector<float> height_vector = std::vector<float> (3);
        height_vector.at(0) = unit_vector.at(0);
        height_vector.at(1) = unit_vector.at(1);
        height_vector.at(2) = unit_vector.at(2); // It must be same at any color


        std::vector<std::vector<float> > vector = std::vector<std::vector<float> > (3);
        vector.at(0) = height_vector;
        vector.at(1) = width_vector;
        vector.at(2) = depth_vector; 

        if(height_step == 2)
        {
            pcl::PointXYZ point_one;
            pcl::PointXYZ point_two;

            if(width_step == 3)
            {
                point_one.x = 2;
                point_one.y = 0.5;
                point_one.z = 0.5;
                
                point_two.x = 2;
                point_two.y = 2.5;
                point_two.z = 0.5;
    
            }
            else if(depth_step == 3)
            {
                point_one.x = 2;
                point_one.y = 0.5;
                point_one.z = 0.5;
                
                point_two.x = 2;
                point_two.y = 0.5;
                point_two.z = 2.5;
    
            }

            pcl::PointXYZ dier;
            pcl::PointXYZ diyi;

            if(Orange_CheckDist(vector, orange_cloud, point_one, point_two))
                { dier = point_one; diyi = point_two; }
            else
                { dier = point_two; diyi = point_one; }

            output_cloud.points.push_back(GivePoint(vector, orange_keypoints.at(0), dier.x - 1/2, dier.y, dier.z));
            output_cloud.points.push_back(GivePoint(vector, orange_keypoints.at(0), dier.x - 3/2, dier.y, dier.z));
            output_cloud.points.push_back(GivePoint(vector, orange_keypoints.at(0), diyi.x - 3/2, diyi.y, diyi.z));
            output_cloud.points.push_back(GivePoint(vector, orange_keypoints.at(0), diyi.x - 3/2, (diyi.y + dier.y)/2, (diyi.z + dier.z)/2));

            if(output_cloud.points.size() == 4)
                { pcl::io::savePLYFileASCII ("output_cloud/orange_output_cloud.ply", output_cloud); return output_cloud; }
            
        }
        else if(height_step == 1)
        {   
            int output_array[width_step][depth_step][height_step] = {};

            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                     output_array[j][k][i] = 0;
                    }
                }
            }

            for (int n = 0; n < orange_cloud->points.size(); n++)
            {
                std::vector<int> index = std::vector<int> (3);

                pcl::PointXYZ pt;
                pt.x = orange_cloud->points[n].x;
                pt.y = orange_cloud->points[n].y;
                pt.z = orange_cloud->points[n].z;


                float min_dist = 100;
                for(int i = 0; i < height_step; i++)
                {
                    for(int j = 0; j < width_step; j++)
                    {
                        for(int k = 0; k < depth_step; k++)
                        {   
                            if(pt.x == 0 && pt.y == 0 && pt.z == 0)
                            {}
                            else
                            {   pcl::PointXYZ point;
                                point.x = orange_keypoints.at(0).x + height_vector.at(0)*(0.025)*i + width_vector.at(0)*(0.025)*j + depth_vector.at(0)*(0.025)*k + (height_vector.at(0)*(0.025) + width_vector.at(0)*(0.025) + depth_vector.at(0)*(0.025)) / 2; 
                                point.y = orange_keypoints.at(0).y + height_vector.at(1)*(0.025)*i + width_vector.at(1)*(0.025)*j + depth_vector.at(1)*(0.025)*k + (height_vector.at(1)*(0.025) + width_vector.at(1)*(0.025) + depth_vector.at(1)*(0.025)) / 2; 
                                point.z = orange_keypoints.at(0).z + height_vector.at(2)*(0.025)*i + width_vector.at(2)*(0.025)*j + depth_vector.at(2)*(0.025)*k + (height_vector.at(2)*(0.025) + width_vector.at(2)*(0.025) + depth_vector.at(2)*(0.025)) / 2;

                                double dist = getDistant(pt, point);

                                if(dist < min_dist)
                                {
                                    min_dist = dist;
                                    index.at(0) = j;
                                    index.at(1) = k;
                                    index.at(2) = i;
                                }
                            }
                        }
                    }
                }

                output_array[index.at(0)][index.at(1)][index.at(2)] += 1;
            }


            int min_value = 100000;
            std::vector<int> min_index = std::vector<int> (3);
            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(output_array[j][k][i] < min_value)
                        {   
                            min_value = output_array[j][k][i];
                            min_index.at(0) = j;
                            min_index.at(1) = k;
                            min_index.at(2) = i;
                        }
                    }
                }
            }

            int min_value_2 = 100000;
            std::vector<int> min_index_2 = std::vector<int> (3);
           for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(j == min_index.at(0) && k == min_index.at(1) && i == min_index.at(2))
                        {}
                        else
                        {
                            if(output_array[j][k][i] < min_value_2)
                            {   
                                min_value_2 = output_array[j][k][i];
                                min_index_2.at(0) = j;
                                min_index_2.at(1) = k;
                                min_index_2.at(2) = i;
                            }
                        }
                    }
                }
            }

            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(min_index.at(0) == j && min_index.at(1) == k && min_index.at(2) == i)
                        {}
                        else if(min_index_2.at(0) == j && min_index_2.at(1) == k && min_index_2.at(2) == i)
                        {}
                        else
                        { 
                            pcl::PointXYZ point;
                            point.x = orange_keypoints.at(0).x + height_vector.at(0)*(0.025)*i + width_vector.at(0)*(0.025)*j + depth_vector.at(0)*(0.025)*k + (height_vector.at(0)*(0.025) + width_vector.at(0)*(0.025) + depth_vector.at(0)*(0.025)) / 2; 
                            point.y = orange_keypoints.at(0).y + height_vector.at(1)*(0.025)*i + width_vector.at(1)*(0.025)*j + depth_vector.at(1)*(0.025)*k + (height_vector.at(1)*(0.025) + width_vector.at(1)*(0.025) + depth_vector.at(1)*(0.025)) / 2; 
                            point.z = orange_keypoints.at(0).z + height_vector.at(2)*(0.025)*i + width_vector.at(2)*(0.025)*j + depth_vector.at(2)*(0.025)*k + (height_vector.at(2)*(0.025) + width_vector.at(2)*(0.025) + depth_vector.at(2)*(0.025)) / 2; 
                            
                            output_cloud.points.push_back(point); 
                        }
                    }
                }
            }
            if(output_cloud.points.size() == 4)
                { pcl::io::savePLYFileASCII ("output_cloud/orange_output_cloud.ply", output_cloud); return output_cloud; }            
        }
    }
}

bool Orange_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr orange_cloud, pcl::PointXYZ point_one, pcl::PointXYZ point_two)
{   
    //box_number_one
    pcl::PointXYZ box_ones;
    pcl::PointXYZ box_onee;

    pcl::PointXYZ box_twos;
    pcl::PointXYZ box_twoe;

    box_ones = GivePoint(vector, orange_keypoints.at(0), point_one.x - 1, point_one.y - 1/2, point_one.z - 1/2);
    box_onee = GivePoint(vector, orange_keypoints.at(0), point_one.x, point_one.y + 1/2, point_one.z + 1/2);

    box_twos = GivePoint(vector, orange_keypoints.at(0), point_two.x - 1, point_two.y - 1/2, point_two.z - 1/2);
    box_twoe = GivePoint(vector, orange_keypoints.at(0), point_two.x, point_two.y + 1/2, point_two.z + 1/2);

    int num_one = 0;
    int num_two = 0;

    for(int i = 0; i < orange_cloud->points.size(); i++)
    {   
        pcl::PointXYZ point = orange_cloud->points[i];

        if(btwCheck(point, box_ones, box_onee))
            { num_one++; }
        if(btwCheck(point, box_twos, box_twoe))
            { num_two++; }
    }

    if(num_one > num_two)
        { return true; }
    else{ return false; }
}


pcl::PointCloud<pcl::PointXYZ> GetYellowSynthetic()
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr yellow_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    yellow_cloud = yellow_vector.at(0);

    pcl::PointCloud<pcl::PointXYZ> output_cloud;

    float block_width = getDistant(yellow_keypoints.at(0), yellow_keypoints.at(1));
    float block_depth = getDistant(yellow_keypoints.at(0), yellow_keypoints.at(3));

    int width_step = getStep(block_width);
    int depth_step = getStep(block_depth);
    int height_step = getStep(yellow_block_height);

    if(width_step * depth_step * height_step == 6)
    {   
        std::vector<float> width_vector = std::vector<float> (3);
        width_vector.at(0) = yellow_keypoints.at(1).x - yellow_keypoints.at(0).x;
        width_vector.at(1) = yellow_keypoints.at(1).y - yellow_keypoints.at(0).y;
        width_vector.at(2) = yellow_keypoints.at(1).z - yellow_keypoints.at(0).z;

        float norm = sqrt(pow(width_vector.at(0), 2) + pow(width_vector.at(1),2) + pow(width_vector.at(2), 2));

        width_vector.at(0) = width_vector.at(0) / norm;
        width_vector.at(1) = width_vector.at(1) / norm;
        width_vector.at(2) = width_vector.at(2) / norm;

        std::vector<float> depth_vector = std::vector<float> (3);
        depth_vector.at(0) = yellow_keypoints.at(3).x - yellow_keypoints.at(0).x;
        depth_vector.at(1) = yellow_keypoints.at(3).y - yellow_keypoints.at(0).y;
        depth_vector.at(2) = yellow_keypoints.at(3).z - yellow_keypoints.at(0).z;

        norm = sqrt(pow(depth_vector.at(0), 2) + pow(depth_vector.at(1),2) + pow(depth_vector.at(2), 2));

        depth_vector.at(0) = depth_vector.at(0) / norm;
        depth_vector.at(1) = depth_vector.at(1) / norm;
        depth_vector.at(2) = depth_vector.at(2) / norm;

        std::vector<float> height_vector = std::vector<float> (3);
        height_vector.at(0) = unit_vector.at(0);
        height_vector.at(1) = unit_vector.at(1);
        height_vector.at(2) = unit_vector.at(2); // It must be same at any color

        std::vector<std::vector<float> > vector = std::vector<std::vector<float> > (3);
        vector.at(0) = height_vector;
        vector.at(1) = width_vector;
        vector.at(2) = depth_vector; 

        if(height_step == 2)
        {
            pcl::PointXYZ point;

            if(width_step == 3)
            {
                point.x = 1.5;
                point.y = 1.5;
                point.z = 0.5;

                output_cloud.points.push_back(GivePoint(vector, yellow_keypoints.at(0), point.x, point.y, point.z));
                output_cloud.points.push_back(GivePoint(vector, yellow_keypoints.at(0), point.x - 1, point.y, point.z));
                output_cloud.points.push_back(GivePoint(vector, yellow_keypoints.at(0), point.x - 1, point.y - 1, point.z));
                output_cloud.points.push_back(GivePoint(vector, yellow_keypoints.at(0), point.x - 1, point.y + 1, point.z));
            }
            else if(depth_step == 3)
            {
                point.x = 1.5;
                point.y = 0.5;
                point.z = 1.5;

                output_cloud.points.push_back(GivePoint(vector, yellow_keypoints.at(0), point.x, point.y, point.z));
                output_cloud.points.push_back(GivePoint(vector, yellow_keypoints.at(0), point.x - 1, point.y, point.z));
                output_cloud.points.push_back(GivePoint(vector, yellow_keypoints.at(0), point.x - 1, point.y, point.z - 1));
                output_cloud.points.push_back(GivePoint(vector, yellow_keypoints.at(0), point.x - 1, point.y, point.z + 1));   
            }

            if(output_cloud.points.size() == 4)
                { pcl::io::savePLYFileASCII ("output_cloud/yellow_output_cloud.ply", output_cloud); return output_cloud; }
            
        }
        else if(height_step == 1)
        {   
            int output_array[width_step][depth_step][height_step] = {};

            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                     output_array[j][k][i] = 0;
                    }
                }
            }

            for (int n = 0; n < yellow_cloud->points.size(); n++)
            {
                std::vector<int> index = std::vector<int> (3);

                pcl::PointXYZ pt;
                pt.x = yellow_cloud->points[n].x;
                pt.y = yellow_cloud->points[n].y;
                pt.z = yellow_cloud->points[n].z;


                float min_dist = 100;
                for(int i = 0; i < height_step; i++)
                {
                    for(int j = 0; j < width_step; j++)
                    {
                        for(int k = 0; k < depth_step; k++)
                        {   
                            if(pt.x == 0 && pt.y == 0 && pt.z == 0)
                            {}
                            else
                            {   pcl::PointXYZ point;
                                point.x = yellow_keypoints.at(0).x + height_vector.at(0)*(0.025)*i + width_vector.at(0)*(0.025)*j + depth_vector.at(0)*(0.025)*k + (height_vector.at(0)*(0.025) + width_vector.at(0)*(0.025) + depth_vector.at(0)*(0.025)) / 2; 
                                point.y = yellow_keypoints.at(0).y + height_vector.at(1)*(0.025)*i + width_vector.at(1)*(0.025)*j + depth_vector.at(1)*(0.025)*k + (height_vector.at(1)*(0.025) + width_vector.at(1)*(0.025) + depth_vector.at(1)*(0.025)) / 2; 
                                point.z = yellow_keypoints.at(0).z + height_vector.at(2)*(0.025)*i + width_vector.at(2)*(0.025)*j + depth_vector.at(2)*(0.025)*k + (height_vector.at(2)*(0.025) + width_vector.at(2)*(0.025) + depth_vector.at(2)*(0.025)) / 2;

                                double dist = getDistant(pt, point);

                                if(dist < min_dist)
                                {
                                    min_dist = dist;
                                    index.at(0) = j;
                                    index.at(1) = k;
                                    index.at(2) = i;
                                }
                            }
                        }
                    }
                }

                output_array[index.at(0)][index.at(1)][index.at(2)] += 1;
            }


            int min_value = 100000;
            std::vector<int> min_index = std::vector<int> (3);
            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(output_array[j][k][i] < min_value)
                        {   
                            min_value = output_array[j][k][i];
                            min_index.at(0) = j;
                            min_index.at(1) = k;
                            min_index.at(2) = i;
                        }
                    }
                }
            }

            int min_value_2 = 100000;
            std::vector<int> min_index_2 = std::vector<int> (3);
           for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(j == min_index.at(0) && k == min_index.at(1) && i == min_index.at(2))
                        {}
                        else
                        {
                            if(output_array[j][k][i] < min_value_2)
                            {   
                                min_value_2 = output_array[j][k][i];
                                min_index_2.at(0) = j;
                                min_index_2.at(1) = k;
                                min_index_2.at(2) = i;
                            }
                        }
                    }
                }
            }

            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(min_index.at(0) == j && min_index.at(1) == k && min_index.at(2) == i)
                        {}
                        else if(min_index_2.at(0) == j && min_index_2.at(1) == k && min_index_2.at(2) == i)
                        {}
                        else
                        { 
                            pcl::PointXYZ point;
                            point.x = yellow_keypoints.at(0).x + height_vector.at(0)*(0.025)*i + width_vector.at(0)*(0.025)*j + depth_vector.at(0)*(0.025)*k + (height_vector.at(0)*(0.025) + width_vector.at(0)*(0.025) + depth_vector.at(0)*(0.025)) / 2; 
                            point.y = yellow_keypoints.at(0).y + height_vector.at(1)*(0.025)*i + width_vector.at(1)*(0.025)*j + depth_vector.at(1)*(0.025)*k + (height_vector.at(1)*(0.025) + width_vector.at(1)*(0.025) + depth_vector.at(1)*(0.025)) / 2; 
                            point.z = yellow_keypoints.at(0).z + height_vector.at(2)*(0.025)*i + width_vector.at(2)*(0.025)*j + depth_vector.at(2)*(0.025)*k + (height_vector.at(2)*(0.025) + width_vector.at(2)*(0.025) + depth_vector.at(2)*(0.025)) / 2; 
                            
                            output_cloud.points.push_back(point); 
                        }
                    }
                }
            }
            if(output_cloud.points.size() == 4)
                { pcl::io::savePLYFileASCII ("output_cloud/yellow_output_cloud.ply", output_cloud); return output_cloud; }            
        }
    }
}

pcl::PointCloud<pcl::PointXYZ> GetGreenSynthetic()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr green_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    green_cloud = green_vector.at(0);

    pcl::PointCloud<pcl::PointXYZ> output_cloud;

    float block_width = getDistant(green_keypoints.at(0), green_keypoints.at(1));
    float block_depth = getDistant(green_keypoints.at(0), green_keypoints.at(3));

    int width_step = getStep(block_width);
    int depth_step = getStep(block_depth);
    int height_step = getStep(green_block_height);

    if(width_step * depth_step * height_step == 6)
    {   
        std::vector<float> width_vector = std::vector<float> (3);
        width_vector.at(0) = green_keypoints.at(1).x - green_keypoints.at(0).x;
        width_vector.at(1) = green_keypoints.at(1).y - green_keypoints.at(0).y;
        width_vector.at(2) = green_keypoints.at(1).z - green_keypoints.at(0).z;

        float norm = sqrt(pow(width_vector.at(0), 2) + pow(width_vector.at(1),2) + pow(width_vector.at(2), 2));

        width_vector.at(0) = width_vector.at(0) / norm;
        width_vector.at(1) = width_vector.at(1) / norm;
        width_vector.at(2) = width_vector.at(2) / norm;

        std::vector<float> depth_vector = std::vector<float> (3);
        depth_vector.at(0) = green_keypoints.at(3).x - green_keypoints.at(0).x;
        depth_vector.at(1) = green_keypoints.at(3).y - green_keypoints.at(0).y;
        depth_vector.at(2) = green_keypoints.at(3).z - green_keypoints.at(0).z;

        norm = sqrt(pow(depth_vector.at(0), 2) + pow(depth_vector.at(1),2) + pow(depth_vector.at(2), 2));

        depth_vector.at(0) = depth_vector.at(0) / norm;
        depth_vector.at(1) = depth_vector.at(1) / norm;
        depth_vector.at(2) = depth_vector.at(2) / norm;

        std::vector<float> height_vector = std::vector<float> (3);
        height_vector.at(0) = unit_vector.at(0);
        height_vector.at(1) = unit_vector.at(1);
        height_vector.at(2) = unit_vector.at(2); // It must be same at any color

        std::vector<std::vector<float> > vector = std::vector<std::vector<float> > (3);
        vector.at(0) = height_vector;
        vector.at(1) = width_vector;
        vector.at(2) = depth_vector; 

        if(height_step == 2)
        {
            pcl::PointXYZ point_one;
            pcl::PointXYZ point_two;

            if(width_step == 3)
            {
                point_one.x = 2;
                point_one.y = 0.5;
                point_one.z = 0.5;
                
                point_two.x = 2;
                point_two.y = 2.5;
                point_two.z = 0.5;
    
            }
            else if(depth_step == 3)
            {
                point_one.x = 2;
                point_one.y = 0.5;
                point_one.z = 0.5;
                
                point_two.x = 2;
                point_two.y = 0.5;
                point_two.z = 2.5;
    
            }

            pcl::PointXYZ dier;
            pcl::PointXYZ diyi;

            if(Green_CheckDist(vector, green_cloud, point_one, point_two))
                { dier = point_one; diyi = point_two; }
            else
                { dier = point_two; diyi = point_one; }

            output_cloud.points.push_back(GivePoint(vector, green_keypoints.at(0), dier.x - 0.5, dier.y, dier.z));
            output_cloud.points.push_back(GivePoint(vector, green_keypoints.at(0), diyi.x - 1.5, diyi.y, diyi.z));
            output_cloud.points.push_back(GivePoint(vector, green_keypoints.at(0), diyi.x - 1.5, (diyi.y + dier.y)/2, (diyi.z + dier.z)/2));
            output_cloud.points.push_back(GivePoint(vector, green_keypoints.at(0), diyi.x - 0.5, (diyi.y + dier.y)/2, (diyi.z + dier.z)/2));


            if(output_cloud.points.size() == 4)
                { pcl::io::savePLYFileASCII ("output_cloud/green_output_cloud.ply", output_cloud); return output_cloud; }
            
        }
        else if(height_step == 1)
        {   
            int output_array[width_step][depth_step][height_step] = {};

            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                     output_array[j][k][i] = 0;
                    }
                }
            }

            for (int n = 0; n < green_cloud->points.size(); n++)
            {
                std::vector<int> index = std::vector<int> (3);

                pcl::PointXYZ pt;
                pt.x = green_cloud->points[n].x;
                pt.y = green_cloud->points[n].y;
                pt.z = green_cloud->points[n].z;


                float min_dist = 100;
                for(int i = 0; i < height_step; i++)
                {
                    for(int j = 0; j < width_step; j++)
                    {
                        for(int k = 0; k < depth_step; k++)
                        {   
                            if(pt.x == 0 && pt.y == 0 && pt.z == 0)
                            {}
                            else
                            {   pcl::PointXYZ point;
                                point.x = green_keypoints.at(0).x + height_vector.at(0)*(0.025)*i + width_vector.at(0)*(0.025)*j + depth_vector.at(0)*(0.025)*k + (height_vector.at(0)*(0.025) + width_vector.at(0)*(0.025) + depth_vector.at(0)*(0.025)) / 2; 
                                point.y = green_keypoints.at(0).y + height_vector.at(1)*(0.025)*i + width_vector.at(1)*(0.025)*j + depth_vector.at(1)*(0.025)*k + (height_vector.at(1)*(0.025) + width_vector.at(1)*(0.025) + depth_vector.at(1)*(0.025)) / 2; 
                                point.z = green_keypoints.at(0).z + height_vector.at(2)*(0.025)*i + width_vector.at(2)*(0.025)*j + depth_vector.at(2)*(0.025)*k + (height_vector.at(2)*(0.025) + width_vector.at(2)*(0.025) + depth_vector.at(2)*(0.025)) / 2;

                                double dist = getDistant(pt, point);

                                if(dist < min_dist)
                                {
                                    min_dist = dist;
                                    index.at(0) = j;
                                    index.at(1) = k;
                                    index.at(2) = i;
                                }
                            }
                        }
                    }
                }

                output_array[index.at(0)][index.at(1)][index.at(2)] += 1;
            }


            int min_value = 100000;
            std::vector<int> min_index = std::vector<int> (3);
            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(output_array[j][k][i] < min_value)
                        {   
                            min_value = output_array[j][k][i];
                            min_index.at(0) = j;
                            min_index.at(1) = k;
                            min_index.at(2) = i;
                        }
                    }
                }
            }

            int min_value_2 = 100000;
            std::vector<int> min_index_2 = std::vector<int> (3);
           for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(j == min_index.at(0) && k == min_index.at(1) && i == min_index.at(2))
                        {}
                        else
                        {
                            if(output_array[j][k][i] < min_value_2)
                            {   
                                min_value_2 = output_array[j][k][i];
                                min_index_2.at(0) = j;
                                min_index_2.at(1) = k;
                                min_index_2.at(2) = i;
                            }
                        }
                    }
                }
            }

            for(int i = 0; i < height_step; i++)
            {
                for(int j = 0; j < width_step; j++)
                {
                    for(int k = 0; k < depth_step; k++)
                    {   
                        if(min_index.at(0) == j && min_index.at(1) == k && min_index.at(2) == i)
                        {}
                        else if(min_index_2.at(0) == j && min_index_2.at(1) == k && min_index_2.at(2) == i)
                        {}
                        else
                        { 
                            pcl::PointXYZ point;
                            point.x = green_keypoints.at(0).x + height_vector.at(0)*(0.025)*i + width_vector.at(0)*(0.025)*j + depth_vector.at(0)*(0.025)*k + (height_vector.at(0)*(0.025) + width_vector.at(0)*(0.025) + depth_vector.at(0)*(0.025)) / 2; 
                            point.y = green_keypoints.at(0).y + height_vector.at(1)*(0.025)*i + width_vector.at(1)*(0.025)*j + depth_vector.at(1)*(0.025)*k + (height_vector.at(1)*(0.025) + width_vector.at(1)*(0.025) + depth_vector.at(1)*(0.025)) / 2; 
                            point.z = green_keypoints.at(0).z + height_vector.at(2)*(0.025)*i + width_vector.at(2)*(0.025)*j + depth_vector.at(2)*(0.025)*k + (height_vector.at(2)*(0.025) + width_vector.at(2)*(0.025) + depth_vector.at(2)*(0.025)) / 2; 
                            
                            output_cloud.points.push_back(point); 
                        }
                    }
                }
            }
            if(output_cloud.points.size() == 4)
                { pcl::io::savePLYFileASCII ("output_cloud/green_output_cloud.ply", output_cloud); return output_cloud; }            
        }
    }
}

bool Green_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr green_cloud, pcl::PointXYZ point_one, pcl::PointXYZ point_two)
{   
    //box_number_one
    pcl::PointXYZ box_ones;
    pcl::PointXYZ box_onee;

    pcl::PointXYZ box_twos;
    pcl::PointXYZ box_twoe;

    box_ones = GivePoint(vector, green_keypoints.at(0), point_one.x - 1, point_one.y - 1/2, point_one.z - 1/2);
    box_onee = GivePoint(vector, green_keypoints.at(0), point_one.x, point_one.y + 1/2, point_one.z + 1/2);

    box_twos = GivePoint(vector, green_keypoints.at(0), point_two.x - 1, point_two.y - 1/2, point_two.z - 1/2);
    box_twoe = GivePoint(vector, green_keypoints.at(0), point_two.x, point_two.y + 1/2, point_two.z + 1/2);

    int num_one = 0;
    int num_two = 0;

    for(int i = 0; i < green_cloud->points.size(); i++)
    {   
        pcl::PointXYZ point = green_cloud->points[i];

        if(btwCheck(point, box_ones, box_onee))
            { num_one++; }
        if(btwCheck(point, box_twos, box_twoe))
            { num_two++; }
    }

    if(num_one > num_two)
        { return true; }
    else{ return false; }
}
pcl::PointCloud<pcl::PointXYZ> GetBlueSynthetic()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr blue_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    blue_cloud = blue_vector.at(0);

    pcl::PointCloud<pcl::PointXYZ> output_cloud;

    float block_width = getDistant(blue_keypoints.at(0), blue_keypoints.at(1));
    float block_depth = getDistant(blue_keypoints.at(0), blue_keypoints.at(3));

    int width_step = getStep(block_width);
    int depth_step = getStep(block_depth);
    int height_step = getStep(blue_block_height);

    if(width_step * depth_step * height_step == 8)
    {   

        std::vector<float> width_vector = std::vector<float> (3);
        width_vector.at(0) = blue_keypoints.at(1).x - blue_keypoints.at(0).x;
        width_vector.at(1) = blue_keypoints.at(1).y - blue_keypoints.at(0).y;
        width_vector.at(2) = blue_keypoints.at(1).z - blue_keypoints.at(0).z;

        float norm = sqrt(pow(width_vector.at(0), 2) + pow(width_vector.at(1),2) + pow(width_vector.at(2), 2));

        width_vector.at(0) = width_vector.at(0) / norm;
        width_vector.at(1) = width_vector.at(1) / norm;
        width_vector.at(2) = width_vector.at(2) / norm;

        std::vector<float> depth_vector = std::vector<float> (3);
        depth_vector.at(0) = blue_keypoints.at(3).x - blue_keypoints.at(0).x;
        depth_vector.at(1) = blue_keypoints.at(3).y - blue_keypoints.at(0).y;
        depth_vector.at(2) = blue_keypoints.at(3).z - blue_keypoints.at(0).z;

        norm = sqrt(pow(depth_vector.at(0), 2) + pow(depth_vector.at(1),2) + pow(depth_vector.at(2), 2));

        depth_vector.at(0) = depth_vector.at(0) / norm;
        depth_vector.at(1) = depth_vector.at(1) / norm;
        depth_vector.at(2) = depth_vector.at(2) / norm;

        std::vector<float> height_vector = std::vector<float> (3);
        height_vector.at(0) = unit_vector.at(0);
        height_vector.at(1) = unit_vector.at(1);
        height_vector.at(2) = unit_vector.at(2); // It must be same at any color


        std::vector<std::vector<float> > vector = std::vector<std::vector<float> > (3);
        vector.at(0) = height_vector;
        vector.at(1) = width_vector;
        vector.at(2) = depth_vector; 


        pcl::PointXYZ dier;
        pcl::PointXYZ diyi_one;
        pcl::PointXYZ diyi_two;

        Blue_CheckDist(vector, blue_cloud, dier, diyi_one, diyi_two);

        output_cloud.points.push_back(GivePoint(vector, blue_keypoints.at(0), dier.x, dier.y, dier.z));
        output_cloud.points.push_back(GivePoint(vector, blue_keypoints.at(0), dier.x - 1, dier.y, dier.z));
        output_cloud.points.push_back(GivePoint(vector, blue_keypoints.at(0), diyi_one.x, diyi_one.y, diyi_one.z));
        output_cloud.points.push_back(GivePoint(vector, blue_keypoints.at(0), diyi_two.x, diyi_two.y, diyi_two.z));

        if(output_cloud.points.size() == 4)
            { pcl::io::savePLYFileASCII ("output_cloud/blue_output_cloud.ply", output_cloud); return output_cloud; }
    }      
}

void Blue_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr blue_cloud, pcl::PointXYZ& dier, pcl::PointXYZ& diyi_one, pcl::PointXYZ& diyi_two)
{
    pcl::PointXYZ point_array[4] = {};

    point_array[0].x = 1.5;
    point_array[0].y = 0.5;
    point_array[0].z = 0.5;

    point_array[1].x = 1.5;
    point_array[1].y = 1.5;
    point_array[1].z = 0.5;

    point_array[2].x = 1.5;
    point_array[2].y = 0.5;
    point_array[2].z = 1.5;

    point_array[3].x = 1.5;
    point_array[3].y = 1.5;
    point_array[3].z = 1.5;          

    int num_array[4] = {};

    ArrayInit(num_array, 4);
    for (int i = 0; i < blue_cloud->points.size(); i++)
    {
        pcl::PointXYZ point = blue_cloud->points[i];

        pcl::PointXYZ point_one = GivePoint(vector, blue_keypoints.at(0), point_array[0].x, point_array[0].y, point_array[0].z);
        pcl::PointXYZ point_two = GivePoint(vector, blue_keypoints.at(0), point_array[1].x, point_array[1].y, point_array[1].z);
        pcl::PointXYZ point_three = GivePoint(vector, blue_keypoints.at(0), point_array[2].x, point_array[2].y, point_array[2].z);
        pcl::PointXYZ point_four = GivePoint(vector, blue_keypoints.at(0), point_array[3].x, point_array[3].y, point_array[3].z);

        if(Blue_btwCheck(point_one, blue_cloud->points[i]))
            { num_array[0]++; }
        if(Blue_btwCheck(point_two, blue_cloud->points[i]))
            { num_array[1]++; }
        if(Blue_btwCheck(point_three, blue_cloud->points[i]))
            { num_array[2]++; }
        if(Blue_btwCheck(point_four, blue_cloud->points[i]))
            { num_array[3]++; }
    }

    int max_index = 0;
    int max_num = 0; 
    for(int i = 0; i < 4; i++)
    {
        if(max_num < num_array[i])
            { max_num = num_array[i]; max_index = i; }
    }

    dier = point_array[max_index];

    // Now Get diling

    point_array[0].x = 0.5;
    point_array[0].y = 0.5;
    point_array[0].z = 0.5;

    point_array[1].x = 0.5;
    point_array[1].y = 1.5;
    point_array[1].z = 0.5;

    point_array[2].x = 0.5;
    point_array[2].y = 0.5;
    point_array[2].z = 1.5;

    point_array[3].x = 0.5;
    point_array[3].y = 1.5;
    point_array[3].z = 1.5;           

    ArrayInit(num_array, 4);
    for (int i = 0; i < blue_cloud->points.size(); i++)
    {
        pcl::PointXYZ point = blue_cloud->points[i];

        pcl::PointXYZ point_one = GivePoint(vector, blue_keypoints.at(0), point_array[0].x, point_array[0].y, point_array[0].z);
        pcl::PointXYZ point_two = GivePoint(vector, blue_keypoints.at(0), point_array[1].x, point_array[1].y, point_array[1].z);
        pcl::PointXYZ point_three = GivePoint(vector, blue_keypoints.at(0), point_array[2].x, point_array[2].y, point_array[2].z);
        pcl::PointXYZ point_four = GivePoint(vector, blue_keypoints.at(0), point_array[3].x, point_array[3].y, point_array[3].z);

        if(Blue_btwCheck(point_one, blue_cloud->points[i]))
            { num_array[0]++; }
        if(Blue_btwCheck(point_two, blue_cloud->points[i]))
            { num_array[1]++; }
        if(Blue_btwCheck(point_three, blue_cloud->points[i]))
            { num_array[2]++; }
        if(Blue_btwCheck(point_four, blue_cloud->points[i]))
            { num_array[3]++; }
    }   

    int min_index = 0;
    int min_num = 1000;
    for(int i = 0; i < 4; i ++)
    {
        if(i != max_index && min_num > num_array[i])
            { min_num = num_array[i]; min_index = i; }
    }

    int array[2] = {};
    int num = 0;
    for(int i = 0; i < 4; i++)
    {
        if(i != max_index && i != min_index)
        { 
            if(num == 0)
            {
                diyi_one = point_array[i];
                num++;
            } 
            else
            {
                diyi_two = point_array[i];
            }
        }
    }
}

bool Blue_btwCheck(pcl::PointXYZ point, pcl::PointXYZ cloud_point)
{
    double dist = getDistant(point, cloud_point);

    if(dist <= sqrt(pow(0.025, 2) + pow(0.025, 2) + pow(0.025, 2)) / 2)
        { return true; }
    else{ return false; }
}

void ArrayInit(int *array, int num)
{
    for(int i = 0; i < num; i++)
    {
        array[i] = 0;
    }
}


pcl::PointCloud<pcl::PointXYZ> GetPurpleSynthetic()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr purple_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    purple_cloud = purple_vector.at(0);

    pcl::PointCloud<pcl::PointXYZ> output_cloud;

    float block_width = getDistant(purple_keypoints.at(0), purple_keypoints.at(1));
    float block_depth = getDistant(purple_keypoints.at(0), purple_keypoints.at(3));

    int width_step = getStep(block_width);
    int depth_step = getStep(block_depth);
    int height_step = getStep(purple_block_height);

    if(width_step * depth_step * height_step == 8)
    {   
        std::vector<float> width_vector = std::vector<float> (3);
        width_vector.at(0) = purple_keypoints.at(1).x - purple_keypoints.at(0).x;
        width_vector.at(1) = purple_keypoints.at(1).y - purple_keypoints.at(0).y;
        width_vector.at(2) = purple_keypoints.at(1).z - purple_keypoints.at(0).z;

        float norm = sqrt(pow(width_vector.at(0), 2) + pow(width_vector.at(1),2) + pow(width_vector.at(2), 2));

        width_vector.at(0) = width_vector.at(0) / norm;
        width_vector.at(1) = width_vector.at(1) / norm;
        width_vector.at(2) = width_vector.at(2) / norm;

        std::vector<float> depth_vector = std::vector<float> (3);
        depth_vector.at(0) = purple_keypoints.at(3).x - purple_keypoints.at(0).x;
        depth_vector.at(1) = purple_keypoints.at(3).y - purple_keypoints.at(0).y;
        depth_vector.at(2) = purple_keypoints.at(3).z - purple_keypoints.at(0).z;

        norm = sqrt(pow(depth_vector.at(0), 2) + pow(depth_vector.at(1),2) + pow(depth_vector.at(2), 2));

        depth_vector.at(0) = depth_vector.at(0) / norm;
        depth_vector.at(1) = depth_vector.at(1) / norm;
        depth_vector.at(2) = depth_vector.at(2) / norm;

        std::vector<float> height_vector = std::vector<float> (3);
        height_vector.at(0) = unit_vector.at(0);
        height_vector.at(1) = unit_vector.at(1);
        height_vector.at(2) = unit_vector.at(2);

        std::vector<std::vector<float> > vector = std::vector<std::vector<float> > (3);
        vector.at(0) = height_vector;
        vector.at(1) = width_vector;
        vector.at(2) = depth_vector; 


        pcl::PointXYZ dier;
        pcl::PointXYZ diyi_one;
        pcl::PointXYZ diyi_two;

        Purple_CheckDist(vector, purple_cloud, dier, diyi_one, diyi_two);

        output_cloud.points.push_back(GivePoint(vector, purple_keypoints.at(0), dier.x, dier.y, dier.z));
        output_cloud.points.push_back(GivePoint(vector, purple_keypoints.at(0), dier.x - 1, dier.y, dier.z));
        output_cloud.points.push_back(GivePoint(vector, purple_keypoints.at(0), diyi_one.x, diyi_one.y, diyi_one.z));
        output_cloud.points.push_back(GivePoint(vector, purple_keypoints.at(0), diyi_two.x, diyi_two.y, diyi_two.z));
        
        if(output_cloud.points.size() == 4)
            { pcl::io::savePLYFileASCII ("output_cloud/purple_output_cloud.ply", output_cloud); return output_cloud; }
    }      
}

void Purple_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr purple_cloud, pcl::PointXYZ& dier, pcl::PointXYZ& diyi_one, pcl::PointXYZ& diyi_two)
{
    pcl::PointXYZ point_array[4] = {};

    point_array[0].x = 1.5;
    point_array[0].y = 0.5;
    point_array[0].z = 0.5;

    point_array[1].x = 1.5;
    point_array[1].y = 1.5;
    point_array[1].z = 0.5;

    point_array[2].x = 1.5;
    point_array[2].y = 0.5;
    point_array[2].z = 1.5;

    point_array[3].x = 1.5;
    point_array[3].y = 1.5;
    point_array[3].z = 1.5;           

    int num_array[4] = {};

    ArrayInit(num_array, 4);
    for (int i = 0; i < purple_cloud->points.size(); i++)
    {
        pcl::PointXYZ point = purple_cloud->points[i];

        pcl::PointXYZ point_one = GivePoint(vector, purple_keypoints.at(0), point_array[0].x, point_array[0].y, point_array[0].z);
        pcl::PointXYZ point_two = GivePoint(vector, purple_keypoints.at(0), point_array[1].x, point_array[1].y, point_array[1].z);
        pcl::PointXYZ point_three = GivePoint(vector, purple_keypoints.at(0), point_array[2].x, point_array[2].y, point_array[2].z);
        pcl::PointXYZ point_four = GivePoint(vector, purple_keypoints.at(0), point_array[3].x, point_array[3].y, point_array[3].z);

        if(Blue_btwCheck(point_one, purple_cloud->points[i]))
            { num_array[0]++; }
        if(Blue_btwCheck(point_two, purple_cloud->points[i]))
            { num_array[1]++; }
        if(Blue_btwCheck(point_three, purple_cloud->points[i]))
            { num_array[2]++; }
        if(Blue_btwCheck(point_four, purple_cloud->points[i]))
            { num_array[3]++; }
    }

    int max_index = 0;
    int max_num = 0; 
    for(int i = 0; i < 4; i++)
    {
        if(max_num < num_array[i])
            { max_num = num_array[i]; max_index = i; }
    }

    dier = point_array[max_index];

    // Now Get diling

    point_array[0].x = 0.5;
    point_array[0].y = 0.5;
    point_array[0].z = 0.5;

    point_array[1].x = 0.5;
    point_array[1].y = 1.5;
    point_array[1].z = 0.5;

    point_array[2].x = 0.5;
    point_array[2].y = 0.5;
    point_array[2].z = 1.5;

    point_array[3].x = 0.5;
    point_array[3].y = 1.5;
    point_array[3].z = 1.5;          

    ArrayInit(num_array, 4);
    for (int i = 0; i < purple_cloud->points.size(); i++)
    {
        pcl::PointXYZ point = purple_cloud->points[i];

        pcl::PointXYZ point_one = GivePoint(vector, purple_keypoints.at(0), point_array[0].x, point_array[0].y, point_array[0].z);
        pcl::PointXYZ point_two = GivePoint(vector, purple_keypoints.at(0), point_array[1].x, point_array[1].y, point_array[1].z);
        pcl::PointXYZ point_three = GivePoint(vector, purple_keypoints.at(0), point_array[2].x, point_array[2].y, point_array[2].z);
        pcl::PointXYZ point_four = GivePoint(vector, purple_keypoints.at(0), point_array[3].x, point_array[3].y, point_array[3].z);

        if(Blue_btwCheck(point_one, purple_cloud->points[i]))
            { num_array[0]++; }
        if(Blue_btwCheck(point_two, purple_cloud->points[i]))
            { num_array[1]++; }
        if(Blue_btwCheck(point_three, purple_cloud->points[i]))
            { num_array[2]++; }
        if(Blue_btwCheck(point_four, purple_cloud->points[i]))
            { num_array[3]++; }
    }

    int min_index = 0;
    int min_num = 1000;
    for(int i = 0; i < 4; i ++)
    {
        if(i != max_index && min_num > num_array[i])
            { min_num = num_array[i]; min_index = i; }
    }

    int array[2] = {};
    int num = 0;
    for(int i = 0; i < 4; i++)
    {
        if(i != max_index && i != min_index)
        { 
            if(num == 0)
            {
                diyi_one = point_array[i];
                num++;
            } 
            else
            {
                diyi_two = point_array[i];
            }
        }
    }
}

pcl::PointCloud<pcl::PointXYZ> GetBrownSynthetic()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr brown_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    brown_cloud = brown_vector.at(0);

    pcl::PointCloud<pcl::PointXYZ> output_cloud;

    float block_width = getDistant(brown_keypoints.at(0), brown_keypoints.at(1));
    float block_depth = getDistant(brown_keypoints.at(0), brown_keypoints.at(3));

    int width_step = getStep(block_width);
    int depth_step = getStep(block_depth);
    int height_step = getStep(brown_block_height);

    if(width_step * depth_step * height_step == 8)
    {   

        std::vector<float> width_vector = std::vector<float> (3);
        width_vector.at(0) = brown_keypoints.at(1).x - brown_keypoints.at(0).x;
        width_vector.at(1) = brown_keypoints.at(1).y - brown_keypoints.at(0).y;
        width_vector.at(2) = brown_keypoints.at(1).z - brown_keypoints.at(0).z;

        float norm = sqrt(pow(width_vector.at(0), 2) + pow(width_vector.at(1),2) + pow(width_vector.at(2), 2));

        width_vector.at(0) = width_vector.at(0) / norm;
        width_vector.at(1) = width_vector.at(1) / norm;
        width_vector.at(2) = width_vector.at(2) / norm;

        std::vector<float> depth_vector = std::vector<float> (3);
        depth_vector.at(0) = brown_keypoints.at(3).x - brown_keypoints.at(0).x;
        depth_vector.at(1) = brown_keypoints.at(3).y - brown_keypoints.at(0).y;
        depth_vector.at(2) = brown_keypoints.at(3).z - brown_keypoints.at(0).z;

        norm = sqrt(pow(depth_vector.at(0), 2) + pow(depth_vector.at(1),2) + pow(depth_vector.at(2), 2));

        depth_vector.at(0) = depth_vector.at(0) / norm;
        depth_vector.at(1) = depth_vector.at(1) / norm;
        depth_vector.at(2) = depth_vector.at(2) / norm;

        std::vector<float> height_vector = std::vector<float> (3);
        height_vector.at(0) = unit_vector.at(0);
        height_vector.at(1) = unit_vector.at(1);
        height_vector.at(2) = unit_vector.at(2); // It must be same at any color


        std::vector<std::vector<float> > vector = std::vector<std::vector<float> > (3);
        vector.at(0) = height_vector;
        vector.at(1) = width_vector;
        vector.at(2) = depth_vector; 

        pcl::PointXYZ dier;
        pcl::PointXYZ diyi_one;
        pcl::PointXYZ diyi_two;

        Brown_CheckDist(vector, brown_cloud, dier, diyi_one, diyi_two);

        output_cloud.points.push_back(GivePoint(vector, brown_keypoints.at(0), dier.x, dier.y, dier.z));
        output_cloud.points.push_back(GivePoint(vector, brown_keypoints.at(0), dier.x - 1, dier.y, dier.z));
        output_cloud.points.push_back(GivePoint(vector, brown_keypoints.at(0), diyi_one.x, diyi_one.y, diyi_one.z));
        output_cloud.points.push_back(GivePoint(vector, brown_keypoints.at(0), diyi_two.x, diyi_two.y, diyi_two.z));
        
        if(output_cloud.points.size() == 4)
            { pcl::io::savePLYFileASCII ("output_cloud/brown_output_cloud.ply", output_cloud); return output_cloud; }
    }      
}
void Brown_CheckDist(std::vector<std::vector<float> > vector, pcl::PointCloud<pcl::PointXYZ>::Ptr brown_cloud, pcl::PointXYZ& dier, pcl::PointXYZ& diyi_one, pcl::PointXYZ& diyi_two)
{
    pcl::PointXYZ point_array[4] = {};

    point_array[0].x = 1.5;
    point_array[0].y = 0.5;
    point_array[0].z = 0.5;

    point_array[1].x = 1.5;
    point_array[1].y = 0.5;
    point_array[1].z = 1.5;

    point_array[2].x = 1.5;
    point_array[2].y = 1.5;
    point_array[2].z = 0.5;

    point_array[3].x = 1.5;
    point_array[3].y = 1.5;
    point_array[3].z = 1.5;        

    int num_array[4] = {};

    ArrayInit(num_array, 4);
    for (int i = 0; i < brown_cloud->points.size(); i++)
    {
        pcl::PointXYZ point = brown_cloud->points[i];

        pcl::PointXYZ point_one = GivePoint(vector, brown_keypoints.at(0), point_array[0].x, point_array[0].y, point_array[0].z);
        pcl::PointXYZ point_two = GivePoint(vector, brown_keypoints.at(0), point_array[1].x, point_array[1].y, point_array[1].z);
        pcl::PointXYZ point_three = GivePoint(vector, brown_keypoints.at(0), point_array[2].x, point_array[2].y, point_array[2].z);
        pcl::PointXYZ point_four = GivePoint(vector, brown_keypoints.at(0), point_array[3].x, point_array[3].y, point_array[3].z);

        if(Blue_btwCheck(point_one, brown_cloud->points[i]))
            { num_array[0]++; }
        if(Blue_btwCheck(point_two, brown_cloud->points[i]))
            { num_array[1]++; }
        if(Blue_btwCheck(point_three, brown_cloud->points[i]))
            { num_array[2]++; }
        if(Blue_btwCheck(point_four, brown_cloud->points[i]))
            { num_array[3]++; }
    }

    int max_index = 0;
    int max_num = 0; 
    for(int i = 0; i < 4; i++)
    {
        if(max_num < num_array[i])
            { max_num = num_array[i]; max_index = i; }
    }

    dier = point_array[max_index];

    // Now Get diling

    point_array[0].x = 0.5;
    point_array[0].y = 0.5;
    point_array[0].z = 0.5;

    point_array[1].x = 0.5;
    point_array[1].y = 1.5;
    point_array[1].z = 0.5;

    point_array[2].x = 0.5;
    point_array[2].y = 0.5;
    point_array[2].z = 1.5;

    point_array[3].x = 0.5;
    point_array[3].y = 1.5;
    point_array[3].z = 1.5;        

    ArrayInit(num_array, 4);
    for (int i = 0; i < brown_cloud->points.size(); i++)
    {
        pcl::PointXYZ point = brown_cloud->points[i];

        pcl::PointXYZ point_one = GivePoint(vector, brown_keypoints.at(0), point_array[0].x, point_array[0].y, point_array[0].z);
        pcl::PointXYZ point_two = GivePoint(vector, brown_keypoints.at(0), point_array[1].x, point_array[1].y, point_array[1].z);
        pcl::PointXYZ point_three = GivePoint(vector, brown_keypoints.at(0), point_array[2].x, point_array[2].y, point_array[2].z);
        pcl::PointXYZ point_four = GivePoint(vector, brown_keypoints.at(0), point_array[3].x, point_array[3].y, point_array[3].z);

        if(Blue_btwCheck(point_one, brown_cloud->points[i]))
            { num_array[0]++; }
        if(Blue_btwCheck(point_two, brown_cloud->points[i]))
            { num_array[1]++; }
        if(Blue_btwCheck(point_three, brown_cloud->points[i]))
            { num_array[2]++; }
        if(Blue_btwCheck(point_four, brown_cloud->points[i]))
            { num_array[3]++; }
    }

    int min_index = 0;
    int min_num = 1000;
    for(int i = 0; i < 4; i ++)
    {
        if(i != max_index && min_num > num_array[i])
            { min_num = num_array[i]; min_index = i; }
    }

    int num = 0;
    for(int i = 0; i < 4; i++)
    {
        if(i != max_index && i != min_index)
        { 
            if(num == 0)
            {
                diyi_one = point_array[i];
                num++;
            } 
            else
            {
                diyi_two = point_array[i];
            }
        }
    }
}

int GetPose(std::string color_string, pcl::PointCloud<pcl::PointXYZ> color_synthetic, std::vector<double> &array)
{   
    pcl::PointCloud<pcl::PointXYZ> source;
    pcl::PointCloud<pcl::PointXYZ> target;


    if(color_string == "red")
    {
        pcl::io::loadPCDFile<pcl::PointXYZ> ("blocks/red.pcd", source);// read 
    }
    else if(color_string == "orange")
    {
        pcl::io::loadPCDFile<pcl::PointXYZ> ("blocks/orange.pcd", source);
    }
    else if(color_string == "yellow")
    {
        pcl::io::loadPCDFile<pcl::PointXYZ> ("blocks/yellow.pcd", source);
    }
    else if(color_string == "green")
    {
        pcl::io::loadPCDFile<pcl::PointXYZ> ("blocks/green.pcd", source);
    }
    else if(color_string == "blue")
    {
        pcl::io::loadPCDFile<pcl::PointXYZ> ("blocks/blue.pcd", source);
    }
    else if(color_string == "purple")
    {
        pcl::io::loadPCDFile<pcl::PointXYZ> ("blocks/purple.pcd", source);
    }
    else
    {
        pcl::io::loadPCDFile<pcl::PointXYZ> ("blocks/brown.pcd", source);
    }

    target = color_synthetic;
    if(target.empty())
        { return 1; }

    int division = 5;
    double num = 0;
    double max_num = 125;

    for (int x = 0; x < division; x++)
    {
        for (int y = 0; y < division; y++)
        {
            for (int z = 0; z < division; z++)
            {   
                num = num + 1;


                pcl::PointCloud<pcl::PointXYZ> rot_source;
                Eigen::Matrix4f trans_rot = Eigen::Matrix4f::Identity();


                rot_source = Rotate(source, x, y, z, division, trans_rot);

                Eigen::Matrix4f trans_icp = Eigen::Matrix4f::Identity();
                double score = ICP(rot_source, target, trans_icp);

                if(score > 1e-14)
                {   
                    if(num == max_num){ return(1);} 
                    else{ break; }
                }
                
                Eigen::Matrix4f trans_tot = Eigen::Matrix4f::Identity();
                trans_tot = MatrixMul(trans_icp, trans_rot);

                cout << "transformation matrix" << " of " << color_string << " : " << endl;
                cout << trans_tot << endl;
                
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {   
                        array.at(4*i + j) = trans_tot (i, j);
                    }
                }
                return(0);
            }
        }
    }
}

pcl::PointCloud<pcl::PointXYZ> Rotate(pcl::PointCloud<pcl::PointXYZ> source, int x, int y, int z, int division, Eigen::Matrix4f &trans_rot)
{
    Eigen::Matrix4f x_rot = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f y_rot = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f z_rot = Eigen::Matrix4f::Identity();
    
    double x_angle = (double) x / division* 2 * PI;
    double y_angle = (double) y / division* 2 * PI;
    double z_angle = (double) z / division* 2 * PI;

    x_rot (0,0) = 1;
    x_rot (0,1) = 0;
    x_rot (0,2) = 0;

    x_rot (1,0) = 0;
    x_rot (1,1) = std::cos (x_angle);
    x_rot (1,2) = - sin (x_angle);

    x_rot (2,0) = 0;
    x_rot (2,1) = sin (x_angle);
    x_rot (2,2) = std::cos (x_angle);

    y_rot (0,0) = std::cos (y_angle);
    y_rot (0,1) = 0;
    y_rot (0,2) = sin (y_angle);

    y_rot (1,0) = 0;
    y_rot (1,1) = 1;
    y_rot (1,2) = 0;

    y_rot (2,0) = - sin (y_angle);
    y_rot (2,1) = 0;
    y_rot (2,2) = std::cos (y_angle);   

    z_rot (0,0) = std::cos (z_angle);
    z_rot (0,1) = - sin (z_angle);
    z_rot (0,2) = 0;

    z_rot (1,0) = sin (z_angle);
    z_rot (1,1) = std::cos (z_angle);
    z_rot (1,2) = 0;

    z_rot (2,0) = 0;
    z_rot (2,1) = 0;
    z_rot (2,2) = 1;      

    trans_rot = MatrixMul(z_rot, MatrixMul(y_rot, x_rot)); // Save trasformation

    pcl::PointCloud<pcl::PointXYZ> rot_source;
    pcl::transformPointCloud (source, rot_source, x_rot);
    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr rot_source_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    *source_ptr = source;
    *rot_source_ptr = rot_source;
    CloudViewer(source_ptr, rot_source_ptr);
    */
    pcl::transformPointCloud (rot_source, rot_source, y_rot);
    pcl::transformPointCloud (rot_source, rot_source, z_rot);
    
    return rot_source;
}

Eigen::Matrix4f MatrixMul(Eigen::Matrix4f obj_mat, Eigen::Matrix4f to_mat)
{   
    Eigen::Matrix4f output_mat;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {   
            double sum = 0;
            for (int k = 0; k < 4; k++)
            {
                sum += obj_mat (i, k) * to_mat (k, j);
            }
            output_mat (i, j) = sum;
        }
    }

    return output_mat;
}

double ICP(pcl::PointCloud<pcl::PointXYZ> source, pcl::PointCloud<pcl::PointXYZ> target, Eigen::Matrix4f &trans_icp)
{
    // 1. translation
    std::vector<double> source_cen = std::vector<double> (3);
    std::vector<double> target_cen = std::vector<double> (3);
    source_cen = GetCenter(source);
    target_cen = GetCenter(target); 

    std::vector<double> move = std::vector<double> (3);
    move.at(0) = target_cen.at(0) - source_cen.at(0);
    move.at(1) = target_cen.at(1) - source_cen.at(1);
    move.at(2) = target_cen.at(2) - source_cen.at(2);

    pcl::PointCloud<pcl::PointXYZ> trans_source;
    Eigen::Matrix4f trans_init = Eigen::Matrix4f::Identity();
    trans_source = Translation(source, move, trans_init);
    
    Eigen::Matrix4f trans_last = Eigen::Matrix4f::Identity();
    double score  = alignment(trans_source, target, trans_last);

    if(score <= 1e-14)
        { trans_icp = MatrixMul(trans_last, trans_init);}

    return score;
}

std::vector<double> GetCenter(pcl::PointCloud<pcl::PointXYZ> point_cloud)
{
    int points_num = 0;
    double x_sum = 0;
    double y_sum = 0;
    double z_sum = 0;

    for (int i = 0; i < point_cloud.points.size (); i++)
    { 
        if(point_cloud.points[i].x == 0 && point_cloud.points[i].y == 0 && point_cloud.points[i].z == 0)
        {}
        else
        {
            points_num = points_num + 1;
            x_sum = point_cloud.points[i].x + x_sum;
            y_sum = point_cloud.points[i].y + y_sum;
            z_sum = point_cloud.points[i].z + z_sum;
        }

    }

    double x_center = x_sum / points_num;
    double y_center = y_sum / points_num;
    double z_center = z_sum / points_num;

    std::vector<double> center = std::vector<double> (3);
    center.at(0) = x_center;
    center.at(1) = y_center;
    center.at(2) = z_center;

    return center;
}

pcl::PointCloud<pcl::PointXYZ> Translation(pcl::PointCloud<pcl::PointXYZ> source, std::vector<double> move, Eigen::Matrix4f &translation)
{
    float theta = 0;

    translation (0,3) = move.at(0);
    translation (1,3) = move.at(1);
    translation (2,3) = move.at(2);

    pcl::PointCloud<pcl::PointXYZ> trans_source;
    pcl::transformPointCloud (source, trans_source, translation);

    return trans_source;
}

double alignment(pcl::PointCloud<pcl::PointXYZ> source, pcl::PointCloud<pcl::PointXYZ> target, Eigen::Matrix4f &trans)
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    *source_ptr = source;
    *target_ptr = target;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_ptr);
    icp.setInputTarget(target_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*final);    

    trans = icp.getFinalTransformation();   

    return icp.getFitnessScore();
}

