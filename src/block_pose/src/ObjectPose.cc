#include <DominantPlane.h>
#include <ObjectPose.h>
#include <opencv2/opencv.hpp>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <opencv2/line_descriptor.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
using namespace cv::line_descriptor;
using namespace std;

ObjectPose::ObjectPose(int _height, int _width, int _Accum_iter,float _fx, float _fy, float _cx, 
                       float _cy, float _Unit_Cube_L, float _dist_thresh, Plane::DominantPlane* plane, cv::FileStorage fconfig)
{
    height = _height;
    width = _width;
    Accum_iter = _Accum_iter;
    Accum_idx = 0; 
    plane_object = plane;
    fx = _fx; 
    fy = _fy;
    cx = _cx; 
    cy = _cy;
    Unit_Cube_L = _Unit_Cube_L;
    dist_thresh = _dist_thresh;
    best_plane = plane->cur_best_plane;
    box_flag=0;
    cv::Mat _Projected_image = cv::Mat::zeros(height, width, CV_8UC3);
    // initilaize Grid vector
    std::vector<int> red_Grid(3);
    std::vector<int> yellow_Grid(3);
    std::vector<int> green_Grid(3);
    std::vector<int> blue_Grid(3);
    std::vector<int> brown_Grid(3);
    std::vector<int> orange_Grid(3);
    std::vector<int> Indigo_Grid(3);
    // initilaize Occupancy Grid vector
    std::vector<int> red_occ_Grid(3);
    std::vector<int> yellow_occ_Grid(3);
    std::vector<int> green_occ_Grid(3);
    std::vector<int> blue_occ_Grid(3);
    std::vector<int> brown_occ_Grid(3);
    std::vector<int> orange_occ_Grid(3);
    std::vector<int> Indigo_occ_Grid(3);
    // initilaize Bounding box information vector
    std::vector<Point3D> BB_info_red(8);
    std::vector<Point3D> BB_info_yellow(8);
    std::vector<Point3D> BB_info_green(8);
    std::vector<Point3D> BB_info_blue(8);
    std::vector<Point3D> BB_info_brown(8);
    std::vector<Point3D> BB_info_orange(8);
    std::vector<Point3D> BB_info_Indigo(8);
    // initialize Block center information vector
    std::vector<Point3D> Block_center_red(3);
    std::vector<Point3D> Block_center_yellow(4);
    std::vector<Point3D> Block_center_green(4);
    std::vector<Point3D> Block_center_blue(4);
    std::vector<Point3D> Block_center_brown(4);
    std::vector<Point3D> Block_center_orange(4);
    std::vector<Point3D> Block_center_Indigo(4);
    // verbose for pose class flag
    Test_all_flag = fconfig["Pose.Test_AllBlock"];
    Test_Individual_flag = fconfig["Pose.Test_IndividualBlock"];
    Test_red_flag = fconfig["Pose.TestRed"];
    Test_yellow_flag = fconfig["Pose.TestYellow"];
    Test_green_flag = fconfig["Pose.TestGreen"];
    Test_blue_flag = fconfig["Pose.TestBlue"];
    Test_brown_flag = fconfig["Pose.TestBrown"];
    Test_orange_flag = fconfig["Pose.TestOrange"];
    Test_indigo_flag = fconfig["Pose.TestIndigo"];
    // Debug Object
    Debug_Object = fconfig["Pose.DebugObject"];
    cout << "------------------------- Pose Test flags -------------------------" << endl;
    cout << "Test all:" << Test_all_flag << endl;
    cout << "Test Red:" << Test_red_flag << endl;
    cout << "Test Yellow:" << Test_yellow_flag << endl;
    cout << "Test Green:" << Test_green_flag << endl;
    cout << "Test Orange:" << Test_orange_flag << endl;
    cout << "Test Blue:" << Test_blue_flag << endl;
    cout << "Test Brown:" << Test_brown_flag << endl;
    cout << "Test Indigo:" << Test_indigo_flag << endl;
    cout << "DEBUG FLAG: " << Debug_Object << endl;
}

void ObjectPose::Accumulate_PointCloud(cv::Mat &pcd_outlier, std::vector<cv::Mat> &Mask)
{
    //red, yellow, green, blue, brown, orange, Indigo
    for(int i = 0; i < Mask.size(); i++)
    {
        if(i==0) // red
        {
            for(int y = 0; y < height ; y++)
            {
                for( int x = 0; x < width; x++ )
                {
                    if(Mask[i].at<uint8_t>(y,x) == 255)
                    {
                        pcl::PointXYZRGB point;
                        point.x = pcd_outlier.at<cv::Vec3f>(y,x)[0];
                        point.y = pcd_outlier.at<cv::Vec3f>(y,x)[1];
                        point.z = pcd_outlier.at<cv::Vec3f>(y,x)[2];
                        point.r = 255;
                        point.g = 0; 
                        point.b = 0; 
                        if(point.x!=0)
                        {
                            red_cloud.push_back(point);
                        }
                    }
                }
            }
        }

        if(i==1) // yellow
        {
            for(int y = 0; y < height ; y++)
            {
                for( int x = 0; x < width; x++ )
                {
                    if(Mask[i].at<uint8_t>(y,x) == 255)
                    {
                        pcl::PointXYZRGB point;
                        point.x = pcd_outlier.at<cv::Vec3f>(y,x)[0];
                        point.y = pcd_outlier.at<cv::Vec3f>(y,x)[1];
                        point.z = pcd_outlier.at<cv::Vec3f>(y,x)[2];
                        point.r = 255;
                        point.g = 255; 
                        point.b = 0; 
                        if(point.x!=0)
                        {
                            yellow_cloud.push_back(point);
                        }
                    }
                }
            }
        }

        if(i==2) // green
        {
            for(int y = 0; y < height ; y++)
            {
                for( int x = 0; x < width; x++ )
                {
                    if(Mask[i].at<uint8_t>(y,x) == 255)
                    {
                        pcl::PointXYZRGB point;
                        point.x = pcd_outlier.at<cv::Vec3f>(y,x)[0];
                        point.y = pcd_outlier.at<cv::Vec3f>(y,x)[1];
                        point.z = pcd_outlier.at<cv::Vec3f>(y,x)[2];
                        point.r = 0;
                        point.g = 128; 
                        point.b = 0; 
                        if(point.x!=0)
                        {
                            green_cloud.push_back(point);
                        }
                    }
                }
            }
        }

        if(i==3) // blue
        {
            for(int y = 0; y < height ; y++)
            {
                for( int x = 0; x < width; x++ )
                {
                    if(Mask[i].at<uint8_t>(y,x) == 255)
                    {
                        pcl::PointXYZRGB point;
                        point.x = pcd_outlier.at<cv::Vec3f>(y,x)[0];
                        point.y = pcd_outlier.at<cv::Vec3f>(y,x)[1];
                        point.z = pcd_outlier.at<cv::Vec3f>(y,x)[2];
                        point.r = 0;
                        point.g = 0;
                        point.b = 255; 
                        if(point.x!=0)
                        {
                            blue_cloud.push_back(point);
                        }
                    }
                }
            }
        }

        if(i==4) // brown
        {
            for(int y = 0; y < height ; y++)
            {
                for( int x = 0; x < width; x++ )
                {
                    if(Mask[i].at<uint8_t>(y,x) == 255)
                    {
                        pcl::PointXYZRGB point;
                        point.x = pcd_outlier.at<cv::Vec3f>(y,x)[0];
                        point.y = pcd_outlier.at<cv::Vec3f>(y,x)[1];
                        point.z = pcd_outlier.at<cv::Vec3f>(y,x)[2];
                        point.r = 72;
                        point.g = 60;
                        point.b = 50; 
                        if(point.x!=0)
                        {
                            brown_cloud.push_back(point);
                        }
                    }
                }
            }
        }

        if(i==5) // orange
        {
            for(int y = 0; y < height ; y++)
            {
                for( int x = 0; x < width; x++ )
                {
                    if(Mask[i].at<uint8_t>(y,x) == 255)
                    {
                        pcl::PointXYZRGB point;
                        point.x = pcd_outlier.at<cv::Vec3f>(y,x)[0];
                        point.y = pcd_outlier.at<cv::Vec3f>(y,x)[1];
                        point.z = pcd_outlier.at<cv::Vec3f>(y,x)[2];
                        point.r = 255;
                        point.g = 165;
                        point.b = 0; 
                        if(point.x!=0)
                        {
                            orange_cloud.push_back(point);
                        }
                    }
                }
            }
        }

        if(i==6) // Indigo
        {
            for(int y=0; y < height; y++)
            {
                for(int x=0; x < width; x++)
                {
                    if(Mask[i].at<uint8_t>(y,x) == 255)
                    {
                        pcl::PointXYZRGB point; 
                        point.x = pcd_outlier.at<cv::Vec3f>(y,x)[0];
                        point.y = pcd_outlier.at<cv::Vec3f>(y,x)[1];
                        point.z = pcd_outlier.at<cv::Vec3f>(y,x)[2];
                        point.r = 40; 
                        point.g = 50; 
                        point.b = 100;
                        if(point.x!=0)
                        {
                            Indigo_cloud.push_back(point);
                        }
                    }
                }
            }
        }

    }

    Accum_idx++;
    // Initialize Total DominanPlane Projected Image
    Total_Projected_image = cv::Mat::zeros(height, width, CV_8UC3);
    if(Accum_idx >= Accum_iter)
    {
        if(Test_all_flag==1)
        {
            ProjectToDominantPlane(red_cloud, "red");
            ProjectToDominantPlane(yellow_cloud, "yellow");
            ProjectToDominantPlane(green_cloud, "green");
            ProjectToDominantPlane(blue_cloud, "blue");
            ProjectToDominantPlane(brown_cloud, "brown");
            ProjectToDominantPlane(orange_cloud, "orange");
            ProjectToDominantPlane(Indigo_cloud, "Indigo");
        }

        if(Test_Individual_flag==1)
        {
            if(Test_red_flag==1)
                ProjectToDominantPlane(red_cloud, "red");
            if(Test_yellow_flag==1)
                ProjectToDominantPlane(yellow_cloud, "yellow");
            if(Test_blue_flag==1)
                ProjectToDominantPlane(blue_cloud, "blue");
            if(Test_green_flag==1)
                ProjectToDominantPlane(green_cloud, "green");
            if(Test_brown_flag==1)
                ProjectToDominantPlane(brown_cloud, "brown");
            if(Test_orange_flag==1)
                ProjectToDominantPlane(orange_cloud, "orange");
            if(Test_indigo_flag==1)
                ProjectToDominantPlane(Indigo_cloud, "Indigo");
        }
        cv::imshow("Drawing", Total_Projected_image);
        cv::waitKey(2);
        Accum_idx = 0; 
    }

}



void ObjectPose::ProjectToDominantPlane(pcl::PointCloud<pcl::PointXYZRGB> in_cloud, std::string _color_string)
{
    color_string = _color_string;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_projected_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    best_plane = plane_object->cur_best_plane;
    float a = best_plane.a;
    float b = best_plane.b; 
    float c = best_plane.c; 
    float d = best_plane.d; 
    bool first_index=1;
    pcl::PointXYZRGB prev_point; 
    for(int i = 0; i < in_cloud.size(); i++)
    {
        pcl::PointXYZRGB point = in_cloud[i];
        float t = (-a*point.x - b*point.y - c*point.z - d)/(a*a+b*b+c*c);
        pcl::PointXYZRGB ProjectedPoint; 
        ProjectedPoint.x = point.x + t*a; 
        ProjectedPoint.y = point.y + t*b; 
        ProjectedPoint.z = point.z + t*c; 
        if(color_string=="red")
        {
            ProjectedPoint.r = 255; 
            ProjectedPoint.g = 0; 
            ProjectedPoint.b = 0; 
        }
        else if(color_string=="yellow")
        {
            ProjectedPoint.r = 255; 
            ProjectedPoint.g = 255; 
            ProjectedPoint.b = 0; 
        }
        else if(color_string=="green")
        {
            ProjectedPoint.r = 0; 
            ProjectedPoint.g = 255; 
            ProjectedPoint.b = 0; 
        }
        else if(color_string=="green")
        {
            ProjectedPoint.r = 0; 
            ProjectedPoint.g = 255; 
            ProjectedPoint.b = 0; 
        }
        else if(color_string=="blue")
        {
            ProjectedPoint.r = 0; 
            ProjectedPoint.g = 0; 
            ProjectedPoint.b = 255; 
        }
        else if(color_string=="brown")
        {
            ProjectedPoint.r = 72; 
            ProjectedPoint.g = 60; 
            ProjectedPoint.b = 50; 
        }
        else if(color_string=="orange")
        {
            ProjectedPoint.r = 255; 
            ProjectedPoint.g = 165;
            ProjectedPoint.b = 0;
        }
        else if(color_string=="Indigo")
        {
            ProjectedPoint.r = 40; 
            ProjectedPoint.g = 50;
            ProjectedPoint.b = 100;
        }
        pcl_projected_cloud->points.push_back(ProjectedPoint);

        float dist_prev = sqrt(pow(ProjectedPoint.x - prev_point.x,2) + pow(ProjectedPoint.y - prev_point.y,2) + pow(ProjectedPoint.z - prev_point.z, 2));
        if(dist_prev < 0.1 && first_index==0)
        {
            pcl_projected_cloud->points.push_back(ProjectedPoint);
            prev_point = ProjectedPoint;
        }
        else
        {
        }
        if(first_index ==1)
        {
            first_index = 0;
            prev_point = ProjectedPoint;
        }
    }
    projected_cloud = *pcl_projected_cloud;
    ProjectedCloudToImagePlane();
}

void ObjectPose::ProjectedCloudToImagePlane()
{
    cv::Mat projected_image = cv::Mat::zeros(height, width, CV_8UC1);
    std::vector<pair<int, int>> pos_vector; 
    for (int i =0; i < projected_cloud.size(); i++)
    {
        pcl::PointXYZRGB temp_cloud; 
        temp_cloud = projected_cloud.points[i];
        float X = temp_cloud.x;
        float Y = temp_cloud.y; 
        float Z = temp_cloud.z; 
        int x = cy + fy*Y/Z;
        int y = cx + fx*X/Z;
        pos_vector.push_back(make_pair(x,y));
        projected_image.at<uchar>(x, y) = 255; 
    }
    _Projected_image = projected_image;
    fitRectangle(projected_image);
}


void ObjectPose::fitRectangle(cv::Mat projected_image)
{
    RNG rng(12345);
    int thresh = 0.1;
    std::vector<Point> RectPoints = std::vector<Point>(4);
    Mat canny_output= cv::Mat::zeros(height, width, CV_8UC1);
    vector<vector<Point>> contours;

    // filling small hole due to occlusion
    if(color_string=="red")
    {
        dilate(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(30,30)));
        erode(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(30,30)));
    }
    else if(color_string=="blue")
    {
        dilate(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));
        erode(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));
    }
    else if(color_string=="green")
    {
        dilate(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));
        erode(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));
    }
    else if(color_string=="orange")
    {
        dilate(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(40,40)));
        erode(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(40,40)));
    }
    else if(color_string=="brown")
    {
        dilate(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));
        erode(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));
    }
    else if(color_string=="yellow")
    {
        dilate(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(40,40)));
        erode(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(40,40)));
    }
    else if(color_string=="Indigo")
    {
        dilate(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));
        erode(projected_image, projected_image, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));
    }

    Canny(projected_image, canny_output, thresh, thresh*10, 3);
    findContours(canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    vector<RotatedRect> minRect(contours.size());
    for( size_t i = 0; i < contours.size(); i++ )
        minRect[i] = minAreaRect( contours[i] );

    double max_size = 100;
    for( size_t i = 0; i< contours.size(); i++ )
    {
        // rotated rectangle
        Point2f rect_points[4];
        minRect[i].points( rect_points );
        
        double dist_1 = sqrt(pow(rect_points[0].x - rect_points[1].x, 2) + pow(rect_points[0].y - rect_points[1].y, 2));
        double dist_2 = sqrt(pow(rect_points[1].x - rect_points[2].x, 2) + pow(rect_points[1].y - rect_points[2].y, 2));
        double size = dist_1 * dist_2;
        if(max_size < size)
        {
            max_size = size;
            for ( int j = 0; j < 4; j++ )
            {   
                RectPoints.at(j) = rect_points[j];
            }  
        }
    }
    _RectPoints = RectPoints;
    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
    BackProjectToDominatPlane(RectPoints);
}

void ObjectPose::BackProjectToDominatPlane(std::vector<cv::Point> Rect_points)
{
    float a = best_plane.a; 
    float b = best_plane.b; 
    float c = best_plane.c;
    float d = best_plane.d;
    float Z_deominator = std::numeric_limits<float>::max();
    float max_length;
    if(color_string=="red")
        max_length = FindBlockHeight(red_cloud, a, b, c, d);
    else if(color_string=="green")
        max_length = FindBlockHeight(green_cloud, a, b, c, d);
    else if(color_string=="blue")
        max_length = FindBlockHeight(blue_cloud, a, b, c, d);
    else if(color_string=="yellow")
        max_length = FindBlockHeight(yellow_cloud, a, b, c, d);
    else if(color_string=="orange")
        max_length = FindBlockHeight(orange_cloud, a, b, c, d);
    else if(color_string=="brown")
        max_length = FindBlockHeight(brown_cloud, a, b, c, d);
    else if(color_string=="Indigo")
        max_length = FindBlockHeight(Indigo_cloud, a, b, c, d);

    std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> BB_points; 
    for (int i = 0; i < Rect_points.size(); i++)
    {
        float x = Rect_points[i].x;
        float y = Rect_points[i].y;
        Z_deominator = a*(x-cx)/fx+b*(y-cy)/fy +c;
        float Z = -d/Z_deominator;
        float X = (x-cx)/fx*Z;
        float Y = (y-cy)/fy*Z;
        pcl::PointXYZRGB temp_cloud;
        temp_cloud.x = X; 
        temp_cloud.y = Y;
        temp_cloud.z = Z;
        temp_cloud.g = 255; 
        pcl::PointXYZRGB temp_cloud_upper;
        temp_cloud_upper.x = X + a*max_length;
        temp_cloud_upper.y = Y + b*max_length; 
        temp_cloud_upper.z = Z + c*max_length;
        temp_cloud_upper.g = 255; 
        BB_points.push_back(make_pair(temp_cloud,temp_cloud_upper));
    }

    BBinfo_temp = BB_points;
    // Find Occupancy Grid
    FindOccGrid(BB_points, max_length);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_for_viewer(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(color_string=="red")
    {
        // cout << "red height: " << max_length << endl;
        // cout << "red point cloud size: " << red_cloud.size() << endl;
        // pcl::copyPointCloud(red_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
        // cout << color_string << " Grid: " << red_Grid[0] << " " << red_Grid[1] << " " << red_Grid[2] << endl;
    }

    else if(color_string=="yellow")
    {
        // cout << "yellow height: " << max_length << endl;
        // cout << "yellow point cloud size: " << yellow_cloud.size() << endl;
        // pcl::copyPointCloud(yellow_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
        // FindOccGrid(pos_vector, max_length);
        // cout << color_string << " Grid: " << yellow_Grid[0] << " " << yellow_Grid[1] << " " << yellow_Grid[2] << endl;
    }

    else if(color_string=="green")
    {
        // cout << "green height: " << max_length << endl;
        // pcl::copyPointCloud(green_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
    }

    else if(color_string=="blue")
    {
        // cout << "blue height: " << max_length << endl;
        // pcl::copyPointCloud(blue_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
        // FindOccGrid(pos_vector, max_length);
        // cout << color_string << " Grid" << blue_Grid[0] << " " << blue_Grid[1] << " " << blue_Grid[2] << endl;
    }

    else if(color_string=="brown")
    {
        // cout << "brown height: " << max_length << endl;
        // pcl::copyPointCloud(brown_cloud, *Cloud_for_viewer);
        // FindOccGrid(pos_vector, max_length);
        // cout << color_string << " Grid" << brown_Grid[0] << " " << brown_Grid[1] << " " << brown_Grid[2] << endl;
        // CloudView(Cloud_for_viewer, pos_vector);
    }

    else if(color_string=="orange")
    {
        // cout << "orange height: " << max_length << endl;
        // pcl::copyPointCloud(orange_cloud, *Cloud_for_viewer);
        // FindOccGrid(pos_vector, max_length);
        // CloudView(Cloud_for_viewer, pos_vector);
        // cout << color_string << " Grid" << orange_Grid[0] << " " << orange_Grid[1] << " " << orange_Grid[2] << endl;
    }

    else if(color_string=="Indigo")
    {
        // cout << "Indigo height: " << max_length << endl;
        // pcl::copyPointCloud(Indigo_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, BB_points);
        // FindOccGrid(pos_vector, max_length);
        // cout << color_string << " Grid" << Indigo_Grid[0] << " " << Indigo_Grid[1] << " " << Indigo_Grid[2] << endl;
    }
}

float ObjectPose::FindBlockHeight(pcl::PointCloud<pcl::PointXYZRGB> in_cloud, float a, float b, float c, float d)
{
    float max_height=0;
    float dist_temp;
    float denom = std::sqrt(a*a + b*b + c*c);
    for(int i=0; i<in_cloud.size(); i++)
    {
        pcl::PointXYZRGB temp_cloud = in_cloud[i];
        dist_temp = std::abs(a*temp_cloud.x + b*temp_cloud.y + c*temp_cloud.z + d)/denom;
        if(max_height<dist_temp)
            max_height = dist_temp;
    }

    // Discretize max height
    if(max_height > Unit_Cube_L-dist_thresh & max_height < Unit_Cube_L+dist_thresh)
        max_height = Unit_Cube_L+0.001;
    else if(max_height > 2*Unit_Cube_L - dist_thresh & max_height < 2*Unit_Cube_L + dist_thresh)
        max_height = 2*Unit_Cube_L+0.003;
    else if(max_height > 3*Unit_Cube_L - dist_thresh & max_height < 3*Unit_Cube_L + dist_thresh)
        max_height = 3*Unit_Cube_L+0.004;
    else if(max_height > 4*Unit_Cube_L - dist_thresh & max_height < 4*Unit_Cube_L + dist_thresh)
        max_height = 4*Unit_Cube_L+0.005;
    else
        max_height = 0; 
    return max_height;
}

float ObjectPose::FindBlockMeanHeight(pcl::PointCloud<pcl::PointXYZRGB> in_cloud, float a, float b, float c, float d)
{
    float max_height=0;
    float dist_temp;
    float denom = std::sqrt(a*a + b*b + c*c);
    float dist_tot = 0; 
    float dist_mean = 0;
    for(int i=0; i<in_cloud.size(); i++)
    {
        pcl::PointXYZRGB temp_cloud = in_cloud[i];
        dist_temp = std::abs(a*temp_cloud.x + b*temp_cloud.y + c*temp_cloud.z + d)/denom;
        dist_tot += dist_temp;
    }
    dist_mean = dist_tot/in_cloud.size();
    return dist_mean;
}

void ObjectPose::FindOccGrid(std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector, float max_height)
{
    std::vector<int> Grid_size(3);
    pcl::PointXYZRGB BB_vertex1 = std::get<0>(pos_vector[0]);
    pcl::PointXYZRGB BB_vertex2 = std::get<0>(pos_vector[1]);
    pcl::PointXYZRGB BB_vertex3 = std::get<0>(pos_vector[2]);
    pcl::PointXYZRGB BB_vertex4 = std::get<0>(pos_vector[3]);
    pcl::PointXYZRGB BB_vertex5 = std::get<1>(pos_vector[0]);
    float dist1 = std::sqrt(std::pow(BB_vertex1.x - BB_vertex2.x, 2) + std::pow(BB_vertex1.y - BB_vertex2.y, 2) + std::pow(BB_vertex1.z - BB_vertex2.z,2));
    float dist2 = std::sqrt(std::pow(BB_vertex1.x - BB_vertex4.x, 2) + std::pow(BB_vertex1.y - BB_vertex4.y, 2) + std::pow(BB_vertex1.z - BB_vertex4.z,2));

    if(dist1 > Unit_Cube_L-dist_thresh & dist1 < Unit_Cube_L+dist_thresh)
        Grid_size[0] = 1;
    else if(dist1 > 2*Unit_Cube_L-dist_thresh & dist1 < 2*Unit_Cube_L+dist_thresh)
        Grid_size[0] = 2;
    else if(dist1 > 3*Unit_Cube_L-dist_thresh & dist1 < 3*Unit_Cube_L+dist_thresh)
        Grid_size[0] = 3;
    else 
        Grid_size[0] = 1; 

    if(dist2 > Unit_Cube_L-dist_thresh & dist2 < Unit_Cube_L+dist_thresh)
        Grid_size[1] = 1;
    else if(dist2 > 2*Unit_Cube_L-dist_thresh & dist2 < 2*Unit_Cube_L+dist_thresh)
        Grid_size[1] = 2;
    else if(dist2 > 3*Unit_Cube_L-dist_thresh & dist2 < 3*Unit_Cube_L+dist_thresh)
        Grid_size[1] = 3;
    else 
        Grid_size[1] = 1; 

    if(max_height > Unit_Cube_L - dist_thresh & max_height < Unit_Cube_L + dist_thresh)
        Grid_size[2] = 1;
    else if(max_height > 2*Unit_Cube_L - dist_thresh & max_height < 2*Unit_Cube_L + dist_thresh)
        Grid_size[2] = 2;
    else if(max_height > 3*Unit_Cube_L - dist_thresh & max_height < 3*Unit_Cube_L + dist_thresh)
        Grid_size[2] = 3;
    else 
        Grid_size[2] = 1; 

    int cnt_tot = 0; 
    if(color_string=="red")
        MeasureOccupany(pos_vector, Grid_size, red_cloud);
    else if(color_string=="yellow")
        MeasureOccupany(pos_vector, Grid_size, yellow_cloud);
    else if(color_string=="green")
        MeasureOccupany(pos_vector, Grid_size, green_cloud);
    else if(color_string=="blue")
        MeasureOccupany(pos_vector, Grid_size, blue_cloud);
    else if(color_string=="brown")
        MeasureOccupany(pos_vector, Grid_size, brown_cloud);
    else if(color_string=="orange")
        MeasureOccupany(pos_vector, Grid_size, orange_cloud);
    else if(color_string=="Indigo")
        MeasureOccupany(pos_vector, Grid_size, Indigo_cloud);
}

void ObjectPose::MeasureOccupany(std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector, 
                                 std::vector<int> Grid_size, pcl::PointCloud<pcl::PointXYZRGB> ref_cloud)
{
    float k_vector_const = 1.3;
    pcl::PointXYZRGB BB_vertex1 = std::get<0>(pos_vector[0]);
    pcl::PointXYZRGB BB_vertex2 = std::get<0>(pos_vector[1]);
    pcl::PointXYZRGB BB_vertex3 = std::get<0>(pos_vector[2]);
    pcl::PointXYZRGB BB_vertex4 = std::get<0>(pos_vector[3]);
    pcl::PointXYZRGB BB_vertex5 = std::get<1>(pos_vector[0]);
    // i vector of Cube
    cv::Vec3f Cube_I(BB_vertex1.x - BB_vertex2.x, BB_vertex1.y - BB_vertex2.y, BB_vertex1.z - BB_vertex2.z);
    // j vector of Cube
    cv::Vec3f Cube_J(BB_vertex1.x - BB_vertex4.x, BB_vertex1.y - BB_vertex4.y, BB_vertex1.z - BB_vertex4.z);
    // k vector of Cube
    cv::Vec3f Cube_K(BB_vertex1.x - BB_vertex5.x, BB_vertex1.y - BB_vertex5.y, BB_vertex1.z - BB_vertex5.z);
    // I vector of Unit Cube
    cv::Vec3f Unit_Cube_I = Cube_I/Grid_size[0];
    // J vector of Unit Cube
    cv::Vec3f Unit_Cube_J = Cube_J/Grid_size[1];
    // k vector of Unit Cube
    cv::Vec3f Unit_Cube_K = k_vector_const*Cube_K/Grid_size[2];
    float Cube_I_value = std::sqrt(std::pow(Unit_Cube_I[0],2) + std::pow(Unit_Cube_I[1],2) + std::pow(Unit_Cube_I[2],2));
    float Cube_J_value = std::sqrt(std::pow(Unit_Cube_J[0],2) + std::pow(Unit_Cube_J[1],2) + std::pow(Unit_Cube_J[2],2));
    float Cube_K_value = std::sqrt(std::pow(Unit_Cube_K[0],2) + std::pow(Unit_Cube_K[1],2) + std::pow(Unit_Cube_K[2],2));

    // Unit vector of i, j, k 
    Unit_Cube_I /= Cube_I_value;
    Unit_Cube_J /= Cube_J_value;
    Unit_Cube_K /= Cube_K_value;

    int cnt_temp=0; // for pcd counting 
    int Grid_cnt=0;
    int Grid_tot_size = Grid_size[0]*Grid_size[1]*Grid_size[2];
    std::vector<int> Grid_pcd_cnt(Grid_tot_size);
    // Dominant plane a,b,c,d
    float a = best_plane.a;
    float b = best_plane.b;
    float c = best_plane.c;
    float d = best_plane.d;

    // TODO: Find max height of occ grid, and find occupancy grid it will be more accurate.
    for(int i=1; i <= Grid_size[0]; i++)
    {
        for(int j=1; j <= Grid_size[1]; j++)
        {
            for(int k=1; k <= Grid_size[2]; k++)
            {
                cv::Vec3f temp_cloud_vec3f = cv::Vec3f(BB_vertex1.x, BB_vertex1.y, BB_vertex1.z);
                temp_cloud_vec3f[0] -= (i-1)*Cube_I_value*Unit_Cube_I[0];
                temp_cloud_vec3f[1] -= (i-1)*Cube_I_value*Unit_Cube_I[1];
                temp_cloud_vec3f[2] -= (i-1)*Cube_I_value*Unit_Cube_I[2];
                temp_cloud_vec3f[0] -= (j-1)*Cube_J_value*Unit_Cube_J[0];
                temp_cloud_vec3f[1] -= (j-1)*Cube_J_value*Unit_Cube_J[1];
                temp_cloud_vec3f[2] -= (j-1)*Cube_J_value*Unit_Cube_J[2];
                temp_cloud_vec3f[0] -= (k-1)*Cube_K_value*Unit_Cube_K[0];
                temp_cloud_vec3f[1] -= (k-1)*Cube_K_value*Unit_Cube_K[1];
                temp_cloud_vec3f[2] -= (k-1)*Cube_K_value*Unit_Cube_K[2];
                for(int l=0; l < ref_cloud.size(); l++)
                {
                    pcl::PointXYZRGB temp_cloud = ref_cloud[l];
                    cv::Vec3f temp_cloud_vec3f2 = cv::Vec3f(temp_cloud_vec3f[0] - temp_cloud.x, temp_cloud_vec3f[1] - temp_cloud.y, temp_cloud_vec3f[2] - temp_cloud.z);
                    float dot_x = Unit_Cube_I.dot(temp_cloud_vec3f2);
                    float dot_y = Unit_Cube_J.dot(temp_cloud_vec3f2);
                    float dot_z = Unit_Cube_K.dot(temp_cloud_vec3f2);
                    if(0 < dot_x & dot_x < Cube_I_value & 0 < dot_y & dot_y < Cube_J_value & 0 < dot_z & dot_z < Cube_K_value)
                    {
                        cnt_temp++;
                    }
                }

                // cout << "i: " << i << "j: " << j << "k: " << k << endl;
                // cout << "cnt : " <<  cnt_temp << endl;
                // float mean_height = FindBlockMeanHeight(CubeInCloud, a, b, c, d);
                Grid_pcd_cnt[Grid_cnt] = cnt_temp;
                cnt_temp = 0; 
                if(k==Grid_size[2])
                {
                    if(k!=1)
                    {
                        int max_value = std::max(Grid_pcd_cnt[Grid_cnt], Grid_pcd_cnt[Grid_cnt-1]);
                        // cout << "max value: " << max_value << endl;
                        // cout << "Grid cnt value: " << Grid_pcd_cnt[Grid_cnt] << endl;
                        if(Grid_pcd_cnt[Grid_cnt]==max_value)
                            Grid_pcd_cnt[Grid_cnt-1] = 0;
                        else
                            Grid_pcd_cnt[Grid_cnt] = 0; 
                    }
                }
                Grid_cnt++;
            }
        }
    }

    std::vector<int> occ_grid(Grid_tot_size);
    for(int i = 0; i<Grid_tot_size; i++)
    {
        // cout << "cnt : " <<  Grid_pcd_cnt[i] << endl;
        if(Grid_pcd_cnt[i] > ref_cloud.size()/(4*Grid_tot_size))
            occ_grid[i] = 1; 
        else 
            occ_grid[i] = 0; 

        if(i!=0 & Grid_size[2]==2 & i%Grid_size[2]==Grid_size[2]-1 & occ_grid[i]==1)
            occ_grid[i-1] = 1;
        if(i>1 & Grid_size[2]==3 & occ_grid[i]==1)
        {
            occ_grid[i-1] = 1;
            occ_grid[i-2] = 1;
        }
    }
    cout << "(Before) Occ grid: " ;
    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
        cout << occ_grid[i] << " " ;
    cout << endl;
    int occ_cnt = 0; 
    for(int i = 0; i < Grid_tot_size; i++)
    {
        if(occ_grid[i]==1)
            occ_cnt++;
    }
    if(color_string=="green")
    {
        if(Grid_tot_size==6 && occ_cnt==5)
        {
            if(occ_grid[1]==0)
                occ_grid[4]=0;
            if(occ_grid[5]==0)
                occ_grid[1]=0;
        }
    }
    if(color_string=="blue")
    {
        if(Grid_tot_size==8 && occ_cnt==5)
        {
            if(occ_grid[0]==0 && occ_grid[1]==0 && occ_grid[4]==0)
                occ_grid[2]=0;
            if(occ_grid[2]==0 && occ_grid[3]==0 && occ_grid[7]==0)
                occ_grid[0]=0;
            if(occ_grid[5]==0 && occ_grid[6]==0 && occ_grid[7]==0)
                occ_grid[0]=0;
            if(occ_grid[5]==0 && occ_grid[6]==0 && occ_grid[7]==0)
                occ_grid[0]=0;
            /*
            0 0 1 1 1 0 1 1 -> 0 0 0 1 1 0 1 1 
            1 1 0 0 1 1 1 0 -> 0 1 0 0 1 1 1 0
            1 1 1 1 0 0 1 0 -> 0 1 1 1 0 0 1 0
            1 1 1 1 1 0 0 0 -> 1 1 1 0 1 0 0 0 
            */
        }
    }
    cout << "After Occ grid: " ;
    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
        cout << occ_grid[i] << " " ;
    cout << endl;

    CheckOccGridWithKnownShape(Grid_size, occ_grid);
}

void ObjectPose::GenerateRealSyntheticCloud(std::vector<int> Grid_size, std::vector<int> occ_Grid, std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector)
{
    pcl::PointXYZRGB BB_vertex1 = std::get<0>(pos_vector[0]);
    pcl::PointXYZRGB BB_vertex2 = std::get<0>(pos_vector[1]);
    pcl::PointXYZRGB BB_vertex3 = std::get<0>(pos_vector[2]);
    pcl::PointXYZRGB BB_vertex4 = std::get<0>(pos_vector[3]);
    pcl::PointXYZRGB BB_vertex5 = std::get<1>(pos_vector[0]);

    // i vector of Cube
    cv::Vec3f Cube_I(BB_vertex1.x - BB_vertex2.x, BB_vertex1.y - BB_vertex2.y, BB_vertex1.z - BB_vertex2.z);
    // j vector of Cube
    cv::Vec3f Cube_J(BB_vertex1.x - BB_vertex4.x, BB_vertex1.y - BB_vertex4.y, BB_vertex1.z - BB_vertex4.z);
    // k vector of Cube
    cv::Vec3f Cube_K(BB_vertex1.x - BB_vertex5.x, BB_vertex1.y - BB_vertex5.y, BB_vertex1.z - BB_vertex5.z);
    // I vector of Unit Cube
    cv::Vec3f Unit_Cube_I = Cube_I/Grid_size[0];
    // J vector of Unit Cube
    cv::Vec3f Unit_Cube_J = Cube_J/Grid_size[1];
    // k vector of Unit Cube
    cv::Vec3f Unit_Cube_K = Cube_K/Grid_size[2];
    float Cube_I_value = std::sqrt(std::pow(Unit_Cube_I[0],2) + std::pow(Unit_Cube_I[1],2) + std::pow(Unit_Cube_I[2],2));
    float Cube_J_value = std::sqrt(std::pow(Unit_Cube_J[0],2) + std::pow(Unit_Cube_J[1],2) + std::pow(Unit_Cube_J[2],2));
    float Cube_K_value = std::sqrt(std::pow(Unit_Cube_K[0],2) + std::pow(Unit_Cube_K[1],2) + std::pow(Unit_Cube_K[2],2));
    // I, J, K vector of normalized vector
    cv::Vec3f Cube_I_normalized = Unit_Cube_I/Cube_I_value;
    cv::Vec3f Cube_J_normalized = Unit_Cube_J/Cube_J_value;
    cv::Vec3f Cube_K_normalized = Unit_Cube_K/Cube_K_value;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CubeInCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int Grid_cnt = 0;
    for(int i=1; i <= Grid_size[0]; i++)
    {
        for(int j=1; j <= Grid_size[1]; j++)
        {
            for(int k=1; k <= Grid_size[2]; k++)
            {
                if(occ_Grid[Grid_cnt]==1)
                {
                    pcl::PointXYZRGB temp_cloud;
                    cv::Vec3f temp_cloud_vec3f = cv::Vec3f(BB_vertex1.x, BB_vertex1.y, BB_vertex1.z);
                    for(int l = 0; l < 3; l++)
                    {
                        temp_cloud_vec3f[l] -= (i-1)*Unit_Cube_L*Cube_I_normalized[l];
                        temp_cloud_vec3f[l] -= (j-1)*Unit_Cube_L*Cube_J_normalized[l];
                        temp_cloud_vec3f[l] -= (k-1)*Unit_Cube_L*Cube_K_normalized[l];
                        temp_cloud_vec3f[l] -= 0.5*Unit_Cube_L*Cube_I_normalized[l];
                        temp_cloud_vec3f[l] -= 0.5*Unit_Cube_L*Cube_J_normalized[l];
                        temp_cloud_vec3f[l] -= 0.5*Unit_Cube_L*Cube_K_normalized[l];
                    }
                    temp_cloud.x = temp_cloud_vec3f[0];
                    temp_cloud.y = temp_cloud_vec3f[1];
                    temp_cloud.z = temp_cloud_vec3f[2];
                    temp_cloud.r = 178;
                    temp_cloud.g = 0; 
                    temp_cloud.b = 255; 
                    CubeInCloud->push_back(temp_cloud);
                }
                Grid_cnt++;
            }
        }
    }
    for (int i=0; i<CubeInCloud->size(); i++)
        Block_center_temp.push_back(CubeInCloud->points[i]);
}

void ObjectPose::CheckOccGridWithKnownShape(std::vector<int> Grid_size, std::vector<int> occ_grid)
{
    cout << color_string << endl;
    RNG rng(12345);
    int min_x =std::numeric_limits<int>::max();
    int min_y =std::numeric_limits<int>::max();
    int max_x = 0; 
    int max_y = 0; 
    for ( int j = 0; j < 4; j++ )
    {
        if(max_x < _RectPoints.at(j).x)
            max_x = _RectPoints.at(j).x;
        if(min_x > _RectPoints.at(j).x)
            min_x = _RectPoints.at(j).x;
        if(max_y < _RectPoints.at(j).y)
            max_y = _RectPoints.at(j).y;
        if(min_y > _RectPoints.at(j).y)
            min_y = _RectPoints.at(j).y;
    }
    if(Debug_Object==1)
        cout << "--------------------" << endl;

    if(color_string=="red")
	{
        if((Grid_size[0]==2 & Grid_size[1]==2 & Grid_size[2]==1)
           || (Grid_size[0]==1 & Grid_size[1]==2 & Grid_size[2]==2) || (Grid_size[0]==2 & Grid_size[1]==1 & Grid_size[2]==2))
        {
            int tot_occ_grid_cnt=0;
            for(int i = 0; i < 4; i++)
                tot_occ_grid_cnt+=occ_grid[i];

            Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            if(tot_occ_grid_cnt==3)
            {
                if(Debug_Object==1)
                {
                    cout << "red block detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
                red_Grid = Grid_size;
                red_occ_Grid = occ_grid;
                BB_info_red.clear();
                for(int k = 0; k < BBinfo_temp.size(); k++)
                {
                    pcl::PointXYZRGB point_temp1 = std::get<0>(BBinfo_temp[k]);
                    pcl::PointXYZRGB point_temp2 = std::get<1>(BBinfo_temp[k]);
                    BB_info_red.push_back(Point3D{point_temp1.x, point_temp1.y, point_temp1.z});
                    BB_info_red.push_back(Point3D{point_temp2.x, point_temp2.y, point_temp2.z});
                }
                Block_center_temp.clear();
                Block_center_red.clear();
                GenerateRealSyntheticCloud(red_Grid, red_occ_Grid, BBinfo_temp);
                for(int l = 0; l < Block_center_temp.size(); l++)
                {
                    pcl::PointXYZRGB point_temp3 = Block_center_temp[l];
                    Block_center_red.push_back(Point3D{point_temp3.x, point_temp3.y, point_temp3.z});
                }
                Red_RectPoints = _RectPoints;
                for(int y = 0; y < height; y++)
                {
                    for(int x = 0; x < width; x++)
                    {
                        if(_Projected_image.at<uchar>(y,x) == 255 && x < max_x && x > min_x && y < max_y && y > min_y)
                            Total_Projected_image.at<Vec3b>(y, x)[2] = 255;
                    }
                }
                for ( int j = 0; j < 4; j++ )
                    line(Total_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);

            }
            else
            {
                if(Debug_Object==1)
                {
                    cout << "Red block mis-detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
            }
        }
        else
        {
            if(Debug_Object==1)
            {
                cout << "Red block mis-detected" << endl;
                cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                cout << "Occ grid: " ;
                for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                    cout << occ_grid[i] << " " ;
                cout << endl;
            }
        }
    }

    if(color_string=="yellow")
    {
       cout << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
       if((Grid_size[0]==3 & Grid_size[1]==2 & Grid_size[2]==1) || (Grid_size[0]==2 & Grid_size[1]==3 & Grid_size[2]==1)
           || (Grid_size[0]==1 & Grid_size[1]==3 & Grid_size[2]==2) || (Grid_size[0]==3 & Grid_size[1]==1 & Grid_size[2]==2))
       {
           int tot_occ_grid_cnt=0;
           for(int i = 0; i < 6; i++)
               tot_occ_grid_cnt+=occ_grid[i];

           Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
           if(tot_occ_grid_cnt==4)
           {
                if(Debug_Object==1)
                {
                    cout << "yellow block detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
                yellow_Grid = Grid_size;
                yellow_occ_Grid = occ_grid;
                BB_info_yellow.clear();
                for(int k = 0; k < BBinfo_temp.size(); k++)
                {
                    pcl::PointXYZRGB point_temp1 = std::get<0>(BBinfo_temp[k]);
                    pcl::PointXYZRGB point_temp2 = std::get<1>(BBinfo_temp[k]);
                    BB_info_yellow.push_back(Point3D{point_temp1.x, point_temp1.y, point_temp1.z});
                    BB_info_yellow.push_back(Point3D{point_temp2.x, point_temp2.y, point_temp2.z});
                }
                Block_center_temp.clear();
                Block_center_yellow.clear();
                GenerateRealSyntheticCloud(yellow_Grid, yellow_occ_Grid, BBinfo_temp);
                for(int l = 0; l < Block_center_temp.size(); l++)
                {
                    pcl::PointXYZRGB point_temp3 = Block_center_temp[l];
                    Block_center_red.push_back(Point3D{point_temp3.x, point_temp3.y, point_temp3.z});
                }
                Red_RectPoints = _RectPoints;
                for(int y = 0; y < height; y++)
                {
                    for(int x = 0; x < width; x++)
                    {
                        if(_Projected_image.at<uchar>(y,x) == 255 && x < max_x && x > min_x && y < max_y && y > min_y)
                        {
                            Total_Projected_image.at<Vec3b>(y, x)[0] = 0;
                            Total_Projected_image.at<Vec3b>(y, x)[1] = 255;
                            Total_Projected_image.at<Vec3b>(y, x)[2] = 255;
                        }
                    }
                }
                for ( int j = 0; j < 4; j++ )
                    line(Total_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);

               for(int l = 0; l < Block_center_temp.size(); l++)
               {
                    pcl::PointXYZRGB point_temp3 = Block_center_temp[l];
                    Block_center_yellow.push_back(Point3D{point_temp3.x, point_temp3.y, point_temp3.z});
               }
           }
           else
           {
                if(Debug_Object==1)
                {
                    cout << "yellow block mis-detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
           }
       }
       else
       {
            if(Debug_Object==1)
            {
                cout << "yellow block mis-detected" << endl;
                cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                cout << "Occ grid: " ;
                for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                    cout << occ_grid[i] << " " ;
                cout << endl;
            }
       }
    }

    if(color_string=="blue")
    {
        if((Grid_size[0]==2 & Grid_size[1]==2 & Grid_size[2]==2))
        {
            int tot_occ_grid_cnt=0;
            int second_floor_cnt=0;
            for(int i = 0; i < 8; i++)
            {
               tot_occ_grid_cnt+=occ_grid[i];
               if(i%2==1)
                   second_floor_cnt += occ_grid[i];
            }
            Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            if(tot_occ_grid_cnt==4 && second_floor_cnt==1)
            {
                blue_Grid = Grid_size;
                blue_occ_Grid = occ_grid;
                BB_info_blue.clear();
                if(Debug_Object==1)
                {
                    cout << "Blue block detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }

                for(int k = 0; k < BBinfo_temp.size(); k++)
                {
                    pcl::PointXYZRGB point_temp1 = std::get<0>(BBinfo_temp[k]);
                    pcl::PointXYZRGB point_temp2 = std::get<1>(BBinfo_temp[k]);
                    BB_info_blue.push_back(Point3D{point_temp1.x, point_temp1.y, point_temp1.z});
                    BB_info_blue.push_back(Point3D{point_temp2.x, point_temp2.y, point_temp2.z});
                }
                Block_center_temp.clear();
                Block_center_blue.clear();
                GenerateRealSyntheticCloud(blue_Grid, blue_occ_Grid, BBinfo_temp);
                for(int y = 0; y < height; y++)
                {
                    for(int x = 0; x < width; x++)
                    {
                        if(_Projected_image.at<uchar>(y,x) == 255 && x < max_x && x > min_x && y < max_y && y > min_y)
                        {
                            Total_Projected_image.at<Vec3b>(y, x)[0] = 255;
                        }
                    }
                }
                for ( int j = 0; j < 4; j++ )
                    line(Total_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);
                for(int l = 0; l < Block_center_temp.size(); l++)
                {
                    pcl::PointXYZRGB point_temp3 = Block_center_temp[l];
                    Block_center_blue.push_back(Point3D{point_temp3.x, point_temp3.y, point_temp3.z});
                }
            }
            else
            {
                if(Debug_Object==1)
                {
                    cout << "Blue block mis-detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
            }
        }
        else
        {
            if(Debug_Object==1)
            {
                cout << "Blue block mis-detected" << endl;
                cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                cout << "Occ grid: " ;
                for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                    cout << occ_grid[i] << " " ;
                cout << endl;
            }
        }
    }

    if(color_string=="brown")
    {
        if((Grid_size[0]==2 & Grid_size[1]==2 & Grid_size[2]==2))
        {
            int tot_occ_grid_cnt=0;
            int second_floor_cnt=0;
            for(int i = 0; i < 8; i++)
            {
                tot_occ_grid_cnt+=occ_grid[i];
                if(i%2==1)
                   second_floor_cnt += occ_grid[i];
            }
            Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            if(tot_occ_grid_cnt==4 && second_floor_cnt==1)
            {
                brown_Grid = Grid_size;
                brown_occ_Grid = occ_grid;
                BB_info_brown.clear();
                for(int k = 0; k < BBinfo_temp.size(); k++)
                {
                    pcl::PointXYZRGB point_temp1 = std::get<0>(BBinfo_temp[k]);
                    pcl::PointXYZRGB point_temp2 = std::get<1>(BBinfo_temp[k]);
                    BB_info_brown.push_back(Point3D{point_temp1.x, point_temp1.y, point_temp1.z});
                    BB_info_brown.push_back(Point3D{point_temp2.x, point_temp2.y, point_temp2.z});
                }
                Block_center_brown.clear();
                Block_center_temp.clear();
                GenerateRealSyntheticCloud(brown_Grid, brown_occ_Grid, BBinfo_temp);
                for(int y = 0; y < height; y++)
                {
                    for(int x = 0; x < width; x++)
                    {
                        if(_Projected_image.at<uchar>(y,x) == 255 && x < max_x && x > min_x && y < max_y && y > min_y)
                        {
                            Total_Projected_image.at<Vec3b>(y, x)[0] = 51;
                            Total_Projected_image.at<Vec3b>(y, x)[1] = 60;
                            Total_Projected_image.at<Vec3b>(y, x)[2] = 72;
                        }
                    }
                }
                for ( int j = 0; j < 4; j++ )
                    line(Total_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);
                for(int l = 0; l < Block_center_temp.size(); l++)
                {
                    pcl::PointXYZRGB point_temp3 = Block_center_temp[l];
                    Block_center_brown.push_back(Point3D{point_temp3.x, point_temp3.y, point_temp3.z});
                }
           }
           else
           {
                cout << "brown block mis detected, Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2];
                cout << " total occ grid: " << tot_occ_grid_cnt << "  second floor cnt: " << second_floor_cnt << endl;
           }
        }
        else
           cout << "brown block mis detected" << endl;
    }

    if(color_string=="green")
    {
        cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
        if((Grid_size[0]==2 & Grid_size[1]==3 & Grid_size[2]==1) || (Grid_size[0]==3 & Grid_size[1]==2 & Grid_size[2]==1) ||
          (Grid_size[0]==1 & Grid_size[1]==3 & Grid_size[2]==2) || (Grid_size[0]==3 & Grid_size[1]==1 & Grid_size[2]==2))
        {
            int tot_occ_grid_cnt=0;
            for(int i = 0; i < 6; i++)
               tot_occ_grid_cnt+=occ_grid[i];
            Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256));
            if(tot_occ_grid_cnt==4)
            {
                green_Grid = Grid_size;
                green_occ_Grid = occ_grid;
                BB_info_green.clear();
                if(Debug_Object==1)
                {
                    cout << "Green block detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
                for(int k = 0; k < BBinfo_temp.size(); k++)
                {
                    pcl::PointXYZRGB point_temp1 = std::get<0>(BBinfo_temp[k]);
                    pcl::PointXYZRGB point_temp2 = std::get<1>(BBinfo_temp[k]);
                    BB_info_green.push_back(Point3D{point_temp1.x, point_temp1.y, point_temp1.z});
                    BB_info_green.push_back(Point3D{point_temp2.x, point_temp2.y, point_temp2.z});
                }
                Block_center_temp.clear();
                Block_center_green.clear();
                GenerateRealSyntheticCloud(green_Grid, green_occ_Grid, BBinfo_temp);
                for(int y = 0; y < height; y++)
                {
                    for(int x = 0; x < width; x++)
                    {
                        if(_Projected_image.at<uchar>(y,x) == 255 && x < max_x && x > min_x && y < max_y && y > min_y)
                        {
                            Total_Projected_image.at<Vec3b>(y, x)[1] = 255;
                        }
                    }
                }
                for ( int j = 0; j < 4; j++ )
                    line(Total_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);

                for(int l = 0; l < Block_center_temp.size(); l++)
                {
                    pcl::PointXYZRGB point_temp3 = Block_center_temp[l];
                    Block_center_green.push_back(Point3D{point_temp3.x, point_temp3.y, point_temp3.z});
                }
            }
            else
            {
                if(Debug_Object==1)
                {
                    cout << "Green block mis-detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
            }
        }
        else
        {
            if(Debug_Object==1)
            {
                cout << "Green block mis-detected" << endl;
                cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                cout << "Occ grid: " ;
                for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                    cout << occ_grid[i] << " " ;
                cout << endl;
            }
        }
    }


    if(color_string=="orange")
    {
        if((Grid_size[0]==1 & Grid_size[1]==3 & Grid_size[2]==2) || (Grid_size[0]==3 & Grid_size[1]==1 & Grid_size[2]==2) ||
        (Grid_size[0]==2 & Grid_size[1]==3 & Grid_size[2]==1) || (Grid_size[0]==3 & Grid_size[1]==2 & Grid_size[2]==1) || 
        (Grid_size[0]==1 & Grid_size[1]==2 & Grid_size[2]==3) || (Grid_size[0]==2 & Grid_size[1]==1 & Grid_size[2]==3))
        {
            int tot_occ_grid_cnt=0;
            for(int i = 0; i < 6; i++)
                tot_occ_grid_cnt+=occ_grid[i];

            Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            if(tot_occ_grid_cnt==4)
            {
                if(Debug_Object==1)
                {
                    cout << "Orange block detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
                orange_Grid = Grid_size;
                orange_occ_Grid = occ_grid;
                BB_info_orange.clear();
                for(int k = 0; k < BBinfo_temp.size(); k++)
                {
                    pcl::PointXYZRGB point_temp1 = std::get<0>(BBinfo_temp[k]);
                    pcl::PointXYZRGB point_temp2 = std::get<1>(BBinfo_temp[k]);
                    BB_info_orange.push_back(Point3D{point_temp1.x, point_temp1.y, point_temp1.z});
                    BB_info_orange.push_back(Point3D{point_temp2.x, point_temp2.y, point_temp2.z});
                }
                Block_center_temp.clear();
                Block_center_orange.clear();
                GenerateRealSyntheticCloud(orange_Grid, orange_occ_Grid, BBinfo_temp);
                for(int y = 0; y < height; y++)
                {
                    for(int x = 0; x < width; x++)
                    {
                        if(_Projected_image.at<uchar>(y,x) == 255 && x < max_x && x > min_x && y < max_y && y > min_y)
                        {
                            Total_Projected_image.at<Vec3b>(y, x)[0] = 0;
                            Total_Projected_image.at<Vec3b>(y, x)[1] = 165;
                            Total_Projected_image.at<Vec3b>(y, x)[2] = 255;
                        }
                    }
                }
                for(int j = 0; j < 4; j++)
                    line(Total_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);

                for(int l = 0; l < Block_center_temp.size(); l++)
                {
                    pcl::PointXYZRGB point_temp3 = Block_center_temp[l];
                    Block_center_orange.push_back(Point3D{point_temp3.x, point_temp3.y, point_temp3.z});
                }
            }
            else
            {
                if(Debug_Object==1)
                {
                    cout << "Orange block mis-detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
            }

        }
        else
        {
            if(Debug_Object==1)
            {
                cout << "Orange block mis-detected" << endl;
                cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                cout << "Occ grid: " ;
                for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                    cout << occ_grid[i] << " " ;
                cout << endl;
            }
        }
    }


    if(color_string=="Indigo")
    {
        if((Grid_size[0]==2 & Grid_size[1]==2 & Grid_size[2]==2))
        {
            int tot_occ_grid_cnt=0;
            int second_floor_cnt=0;
            for(int i = 0; i < 8; i++)
            {
                tot_occ_grid_cnt+=occ_grid[i];
                if(i%2==0)
                    second_floor_cnt += occ_grid[i];
            }
            Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            if(tot_occ_grid_cnt==4)
            {
                if(Debug_Object==1)
                {
                    cout << "Indigo block detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
                Indigo_Grid = Grid_size;
                Indigo_occ_Grid = occ_grid;
                BB_info_Indigo.clear();
                for(int k = 0; k < BBinfo_temp.size(); k++)
                {
                    pcl::PointXYZRGB point_temp1 = std::get<0>(BBinfo_temp[k]);
                    pcl::PointXYZRGB point_temp2 = std::get<1>(BBinfo_temp[k]);
                    BB_info_Indigo.push_back(Point3D{point_temp1.x, point_temp1.y, point_temp1.z});
                    BB_info_Indigo.push_back(Point3D{point_temp2.x, point_temp2.y, point_temp2.z});
                }
                Block_center_temp.clear();
                Block_center_Indigo.clear();
                GenerateRealSyntheticCloud(Indigo_Grid, Indigo_occ_Grid, BBinfo_temp);
                for(int y = 0; y < height; y++)
                {
                    for(int x = 0; x < width; x++)
                    {
                        if(_Projected_image.at<uchar>(y,x) == 255 && x < max_x && x > min_x && y < max_y && y > min_y)
                        {
                            Total_Projected_image.at<Vec3b>(y, x)[0] = 100;
                            Total_Projected_image.at<Vec3b>(y, x)[1] = 50;
                            Total_Projected_image.at<Vec3b>(y, x)[2] = 40;
                        }
                    }
                }
                for(int j = 0; j < 4; j++)
                    line(Total_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);

                for(int l = 0; l < Block_center_temp.size(); l++)
                {
                    pcl::PointXYZRGB point_temp3 = Block_center_temp[l];
                    Block_center_Indigo.push_back(Point3D{point_temp3.x, point_temp3.y, point_temp3.z});
                }
            }
            else 
            {
                if(Debug_Object==1)
                {
                    cout << "Indigo block mis-detected" << endl;
                    cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                    cout << "Occ grid: " ;
                    for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                        cout << occ_grid[i] << " " ;
                    cout << endl;
                }
            }
        }
        else
        {
            if(Debug_Object==1)
            {
                cout << "Indigo block mis-detected" << endl;
                cout << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
                cout << "Occ grid: " ;
                for(int i=0; i<Grid_size[0]*Grid_size[1]*Grid_size[2]; i++)
                    cout << occ_grid[i] << " " ;
                cout << endl;
            }
        }
    }
}




void ObjectPose::CloudView(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector) 
{
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(in_cloud);
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    //blocks until the cloud is actually rendered
    viewer.addPointCloud(in_cloud, "in_cloud", 0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "in_cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // viewer.addCube(BB.X_min, BB.X_max, BB.Y_min, BB.Y_max, BB.Z_min, BB.Z_max,1.0, 0, 0, "in_cloud", 0);
    // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "in_cloud", 0);
    for(int i = 0; i < 4; i++)
    {
        viewer.addLine<pcl::PointXYZRGB, pcl::PointXYZRGB> (std::get<0>(pos_vector[i]), std::get<1>(pos_vector[i]), 0.0, 1.0, 0.5, "line" + std::to_string(i));
        viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "line" + std::to_string(i));
    }

    for(int i = 0; i < 4; i++)
    {
        if(i<3)
        {
            viewer.addLine<pcl::PointXYZRGB, pcl::PointXYZRGB> (std::get<0>(pos_vector[i]), std::get<0>(pos_vector[i+1]), 0.0, 1.0, 0.5, "Line1" + std::to_string(i));
            viewer.addLine<pcl::PointXYZRGB, pcl::PointXYZRGB> (std::get<1>(pos_vector[i]), std::get<1>(pos_vector[i+1]), 0.0, 1.0, 0.5, "Line2" + std::to_string(i));
        }
        else 
        {
            viewer.addLine<pcl::PointXYZRGB, pcl::PointXYZRGB> (std::get<0>(pos_vector[i]), std::get<0>(pos_vector[0]), 0.0, 1.0, 0.5, "Line1" + std::to_string(i));
            viewer.addLine<pcl::PointXYZRGB, pcl::PointXYZRGB> (std::get<1>(pos_vector[i]), std::get<1>(pos_vector[0]), 0.0, 1.0, 0.5, "Line2" + std::to_string(i));
        }
        viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "Line1" + std::to_string(i));
        viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "Line2" + std::to_string(i));
    }


    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
    
    //This will only get called once
    //This will get called once per visualization iteration
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}

void ObjectPose::ClearVariable()
{
    red_cloud.clear();
    yellow_cloud.clear();
    green_cloud.clear();
    blue_cloud.clear();
    brown_cloud.clear();
    orange_cloud.clear();
    Indigo_cloud.clear();
    BBinfo_temp.clear();
}
