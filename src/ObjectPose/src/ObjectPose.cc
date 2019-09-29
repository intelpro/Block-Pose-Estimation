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
                       float _cy, float _unit_length, float _dist_thresh, Plane::DominantPlane* plane)
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
    unit_length = _unit_length;
    dist_thresh = _dist_thresh;
    best_plane = plane->cur_best_plane;
    box_flag=0;
    color_string="red";
    std::vector<int> red_Grid(3);
    std::vector<int> yellow_Grid(3);
    std::vector<int> green_Grid(3);
    std::vector<int> blue_Grid(3);
    std::vector<int> brown_Grid(3);
    std::vector<int> orange_Grid(3);
    std::vector<int> purple_Grid(3);
    std::vector<int> red_occ_Grid(3);
    std::vector<int> yellow_occ_Grid(3);
    std::vector<int> green_occ_Grid(3);
    std::vector<int> blue_occ_Grid(3);
    std::vector<int> brown_occ_Grid(3);
    std::vector<int> orange_occ_Grid(3);
    std::vector<int> purple_occ_Grid(3);
}

void ObjectPose::Accumulate_PointCloud(cv::Mat &pcd_outlier, std::vector<cv::Mat> &Mask)
{
    //red, yellow, green, blue, brown, orange, purple
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

        if(i==6) // purple
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
                        point.r = 102; 
                        point.g = 51; 
                        point.b = 153;
                        if(point.x!=0)
                        {
                            purple_cloud.push_back(point);
                        }
                    }
                }
            }
        }

    }

    Accum_idx++;
    if(Accum_idx >= Accum_iter)
    {
        ProjectToDominantPlane(red_cloud, "red");
        ProjectToDominantPlane(yellow_cloud, "yellow");
        // ProjectToDominantPlane(green_cloud, "green");
        ProjectToDominantPlane(blue_cloud, "blue");
        ProjectToDominantPlane(brown_cloud, "brown");
        ProjectToDominantPlane(orange_cloud, "orange");
        ProjectToDominantPlane(purple_cloud, "purple");
        Accum_idx = 0; 
    }

}



void ObjectPose::ProjectToDominantPlane(pcl::PointCloud<pcl::PointXYZRGB> in_cloud, std::string _color_string)
{
    color_string=_color_string;
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
        else if(color_string=="purple")
        {
            ProjectedPoint.r = 102; 
            ProjectedPoint.g = 51;
            ProjectedPoint.b = 153;
        }
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
    projected_cloud = pcl_projected_cloud;
    ProjectedCloudToImagePlane();
}

void ObjectPose::ProjectedCloudToImagePlane()
{
    cv::Mat projected_image = cv::Mat::zeros(height, width, CV_8UC3);
    std::vector<pair<int, int>> pos_vector; 
    for (int i =0; i < projected_cloud->size(); i++)
    {
        pcl::PointXYZRGB temp_cloud; 
        temp_cloud = projected_cloud->points[i];
        float X = temp_cloud.x;
        float Y = temp_cloud.y; 
        float Z = temp_cloud.z; 
        int x = cy + fy*Y/Z;
        int y = cx + fx*X/Z;
        pos_vector.push_back(make_pair(x,y));
        if(color_string=="red")
            projected_image.at<Vec3b>(x, y)[2] = 255;
        else if(color_string=="yellow")
        {
            projected_image.at<Vec3b>(x, y)[1] = 255;
            projected_image.at<Vec3b>(x, y)[2] = 255;
        }
        else if(color_string=="green")
            projected_image.at<Vec3b>(x, y)[1] = 255;
        else if(color_string=="blue")
            projected_image.at<Vec3b>(x, y)[0] = 255;
        else if(color_string=="brown")
        {
            projected_image.at<Vec3b>(x,y)[0] = 50; 
            projected_image.at<Vec3b>(x,y)[1] = 60; 
            projected_image.at<Vec3b>(x,y)[2] = 72;
        }
        else if(color_string=="orange")
        {
            projected_image.at<Vec3b>(x,y)[1] = 165; 
            projected_image.at<Vec3b>(x,y)[2] = 255;
        }
        else if(color_string=="purple")
        {
            projected_image.at<Vec3b>(x,y)[0] = 153; 
            projected_image.at<Vec3b>(x,y)[1] = 51; 
            projected_image.at<Vec3b>(x,y)[2] = 102;
        }
    }

    _Projected_image = projected_image;
    fitRectangle(projected_image);

    // cv::imwrite("projected_image.png", projected_image);
    /*
    int size_vector = pos_vector.size();
    int x_temp1, y_temp1, x_temp2, y_temp2;
    float slope; 
    int max_x, min_x, min_y, max_y; 
    for(int i = 0; i < 10; i++)
    {
        int RandIndex1 = rand() % size_vector; 
        int RandIndex2 = rand() % size_vector; 
        x_temp1 = std::get<0>(pos_vector[RandIndex1]);
        y_temp1 = std::get<1>(pos_vector[RandIndex1]);
        x_temp2 = std::get<0>(pos_vector[RandIndex2]); 
        y_temp2 = std::get<1>(pos_vector[RandIndex2]);
        if (y_temp1 != y_temp2)
        {
            slope = (x_temp2 - x_temp1) / (y_temp2 - y_temp1);
            max_x = std::max(x_temp1, x_temp2);
            max_y = std::max(y_temp1, y_temp2);
            min_x = std::min(x_temp1, x_temp2);
            min_y = std::min(y_temp1, y_temp2);
            int length = max_y - min_y;
            if(std::abs(slope) > 0.1) 
            {
                for(float y = min_y; y < min_y + length;  y++)
                {
                    float x = slope*(y-y_temp1) + x_temp1;
                    if(color_string=="green")
                        projected_image.at<Vec3b>(x,y)[1] = 255; 
                    else if(color_string=="red")
                        projected_image.at<Vec3b>(x,y)[2] = 255; 
                    else if(color_string=="blue")
                        projected_image.at<Vec3b>(x,y)[0] = 255; 
                    else if(color_string=="yellow")
                    {
                        projected_image.at<Vec3b>(x, y)[1] = 255;
                        projected_image.at<Vec3b>(x, y)[2] = 255;
                    }
                    else if(color_string=="orange")
                    {
                        projected_image.at<Vec3b>(x,y)[1] = 165; 
                        projected_image.at<Vec3b>(x,y)[2] = 255;
                    }
                }
            }
        }
    }
    // cv::imshow("projected_image", projected_image);
    */
}


void ObjectPose::fitRectangle(cv::Mat projected_image)
{
    int thresh = 0.1;
    Mat gray;
    cv::cvtColor(projected_image, gray, CV_BGR2GRAY);

    std::vector<Point> RectPoints = std::vector<Point>(4);
    Mat canny_output;
    Canny(projected_image, canny_output, thresh, thresh*10, 3 );
    vector<vector<Point> > contours;
    findContours( canny_output, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        minRect[i] = minAreaRect( contours[i] );
    }
    
    double max_size = 1000;
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
    BackProjectToDominatPlane(RectPoints);

    /*
    double prev_dist_rect_1 = sqrt(pow(prev_RectPoints[0].x - prev_RectPoints[1].x, 2) + pow(prev_RectPoints[0].y - prev_RectPoints[1].y, 2));
    double prev_dist_rect_2 = sqrt(pow(prev_RectPoints[1].x - prev_RectPoints[2].x, 2) + pow(prev_RectPoints[1].y - prev_RectPoints[2].y, 2));
    double prev_dist_rect = prev_dist_rect_1 * prev_dist_rect_2;
    Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );

    if(box_flag==0)
    {
        prev_RectPoints = RectPoints;
        box_flag = 1;
    }

    float mean_x = 0; 
    float mean_y = 0;
    float sum_x = 0; 
    float sum_y = 0; 
    int cnt = 0; 
    for (int x = 0; x < projected_image.rows; x++)
    {
        for ( int y = 0; y < projected_image.cols; y++ )
        {
            if(projected_image.at<Vec3b>(y,x)[2]==255)
            {
                sum_x += x; 
                sum_y += y; 
                cnt++;
            }

        }
    }

    mean_x = sum_x /cnt; 
    mean_y = sum_y /cnt;
    float centroid_x = 0;
    float centroid_y = 0;
    for ( int j = 0; j < 4; j++ )
    {   
        centroid_x += float(RectPoints[j].x);
        centroid_y += float(RectPoints[j].y);
    } 
    float mean_centorid_x = centroid_x/4;
    float mean_centorid_y = centroid_y/4;

    cout << "diff x: " << std::abs(mean_centorid_x -mean_x) << "  diff y: " << std::abs(mean_centorid_y - mean_y) << endl;

    if((max_size > prev_dist_rect || std::abs(max_size - prev_dist_rect) < 1000) && box_flag==1 && std::abs(mean_centorid_x - mean_x) < 50 && std::abs(mean_centorid_y - mean_y) < 50)
    {
        prev_RectPoints = RectPoints;
    }
    */
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
    else if(color_string=="purple")
        max_length = FindBlockHeight(purple_cloud, a, b, c, d);

    std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector; 
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
        pos_vector.push_back(make_pair(temp_cloud,temp_cloud_upper));
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_for_viewer(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(color_string=="red")
    {
        // cout << "red height: " << max_length << endl;
        // cout << "red point cloud size: " << red_cloud.size() << endl;
        pcl::copyPointCloud(red_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
        FindOccGrid(pos_vector, max_length);
        // cout << color_string << " Grid: " << red_Grid[0] << " " << red_Grid[1] << " " << red_Grid[2] << endl;
        red_cloud.clear();
    }

    else if(color_string=="yellow")
    {
        // cout << "yellow height: " << max_length << endl;
        // cout << "yellow point cloud size: " << yellow_cloud.size() << endl;
        pcl::copyPointCloud(yellow_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
        FindOccGrid(pos_vector, max_length);
        // cout << color_string << " Grid: " << yellow_Grid[0] << " " << yellow_Grid[1] << " " << yellow_Grid[2] << endl;
        yellow_cloud.clear();
    }

    else if(color_string=="green")
    {
        // cout << "green height: " << max_length << endl;
        pcl::copyPointCloud(green_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
    }

    else if(color_string=="blue")
    {
        // cout << "blue height: " << max_length << endl;
        pcl::copyPointCloud(blue_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
        FindOccGrid(pos_vector, max_length);
        // cout << color_string << " Grid" << blue_Grid[0] << " " << blue_Grid[1] << " " << blue_Grid[2] << endl;
        blue_cloud.clear();
    }

    else if(color_string=="brown")
    {
        // cout << "brown height: " << max_length << endl;
        pcl::copyPointCloud(brown_cloud, *Cloud_for_viewer);
        FindOccGrid(pos_vector, max_length);
        // cout << color_string << " Grid" << brown_Grid[0] << " " << brown_Grid[1] << " " << brown_Grid[2] << endl;
        // CloudView(Cloud_for_viewer, pos_vector);
        brown_cloud.clear();
    }

    else if(color_string=="orange")
    {
        // cout << "orange height: " << max_length << endl;
        pcl::copyPointCloud(orange_cloud, *Cloud_for_viewer);
        FindOccGrid(pos_vector, max_length);
        // CloudView(Cloud_for_viewer, pos_vector);
        // cout << color_string << " Grid" << orange_Grid[0] << " " << orange_Grid[1] << " " << orange_Grid[2] << endl;
        orange_cloud.clear();
    }

    else if(color_string=="purple")
    {
        // cout << "purple height: " << max_length << endl;
        pcl::copyPointCloud(purple_cloud, *Cloud_for_viewer);
        FindOccGrid(pos_vector, max_length);
        // cout << color_string << " Grid" << purple_Grid[0] << " " << purple_Grid[1] << " " << purple_Grid[2] << endl;
        // CloudView(Cloud_for_viewer, pos_vector);
        purple_cloud.clear();
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

    cout << "max height: " << max_height << endl;
    // Discretize max height
    if(max_height > unit_length-dist_thresh & max_height < unit_length+dist_thresh)
        max_height = unit_length + 0.003;
    else if(max_height > 2*unit_length - dist_thresh & max_height < 2*unit_length + dist_thresh)
        max_height = 2*unit_length + 0.003;
    else if(max_height > 3*unit_length - dist_thresh & max_height < 3*unit_length + dist_thresh)
        max_height = 3*unit_length + 0.003;
    else if(max_height > 4*unit_length - dist_thresh & max_height < 4*unit_length + dist_thresh)
        max_height = -1; 
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
    cout << "mean height: " << dist_mean << endl;
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

    if(dist1 > unit_length-dist_thresh & dist1 < unit_length+dist_thresh)
        Grid_size[0] = 1;
    else if(dist1 > 2*unit_length-dist_thresh & dist1 < 2*unit_length+dist_thresh)
        Grid_size[0] = 2;
    else if(dist1 > 3*unit_length-dist_thresh & dist1 < 3*unit_length+dist_thresh)
        Grid_size[0] = 3;
    else 
        Grid_size[0] = -1; 

    if(dist2 > unit_length-dist_thresh & dist2 < unit_length+dist_thresh)
        Grid_size[1] = 1;
    else if(dist2 > 2*unit_length-dist_thresh & dist2 < 2*unit_length+dist_thresh)
        Grid_size[1] = 2;
    else if(dist2 > 3*unit_length-dist_thresh & dist2 < 3*unit_length+dist_thresh)
        Grid_size[1] = 3;
    else 
        Grid_size[1] = -1; 

    if(max_height > unit_length - dist_thresh & max_height < unit_length + dist_thresh)
        Grid_size[2] = 1;
    else if(max_height > 2*unit_length - dist_thresh & max_height < 2*unit_length + dist_thresh)
        Grid_size[2] = 2;
    else if(max_height > 3*unit_length - dist_thresh & max_height < 3*unit_length + dist_thresh)
        Grid_size[2] = 3;
    else 
        Grid_size[2] = -1; 

    int cnt_tot = 0; 
    if(color_string=="red")
        MeasureOccupany(pos_vector, Grid_size, red_cloud);
    else if(color_string=="yellow")
        MeasureOccupany(pos_vector, Grid_size, yellow_cloud);
    else if(color_string=="green")
        green_Grid=Grid_size;
    else if(color_string=="blue")
        MeasureOccupany(pos_vector, Grid_size, blue_cloud);
    else if(color_string=="brown")
        MeasureOccupany(pos_vector, Grid_size, brown_cloud);
    else if(color_string=="orange")
        MeasureOccupany(pos_vector, Grid_size, orange_cloud);
    else if(color_string=="purple")
        MeasureOccupany(pos_vector, Grid_size, purple_cloud);
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
    pcl::PointCloud<pcl::PointXYZRGB> CubeInCloud;
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
                        CubeInCloud.push_back(temp_cloud);
                    }
                }

                // cout << "i: " << i << "j: " << j << "k: " << k << endl;
                // cout << "cnt : " <<  cnt_temp << endl;
                // float mean_height = FindBlockMeanHeight(CubeInCloud, a, b, c, d);
                Grid_pcd_cnt[Grid_cnt] = cnt_temp;
                CubeInCloud.clear();
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

    cout << "-------------------------" << endl;
    std::vector<int> occ_grid(Grid_tot_size);
    for(int i = 0; i<Grid_tot_size; i++)
    {
        cout << "cnt : " <<  Grid_pcd_cnt[i] << endl;
        if(Grid_pcd_cnt[i] > ref_cloud.size()/(3*Grid_tot_size))
            occ_grid[i] = 1; 
        else 
            occ_grid[i] = 0; 

        if(i!=0 & Grid_size[2]!=1 & i%Grid_size[2]==Grid_size[2]-1 & occ_grid[i]==1)
            occ_grid[i-1] = 1;
    }

    // cout << "occ grid: " ;
    // for(int i=0; i<Grid_tot_size; i++)
    //    cout << occ_grid[i] << " " ;
    cout << "-------------------------" << endl;
    CheckOccGridWithKnownShape(Grid_size, occ_grid);
}

void ObjectPose::CheckOccGridWithKnownShape(std::vector<int> Grid_size, std::vector<int> occ_grid)
{
    RNG rng(12345);
    if(color_string=="red")
    {
        if((Grid_size[0]==2 & Grid_size[1]==2 & Grid_size[2]==1)
           || (Grid_size[0]==1 & Grid_size[1]==2 & Grid_size[2]==2) || (Grid_size[0]==2 & Grid_size[1]==1 & Grid_size[2]==1))
        {
            int tot_occ_grid_cnt=0;
            for(int i = 0; i < 4; i++)
                tot_occ_grid_cnt+=occ_grid[i];

            Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            if(tot_occ_grid_cnt==3)
            {
                cout << "red block detected" << endl;
                for ( int j = 0; j < 4; j++ )
                {   
                    line(_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);
                } 
                cv::imshow(color_string + "_drawing", _Projected_image);
                red_Grid = Grid_size;
                red_occ_Grid = occ_grid;
            }
            else
               cout << "red block mis detected" << endl;
        }
        else
        {
            cout << "red block mis detected" << endl;
        }
    }

    if(color_string=="yellow")
    {
       if((Grid_size[0]==3 & Grid_size[1]==2 & Grid_size[2]==1) || (Grid_size[0]==2 & Grid_size[1]==3 & Grid_size[2]==1)
           || (Grid_size[0]==1 & Grid_size[1]==3 & Grid_size[2]==2) || (Grid_size[0]==3 & Grid_size[1]==1 & Grid_size[2]==2))
       {
           int tot_occ_grid_cnt=0;
           for(int i = 0; i < 6; i++)
               tot_occ_grid_cnt+=occ_grid[i];

           Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
           if(tot_occ_grid_cnt==4)
           {
               cout << "yellow block detected" << endl;
               for ( int j = 0; j < 4; j++ )
               {   
                   line(_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);
               } 
               cv::imshow(color_string + "_drawing", _Projected_image);
               yellow_Grid = Grid_size;
               yellow_occ_Grid = occ_grid;
           }
           else
               cout << "yellow block mis detected" << endl;
       }
       else
           cout << "yellow block mis detected" << endl;
    }

    if(color_string=="blue")
    {
       if((Grid_size[0]==2 & Grid_size[1]==2 & Grid_size[2]==2))
       {
           int tot_occ_grid_cnt=0;
           for(int i = 0; i < 8; i++)
               tot_occ_grid_cnt+=occ_grid[i];

           Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
           if(tot_occ_grid_cnt==4)
           {
               cout << "blue block detected" << endl;
               for ( int j = 0; j < 4; j++ )
               {   
                   line(_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);
               } 
               cv::imshow(color_string + "_drawing", _Projected_image);
               blue_Grid = Grid_size;
               blue_occ_Grid = occ_grid;
           }
           else
               cout << "blue block mis detected" << endl;
       }
       else
       {
           cout << "blue block mis detected" << endl;
       }
    }



    if(color_string=="brown")
    {
       if((Grid_size[0]==2 & Grid_size[1]==2 & Grid_size[2]==2))
       {
           int tot_occ_grid_cnt=0;
           for(int i = 0; i < 8; i++)
               tot_occ_grid_cnt+=occ_grid[i];

           Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
           if(tot_occ_grid_cnt==4)
           {
               cout << "brown block detected" << endl;
               for ( int j = 0; j < 4; j++ )
               {   
                   line(_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);
               } 
               cv::imshow(color_string + "_drawing", _Projected_image);
               brown_Grid = Grid_size;
               brown_occ_Grid = occ_grid;
           }
           else
           {
               cout << "brown block mis detected" << endl;
           }
       }
       else
       {
           cout << "brown block mis detected" << endl;
       }
    }


    if(color_string=="orange")
    {
       if((Grid_size[0]==1 & Grid_size[1]==3 & Grid_size[2]==2) || (Grid_size[0]==3 & Grid_size[1]==1 & Grid_size[2]==2) ||
          (Grid_size[0]==2 & Grid_size[1]==3 & Grid_size[2]==1) || (Grid_size[0]==3 & Grid_size[2]==2 & Grid_size[3]==1))
       {
           int tot_occ_grid_cnt=0;
           for(int i = 0; i < 6; i++)
               tot_occ_grid_cnt+=occ_grid[i];

           Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
           if(tot_occ_grid_cnt==4)
           {
               cout << "orange block detected" << endl;
               for ( int j = 0; j < 4; j++ )
               {   
                   line(_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);
               } 
               cv::imshow(color_string + "_drawing", _Projected_image);
               orange_Grid = Grid_size;
               orange_occ_Grid = occ_grid;
           }
           else
           {
               cout << "orange block mis detected" << "tot cnt occ: " << tot_occ_grid_cnt << endl;
           }
       }
       else
       {
           cout << "orange block mis detected     " << "Grid size: " << Grid_size[0] << " " << Grid_size[1] << " " << Grid_size[2] << endl;
       }
    }

    if(color_string=="purple")
    {
       if((Grid_size[0]==2 & Grid_size[1]==2 & Grid_size[2]==2))
       {
           int tot_occ_grid_cnt=0;
           for(int i = 0; i < 6; i++)
               tot_occ_grid_cnt+=occ_grid[i];

           Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
           if(tot_occ_grid_cnt==4)
           {
               cout << "purple block detected" << endl;
               for ( int j = 0; j < 4; j++ )
               {   
                   line(_Projected_image, _RectPoints.at(j), _RectPoints.at((j+1)%4), color);
               } 
               cv::imshow(color_string + "_drawing", _Projected_image);
               purple_Grid = Grid_size;
               purple_occ_Grid = occ_grid;
           }
           else 
               cout << "purple block mis detected" << endl;
       }
       else
       {
           cout << "purple block mis detected" << endl;
       }
    }
}

void ObjectPose::CloudView(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector) 
{
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(in_cloud);
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    
    //blocks until the cloud is actually rendered
    viewer.addPointCloud(in_cloud, "in_cloud", 0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "in_cloud", 0);
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
        if(i <3)
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
    exit(0);
}
