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
    std::vector<int> red_Grid(3);
    std::vector<int> yellow_Grid(3);
    std::vector<int> green_Grid(3);
    std::vector<int> blue_Grid(3);
    std::vector<int> brown_Grid(3);
    std::vector<int> orange_Grid(3);
    std::vector<int> purple_Grid(3);
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
        // ProjectToDominantPlane(yellow_cloud, "yellow");
        // ProjectToDominantPlane(green_cloud, "green");
        // ProjectToDominantPlane(blue_cloud, "blue");
        // ProjectToDominantPlane(brown_cloud, "brown");
        // ProjectToDominantPlane(orange_cloud, "orange");
        // ProjectToDominantPlane(purple_cloud, "purple");
        Accum_idx = 0; 
    }

}



void ObjectPose::ProjectToDominantPlane(pcl::PointCloud<pcl::PointXYZRGB> in_cloud, std::string color_string)
{
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
    ProjectedCloudToImagePlane(color_string);
}

void ObjectPose::ProjectedCloudToImagePlane(std::string color_string)
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
    std::vector<Point> RectPoints = std::vector<Point> (4);
    fitRectangle(projected_image, RectPoints, color_string);
}


void ObjectPose::fitRectangle(cv::Mat projected_image, std::vector<Point> RectPoints, std::string color_string)
{
    static std::vector<Point> prev_RectPoints = std::vector<Point>(4);
    int thresh = 0.1;
    RNG rng(12345);
    Mat gray;
    cv::cvtColor(projected_image, gray, CV_BGR2GRAY);

    Mat canny_output;
    Canny( projected_image, canny_output, thresh, thresh*10, 3 );
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

    Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
    for ( int j = 0; j < 4; j++ )
    {   
        line(projected_image, RectPoints.at(j), RectPoints.at((j+1)%4), color);
    } 

	cv::imshow(color_string + "_drawing", projected_image);
    BackProjectToDominatPlane(RectPoints, color_string);

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

void ObjectPose::BackProjectToDominatPlane(std::vector<cv::Point> Rect_points, std::string color_string)
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
        cout << "red height: " << max_length << endl;
        pcl::copyPointCloud(red_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
        // MeasureOccupanyBB(pos_vector, max_length);
        MeasureOccupanyBB(pos_vector, max_length, color_string);
        cout << color_string << " Grid" << red_Grid[0] << " " << red_Grid[1] << " " << red_Grid[2] << endl;
    }
    else if(color_string=="green")
    {
        cout << "green height: " << max_length << endl;
        pcl::copyPointCloud(green_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
    }
    else if(color_string=="blue")
    {
        cout << "blue height: " << max_length << endl;
        pcl::copyPointCloud(blue_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
        MeasureOccupanyBB(pos_vector, max_length, color_string);
        cout << color_string << " Grid" << blue_Grid[0] << " " << blue_Grid[1] << " " << blue_Grid[2] << endl;
    }
    else if(color_string=="brown")
    {
        cout << "brown height: " << max_length << endl;
        pcl::copyPointCloud(brown_cloud, *Cloud_for_viewer);
        MeasureOccupanyBB(pos_vector, max_length, color_string);
        cout << color_string << " Grid" << brown_Grid[0] << " " << brown_Grid[1] << " " << brown_Grid[2] << endl;
        //CloudView(Cloud_for_viewer, pos_vector);
    }

    else if(color_string=="yellow")
    {
        cout << "yellow height: " << max_length << endl;
        pcl::copyPointCloud(yellow_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
        MeasureOccupanyBB(pos_vector, max_length, color_string);
        cout << color_string << " Grid" << yellow_Grid[0] << " " << yellow_Grid[1] << " " << yellow_Grid[2] << endl;
    }
    else if(color_string=="orange")
    {
        cout << "orange height: " << max_length << endl;
        pcl::copyPointCloud(orange_cloud, *Cloud_for_viewer);
        MeasureOccupanyBB(pos_vector, max_length, color_string);
        cout << color_string << " Grid" << orange_Grid[0] << " " << orange_Grid[1] << " " << orange_Grid[2] << endl;
    }
    else if(color_string=="purple")
    {
        cout << "purple height: " << max_length << endl;
        pcl::copyPointCloud(purple_cloud, *Cloud_for_viewer);
        MeasureOccupanyBB(pos_vector, max_length, color_string);
        cout << color_string << " Grid" << purple_Grid[0] << " " << purple_Grid[1] << " " << purple_Grid[2] << endl;
        // CloudView(Cloud_for_viewer, pos_vector);
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

void ObjectPose::MeasureOccupanyBB(std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector, float max_height, std::string color_string)
{
    std::vector<int> Grid_size(3);
    pcl::PointXYZRGB temp_cloud_1 = std::get<0>(pos_vector[0]);
    pcl::PointXYZRGB temp_cloud_2 = std::get<0>(pos_vector[1]);
    pcl::PointXYZRGB temp_cloud_3 = std::get<0>(pos_vector[2]);
    pcl::PointXYZRGB temp_cloud_4 = std::get<0>(pos_vector[3]);
    pcl::PointXYZRGB temp_cloud_5 = std::get<1>(pos_vector[0]);
    float dist1 = std::sqrt(std::pow(temp_cloud_1.x - temp_cloud_2.x, 2) + std::pow(temp_cloud_1.y - temp_cloud_2.y, 2) + std::pow(temp_cloud_1.z - temp_cloud_2.z,2));
    float dist2 = std::sqrt(std::pow(temp_cloud_1.x - temp_cloud_4.x, 2) + std::pow(temp_cloud_1.y - temp_cloud_4.y, 2) + std::pow(temp_cloud_1.z - temp_cloud_4.z,2));

    cout << "dist1: " << dist1 << "dist2: " << dist2 << "height: " << max_height << endl;
    cout << "a: " << best_plane.a <<  "b: " << best_plane.b << "c: " << best_plane.c << endl;
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

    // i vector of cuboid
    cv::Vec3f Cuboid_I(temp_cloud_1.x - temp_cloud_2.x, temp_cloud_1.y - temp_cloud_2.y, temp_cloud_1.z - temp_cloud_2.z);
    // j vector of cuboid
    cv::Vec3f Cuboid_J(temp_cloud_1.x - temp_cloud_4.x, temp_cloud_1.y - temp_cloud_4.y, temp_cloud_1.z - temp_cloud_4.z);
    // k vector of cuboid
    cv::Vec3f Cuboid_K(temp_cloud_1.x - temp_cloud_5.x, temp_cloud_1.y - temp_cloud_5.y, temp_cloud_1.z - temp_cloud_5.z);
    cout << "I vector: " << Cuboid_I << endl;
    cout << "J vector: " << Cuboid_J << endl;
    cout << "K vector: " << Cuboid_K << endl;

    if(color_string=="red")
    {
        red_Grid=Grid_size;
        for(int i=1; i <= red_Grid[0]; i++)
        {
            for(int j=1; j <= red_Grid[1]; j++)
            {
                for(int k=1; k <= red_Grid[2]; k++)
                {
                    int cnt_x = 0; 
                    int cnt_y = 0; 
                    int cnt_z = 0; 
                    // I vector of cuboid
                    cv::Vec3f Cuboid_I_temp = i*Cuboid_I/red_Grid[0];
                    // J vector of cuboid
                    cv::Vec3f Cuboid_J_temp = j*Cuboid_J/red_Grid[1];
                    // k vector of cuboid
                    cv::Vec3f Cuboid_K_temp = k*Cuboid_K/red_Grid[2];
                    float Cuboid_I_value = std::sqrt(std::pow(Cuboid_I_temp[0],2) + std::pow(Cuboid_I_temp[1],2) + std::pow(Cuboid_I_temp[2],2));
                    float Cuboid_J_value = std::sqrt(std::pow(Cuboid_J_temp[0],2) + std::pow(Cuboid_J_temp[1],2) + std::pow(Cuboid_J_temp[2],2));
                    float Cuboid_K_value = std::sqrt(std::pow(Cuboid_K_temp[0],2) + std::pow(Cuboid_K_temp[1],2) + std::pow(Cuboid_K_temp[2],2));
                    for(int l=0; l < red_cloud.size(); l++)
                    {
                        pcl::PointXYZRGB temp_cloud = red_cloud[l];
                        cv::Vec3f temp_cloud_vec3f = cv::Vec3f(temp_cloud_1.x - temp_cloud.x, temp_cloud_1.y - temp_cloud.y, temp_cloud_2.z - temp_cloud.z);
                        float dot_x = Cuboid_I_temp.dot(temp_cloud_vec3f);
                        float dot_y = Cuboid_J_temp.dot(temp_cloud_vec3f);
                        float dot_z = Cuboid_K_temp.dot(temp_cloud_vec3f);
                        if(0 < dot_x & dot_x < Cuboid_I_value)
                            cnt_x++;
                        if(0 < dot_y & dot_y < Cuboid_J_value)
                            cnt_y++;
                        if(0 < dot_z & dot_z < Cuboid_K_value)
                            cnt_z++;
                    }
                    cout << "i: " << i << "j: " << j << "k: " << k << endl;
                    cout << "cnt x: " <<  cnt_x << "cnt y: " << cnt_y << "cnt z: " << cnt_z << endl; 
                }
            }
        }
    }
    else if(color_string=="yellow")
        yellow_Grid=Grid_size;
    else if(color_string=="green")
        green_Grid=Grid_size;
    else if(color_string=="blue")
        blue_Grid=Grid_size;
    else if(color_string=="brown")
        brown_Grid=Grid_size;
    else if(color_string=="orange")
        orange_Grid=Grid_size;
    else if(color_string=="purple")
        purple_Grid=Grid_size;
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

