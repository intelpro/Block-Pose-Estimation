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

ObjectPose::ObjectPose(int _height, int _width, int _Accum_iter,float _fx, float _fy, float _cx, float _cy, Plane::DominantPlane* plane)
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
    best_plane = plane->cur_best_plane;
    box_flag=0;
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

    }

    Accum_idx++;
    if(Accum_idx >= Accum_iter)
    {
        ProjectToDominantPlane(red_cloud, "red");
        ProjectToDominantPlane(yellow_cloud, "yellow");
        ProjectToDominantPlane(green_cloud, "green");
        ProjectToDominantPlane(blue_cloud, "blue");
        ProjectToDominantPlane(orange_cloud, "orange");
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
        else if(color_string=="blue")
        {
            ProjectedPoint.r = 0; 
            ProjectedPoint.g = 0; 
            ProjectedPoint.b = 255; 
        }
        else if(color_string=="orange")
        {
            ProjectedPoint.r = 255; 
            ProjectedPoint.g = 165;
            ProjectedPoint.b = 0;
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
        else if(color_string=="orange")
        {
            projected_image.at<Vec3b>(x,y)[1] = 165; 
            projected_image.at<Vec3b>(x,y)[2] = 255;
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
    // TODO: FIND max length between dominant plane and point cloud
    float max_length;
    if(color_string=="red")
        max_length = FindBlockHeight(red_cloud);
    else if(color_string=="green")
        max_length = FindBlockHeight(green_cloud);
    else if(color_string=="blue")
        max_length = FindBlockHeight(blue_cloud);
    else if(color_string=="yellow")
        max_length = FindBlockHeight(yellow_cloud);
    else if(color_string=="orange")
        max_length = FindBlockHeight(orange_cloud);
    std::vector<pair<pcl::PointXYZRGB, pcl::PointXYZRGB>> pos_vector; 
    for (int i = 0; i < Rect_points.size(); i++)
    {
        float x = Rect_points[i].x;
        float y = Rect_points[i].y;
        Z_deominator = a*(x-cx)/fx+b*(y-cy)/fy +c;
        float Z = -d/Z_deominator;
        float X = (x-cx)/fx*Z;
        float Y = (y-cy)/fy*Z;
        cout << "X: " << X << "Y: " << Y << "Z: " << Z << endl;
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
        pcl::copyPointCloud(red_cloud, *Cloud_for_viewer);
    }
    else if(color_string=="green")
    {
        pcl::copyPointCloud(green_cloud, *Cloud_for_viewer);
    }
    else if(color_string=="blue")
    {
        pcl::copyPointCloud(blue_cloud, *Cloud_for_viewer);
        // CloudView(Cloud_for_viewer, pos_vector);
    }
    else if(color_string=="yellow")
    {
        cout << "yellow length: " << max_length << endl;
        pcl::copyPointCloud(yellow_cloud, *Cloud_for_viewer);
        CloudView(Cloud_for_viewer, pos_vector);
    }
    else if(color_string=="orange")
    {
        cout << "orange length: " << max_length << endl;
        pcl::copyPointCloud(orange_cloud, *Cloud_for_viewer);
    }

}

float ObjectPose::FindBlockHeight(pcl::PointCloud<pcl::PointXYZRGB> in_cloud)
{
    float max_height=0;
    float dist_temp;
    float a = best_plane.a; 
    float b = best_plane.b;
    float c = best_plane.c;
    float d = best_plane.d;
    float denom = std::sqrt(a*a + b*b + c*c);
    for(int i=0; i<in_cloud.size(); i++)
    {
        pcl::PointXYZRGB temp_cloud = in_cloud[i];
        dist_temp = std::abs(a*temp_cloud.x + b*temp_cloud.y + c*temp_cloud.z + d)/denom;
        if(max_height<dist_temp)
            max_height = dist_temp;
    }
    return max_height;
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

