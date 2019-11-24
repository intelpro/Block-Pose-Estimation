#include <DominantPlane.h>
#include <ObjectPose.h>
#include <SystemHandler.h>

using namespace std;
using namespace cv;

void Show_Results(cv::Mat& pointCloud, cv::Mat RGB_image_original, cv::Mat RGB_masked, std::string window_name);

int main(int argc, char** argv)
{
    ros::init(argc, argv,"block_pose");
    SystemHandler System(argv[1]);
    ros::spin();
    return 0;
}


void SystemHandler::preprocess_image(cv::Mat& imRGB, cv::Mat& imDepth)
{
    imRGB_processed = imRGB.clone(); 
    imDepth_processed = imDepth.clone();
    for (int y=0; y<height; y++)
    {
        for(int x=0; x < width; x++)
        {
            if(!(x>Crop_x_min && x<Crop_x_max && y>Crop_y_min && y<Crop_y_max))
            {
                imRGB_processed.at<Vec3b>(y,x)[0] = 0; 
                imRGB_processed.at<Vec3b>(y,x)[1] = 0; 
                imRGB_processed.at<Vec3b>(y,x)[2] = 0; 
                imDepth_processed.at<uint16_t>(y,x) = 0; 
            }
            else{
            }
        }
    }
}

void SystemHandler::Run_pipeline(cv::Mat& image_RGB, cv::Mat& image_Depth)
{
    std::vector<cv::Mat> Total_mask(7);
    clock_t start, end;

    cv::Mat pCloud(height, width, CV_32FC3);
    cv::Mat pCloud_inlier(height, width, CV_32FC3);

    start = clock();
    cv::Mat pointCloud = PlaneFinder->Depth2pcd(image_Depth);
    cv::Mat pcd_outlier = cv::Mat::zeros(height, width, CV_32FC3);
    Plane::Plane_model best_plane = PlaneFinder->RunRansac(pCloud_inlier);
    cv::Mat pCloud_outlier = cv::Mat::zeros(height, width, CV_32FC3);

    for (int y=0; y<height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            pCloud_outlier.at<cv::Vec3f>(y,x) = pointCloud.at<cv::Vec3f>(y,x) - pCloud_inlier.at<cv::Vec3f>(y,x);
        }
    }

    cv::Mat pcd_object = cv::Mat::zeros(height, width, CV_32FC3);
    // PlaneFinder->ObjectSegmentation(best_plane, pcd_object);

    end = clock();
    double result = (double)(end - start)/CLOCKS_PER_SEC;
	Show_Results(pCloud_outlier, imRGB, imRGB_processed, "seg_image");
    ColorSegmenation(image_RGB, Total_mask);
    if(Red_imshow_flag == 1)
        cv::imshow("Red", Total_mask[0]);
    if(Yellow_imshow_flag == 1)
        cv::imshow("Yellow", Total_mask[1]);
    if(Green_imshow_flag == 1)
        cv::imshow("Green", Total_mask[2]);
    if(Blue_imshow_flag == 1)
        cv::imshow("Blue", Total_mask[3]);
    if(Brown_imshow_flag == 1)
        cv::imshow("Brown", Total_mask[4]);
    if(Orange_imshow_flag == 1)
        cv::imshow("Orange", Total_mask[5]);
    if(Indigo_imshow_flag== 1)
        cv::imshow("Indigo", Total_mask[6]);
    waitKey(2);
    // PoseFinder->Accumulate_PointCloud(pCloud_outlier, Total_mask);
    /*
    cv::imshow("RGB", image_RGB);
    */
}

void SystemHandler::Publish_Message()
{
    std::vector<int> occ_grid_temp;
    std::vector<int> Grid_temp; 
    std::vector<Point3D> BB_Point_temp;
    std::vector<Point3D> Block_center_temp;

    // red block 
    Grid_temp = PoseFinder->red_Grid;
    occ_grid_temp = PoseFinder->red_occ_Grid;
    BB_Point_temp = PoseFinder->BB_info_red;
    Block_center_temp = PoseFinder->Block_center_red;
    Red_msg.Frame_id = Frame_count;
    Red_msg.Object_id = 0; 
    for(int i = 0; i < Grid_temp.size(); i++)
        Red_msg.Grid_size.push_back(Grid_temp[i]);
    for(int i = 0; i < occ_grid_temp.size(); i++)
        Red_msg.Occupancy_Grid.push_back(occ_grid_temp[i]);
    for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Red_msg.BB_Points.push_back(point);
    }
    for (std::vector<Point3D>::iterator it = Block_center_temp.begin(); it != Block_center_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Red_msg.Block_center_Points.push_back(point);
    }
    pub_red.publish(Red_msg);

    // yellow block 
    Grid_temp = PoseFinder->yellow_Grid;
    occ_grid_temp = PoseFinder->yellow_occ_Grid;
    BB_Point_temp = PoseFinder->BB_info_yellow;
    Block_center_temp = PoseFinder->Block_center_yellow;
    Yellow_msg.Frame_id = Frame_count;
    Yellow_msg.Object_id = 1;
    for(int i = 0; i < Grid_temp.size(); i++)
        Yellow_msg.Grid_size.push_back(Grid_temp[i]);
    for(int i = 0; i < occ_grid_temp.size(); i++)
        Yellow_msg.Occupancy_Grid.push_back(occ_grid_temp[i]);
    for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Yellow_msg.BB_Points.push_back(point);
    }
    for (std::vector<Point3D>::iterator it = Block_center_temp.begin(); it != Block_center_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Yellow_msg.Block_center_Points.push_back(point);
    }
    pub_yellow.publish(Yellow_msg);

    // Green block 
    // 
    // Temporally deactivate
    //
    // End of Green block 
    
    // blue block
    Grid_temp = PoseFinder->blue_Grid;
    occ_grid_temp = PoseFinder->blue_occ_Grid;
    BB_Point_temp = PoseFinder->BB_info_blue;
    Block_center_temp = PoseFinder->Block_center_blue;
    Blue_msg.Frame_id = Frame_count;
    Blue_msg.Object_id = 3;
    for(int i = 0; i < Grid_temp.size(); i++)
        Blue_msg.Grid_size.push_back(Grid_temp[i]);
    for(int i = 0; i < occ_grid_temp.size(); i++)
        Blue_msg.Occupancy_Grid.push_back(occ_grid_temp[i]);
    for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Blue_msg.BB_Points.push_back(point);
    }
    for (std::vector<Point3D>::iterator it = Block_center_temp.begin(); it != Block_center_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Blue_msg.Block_center_Points.push_back(point);
    }
    pub_blue.publish(Blue_msg);

    // Brown Block 
    Grid_temp = PoseFinder->brown_Grid;
    occ_grid_temp = PoseFinder->brown_occ_Grid;
    BB_Point_temp = PoseFinder->BB_info_brown;
    Block_center_temp = PoseFinder->Block_center_brown;
    Brown_msg.Frame_id = Frame_count;
    Brown_msg.Object_id = 4;
    for(int i = 0; i < Grid_temp.size(); i++)
        Brown_msg.Grid_size.push_back(Grid_temp[i]);
    for(int i = 0; i < occ_grid_temp.size(); i++)
        Brown_msg.Occupancy_Grid.push_back(occ_grid_temp[i]);
    for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Brown_msg.BB_Points.push_back(point);
    }
    for (std::vector<Point3D>::iterator it = Block_center_temp.begin(); it != Block_center_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Brown_msg.Block_center_Points.push_back(point);
    }
    pub_brown.publish(Brown_msg);

    // Orange Block
    Grid_temp = PoseFinder->orange_Grid;
    occ_grid_temp = PoseFinder->orange_occ_Grid;
    BB_Point_temp = PoseFinder->BB_info_orange;
    Block_center_temp = PoseFinder->Block_center_orange;
    Orange_msg.Frame_id = Frame_count;
    Orange_msg.Object_id = 5;
    for(int i = 0; i < Grid_temp.size(); i++)
        Orange_msg.Grid_size.push_back(Grid_temp[i]);
    for(int i = 0; i < occ_grid_temp.size(); i++)
        Orange_msg.Occupancy_Grid.push_back(occ_grid_temp[i]);
    for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Orange_msg.BB_Points.push_back(point);
    }
    for (std::vector<Point3D>::iterator it = Block_center_temp.begin(); it != Block_center_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Orange_msg.Block_center_Points.push_back(point);
    }
    pub_orange.publish(Orange_msg);

    // Purple Block
    Grid_temp = PoseFinder->purple_Grid;
    occ_grid_temp = PoseFinder->purple_occ_Grid;
    BB_Point_temp = PoseFinder->BB_info_purple;
    Block_center_temp = PoseFinder->Block_center_purple;
    Purple_msg.Frame_id = Frame_count;
    Purple_msg.Object_id = 6;
    for(int i = 0; i < Grid_temp.size(); i++)
        Purple_msg.Grid_size.push_back(Grid_temp[i]);
    for(int i = 0; i < occ_grid_temp.size(); i++)
        Purple_msg.Occupancy_Grid.push_back(occ_grid_temp[i]);
    for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Purple_msg.BB_Points.push_back(point);
    }
    for (std::vector<Point3D>::iterator it = Block_center_temp.begin(); it != Block_center_temp.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = (*it).z;
        Purple_msg.Block_center_Points.push_back(point);
    }
    pub_purple.publish(Purple_msg);

    // clear all object
    // Red message clear
    Red_msg.Grid_size.clear();
    Red_msg.Occupancy_Grid.clear();
    Red_msg.BB_Points.clear();
    Red_msg.Block_center_Points.clear();
    // Yellow message clear
    Yellow_msg.Grid_size.clear();
    Yellow_msg.Occupancy_Grid.clear();
    Yellow_msg.BB_Points.clear();
    Yellow_msg.Block_center_Points.clear();
    // Blue message clear
    Blue_msg.Grid_size.clear();
    Blue_msg.Occupancy_Grid.clear();
    Blue_msg.BB_Points.clear();
    Blue_msg.Block_center_Points.clear();
    // Brown message clear
    Brown_msg.Grid_size.clear();
    Brown_msg.Occupancy_Grid.clear();
    Brown_msg.BB_Points.clear();
    Brown_msg.Block_center_Points.clear();
    // Orange message clear
    Orange_msg.Grid_size.clear();
    Orange_msg.Occupancy_Grid.clear();
    Orange_msg.BB_Points.clear();
    Orange_msg.Block_center_Points.clear();
    // Purple message clear
    Purple_msg.Grid_size.clear();
    Purple_msg.Occupancy_Grid.clear();
    Purple_msg.BB_Points.clear();
    Purple_msg.Block_center_Points.clear();
    PoseFinder->ClearVariable();
}

void Show_Results(cv::Mat& pointCloud, cv::Mat RGB_image_original, cv::Mat RGB_masked, std::string window_name)
{
    cv::Mat RGB_image = RGB_image_original.clone();

    for (int y = 0; y < RGB_image.rows; y++)
    {
        for(int x = 0; x < RGB_image.cols; x++) {

            if (pointCloud.at<cv::Vec3f>(y, x)[0] == 0 && pointCloud.at<cv::Vec3f>(y, x)[1] == 0 && pointCloud.at<cv::Vec3f>(y, x)[2] == 0) {
                RGB_image.at<Vec3b>(y, x)[0] = 0;
                RGB_image.at<Vec3b>(y, x)[1] = 0;
                RGB_image.at<Vec3b>(y, x)[2] = 0;
            }
        }

    }

    cv::Mat red_image;
    imshow("RGB_image_orginal", RGB_image_original);
    imshow("RGB_image_seg", RGB_image);
    waitKey(2);
}

void SystemHandler::ColorSegmenation(cv::Mat& RGB_image, std::vector<cv::Mat>& Mask_vector)
{

    Mat hsv_image;
    cvtColor(RGB_image, hsv_image, COLOR_BGR2HSV); // convert BGR2HSV
    imshow("HSV_image", hsv_image);

    Mat lower_red_hue;
    Mat upper_red_hue;

    // Color threshold for red color
    inRange(hsv_image, lower_Red_value1, lower_Red_value2, lower_red_hue);
    inRange(hsv_image, upper_Red_value1, upper_Red_value2, upper_red_hue);

    Mat red;
    addWeighted(lower_red_hue, 1.0, upper_red_hue, 1.0, 0.0, red);

    // Threshold for orange color
    Mat orange;
    inRange(hsv_image, Orange_value1, Orange_value2, orange);

    // Threshold for yellow color
    cv::Mat yellow = cv::Mat::zeros(640, 480, CV_8UC1);
    inRange(hsv_image, Yellow_value1, Yellow_value2, yellow);

    // Threshold for green color
    Mat green;
    inRange(hsv_image, Green_value1, Green_value2, green);
    //inRange(hsv_image, Scalar(50, 100, 25), Scalar(90, 180, 60), green);

    // Threshold for blue color
    Mat blue;
    inRange(hsv_image, Blue_value1, Blue_value2, blue);
    //inRange(hsv_image, Scalar(102, 70, 20), Scalar(130, 200, 60), blue);

    // Threshold for purple color. the hue for purple is the same as red. Only difference is value.
    Mat lower_purple, upper_purple, purple;
    inRange(hsv_image, lower_Indigo_value1, lower_Indigo_value2, lower_purple);
    inRange(hsv_image, upper_Indigo_value1, upper_Indigo_value2, upper_purple);
    addWeighted(lower_purple, 1.0, upper_purple, 1.0, 0.0, purple);

   // Threshold for brown color. the hue for brown is the same as red and orange. Only difference is value.
    Mat lower_brown, upper_brown, brown;
    inRange(hsv_image, upper_Brown_value1, upper_Brown_value2, upper_brown);
    inRange(hsv_image, lower_Brown_value1, lower_Brown_value2, lower_brown);
    addWeighted(lower_brown, 1.0, upper_brown, 1.0, 0.0, brown);

    // morphological opening (remove small objects from the foreground)
    erode(red, red, getStructuringElement(MORPH_ELLIPSE, Size(15,15)));
    dilate(red, red, getStructuringElement(MORPH_ELLIPSE, Size(15,15)));

    // morphological closing (fill small holes in the foreground)
    dilate(red, red, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));
    erode(red, red, getStructuringElement(MORPH_ELLIPSE, Size(20,20)));
                            
    // morphological opening (remove small objects from the foreground)
    erode(orange, orange, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    dilate(orange, orange, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological closing (fill small holes in the foreground)
    dilate(orange, orange, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(orange, orange, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // morphological opening (remove small objects from the foreground)
    erode(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(4,4)));
    dilate(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological closing (fill small holes in the foreground)
    dilate(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    erode(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological opening (remove small objects from the foreground)
    erode(green, green, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(green, green, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // morphological closing (fill small holes in the foreground)
    dilate(green, green, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
    erode(green, green, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

    // morphological opening (remove small objects from the foreground)
    erode(blue, blue, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
    dilate(blue, blue, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

    // morphological closing (fill small holes in the foreground)
    dilate(blue, blue, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(blue, blue, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // morphological opening (remove small objects from the foreground)
    erode(purple, purple, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
    dilate(purple, purple, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

    // morphological closing (fill small holes in the foreground)
    dilate(purple, purple, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(purple, purple, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // morphological opening (remove small objects from the foreground)
    erode(brown, brown, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
    dilate(brown, brown, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

    // morphological closing (fill small holes in the foreground)
    dilate(brown, brown, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    erode(brown, brown, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    cv::Mat yellow_display = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat green_display = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat blue_display = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat brown_display = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat orange_display = cv::Mat::zeros(480, 640, CV_8UC1);

    for(int y=0; y < yellow.rows; y++)
    {
        for(int x=0; x < yellow.cols; x++)
        {
            if(yellow.at<uchar>(y,x) == 255 && y > 10 && x > 10)
            {
                // yellow.at<uchar>(y,x) = 0; 
                int cnt = 0; 
                for(int y1 = 0; y1 < 10; y1++)
                {
                    for(int x1 = 0; x1 < 10; x1++)
                    {
                        if(yellow.at<uchar>(y-y1,x-x1)==255)
                        {
                            cnt ++;
                        }
                    }
                }

                if(cnt<50)
                {
                    yellow_display.at<uchar>(y,x) = 0; 
                }
                else
                {
                    yellow_display.at<uchar>(y,x) = 255; 
                }
            }
            else
            {
                // cout << yellow.at<int>(y,x) << endl;
            }
        }
    }


    /*
    for(int y=0; y < blue.rows; y++)
    {
        for(int x=0; x < blue.cols; x++)
        {
            if(blue.at<uchar>(y,x) == 255 && y > 10 && x > 10)
            {
                int cnt = 0; 
                for(int y1 = 0; y1 < 10; y1++)
                {
                    for(int x1 = 0; x1 < 10; x1++)
                    {
                        if(blue.at<uchar>(y-y1,x-x1)==255)
                        {
                            cnt ++;
                        }
                    }
                }
                if(cnt<50)
                {
                    blue_display.at<uchar>(y,x) = 0; 
                }
                else
                {
                    blue_display.at<uchar>(y,x) = 255; 
                }
            }
            else
            {
                // cout << yellow.at<int>(y,x) << endl;
            }
        }
    }

    float sum_x_brown=0; 
    float sum_y_brown=0;
    int cnt_brown=0; 

    for(int y=0; y < brown.rows; y++)
    {
        for(int x=0; x < brown.cols; x++)
        {
            if(brown.at<uchar>(y,x)==255)
            {
                cnt_brown++;
                sum_x_brown += x;
                sum_y_brown += y;
            }
        }
    }

    float mean_x_brown = sum_x_brown/cnt_brown;
    float mean_y_brown = sum_y_brown/cnt_brown;

    for(int y=0; y < brown.rows; y++)
    {
        for(int x=0; x < brown.cols; x++)
        {
            if(brown.at<uchar>(y,x) == 255 && y > 10 && x > 10)
            {
                int cnt = 0; 
                for(int y1 = 0; y1 < 10; y1++)
                {
                    for(int x1 = 0; x1 < 10; x1++)
                    {
                        if(brown.at<uchar>(y-y1,x-x1)==255)
                        {
                            cnt ++;
                        }
                    }
                }
                if(cnt<50)
                {
                    brown_display.at<uchar>(y,x) = 0; 
                }
                else
                {
                    brown_display.at<uchar>(y,x) = 255; 
                }
            }
            else
            {
            }
            if(std::sqrt(std::pow(x - mean_x_brown, 2) + std::pow(y-mean_y_brown,2)) > 100)
                brown_display.at<uchar>(y,x) = 0; 
        }
    }
    */

    Mask_vector[0] = red.clone();
    Mask_vector[1] = yellow_display.clone();
    Mask_vector[2] = green.clone();
    Mask_vector[3] = blue.clone();
    Mask_vector[4] = brown.clone();
    Mask_vector[5] = orange.clone();
    Mask_vector[6] = purple.clone();
}
