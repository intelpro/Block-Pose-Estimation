#include <DominantPlane.h>
#include <ObjectPose.h>
#include <SystemHandler.h>

using namespace std;
using namespace cv;

void Show_Results(cv::Mat& pointCloud, cv::Mat RGB_image_original, std::string window_name);
void imageCb(cv::Mat& RGB_image, std::vector<cv::Mat>& Mask_vector);
float fx = 615.6707153320312;
float fy = 615.962158203125;
float cx = 328.0010681152344;
float cy = 241.31031799316406;
float scale = 1000;
float Distance_theshold = 0.005;
float unit_length = 0.025; 
float Threshold_for_occgrid = unit_length/2;
int width = 640;
int height = 480;
int max_iter = 100;
int Depth_Accum_iter = 3; 


int main(int argc, char** argv)
{
    Plane::DominantPlane plane(fx,fy,cx,cy, scale, Distance_theshold, max_iter, width, height);
    ObjectPose pose(height, width, Depth_Accum_iter, fx, fy, cx, cy, unit_length, Threshold_for_occgrid, &plane);
    ros::init(argc, argv,"block_pose");
    SubscribeAndPublish SAPObject(&plane, &pose);
    ros::spin();
    return 0;
}


void SubscribeAndPublish::Run_pipeline(cv::Mat& image_RGB, cv::Mat& image_Depth)
{
    std::vector<cv::Mat> Total_mask(7);
    clock_t start, end;

    cv::Mat pCloud(height, width, CV_32FC3);
    cv::Mat pCloud_inlier(height, width, CV_32FC3);

    start = clock();
    cv::Mat pointCloud = plane_obj->Depth2pcd(image_Depth);
    cv::Mat pcd_outlier = cv::Mat::zeros(height, width, CV_32FC3);
    Plane::Plane_model best_plane = plane_obj->RunRansac(pCloud_inlier);
    cv::Mat pCloud_outlier = cv::Mat::zeros(height, width, CV_32FC3);

    for (int y=0; y<height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            pCloud_outlier.at<cv::Vec3f>(y,x) = pointCloud.at<cv::Vec3f>(y,x) - pCloud_inlier.at<cv::Vec3f>(y,x);
        }
    }

    cv::Mat pcd_object = cv::Mat::zeros(height, width, CV_32FC3);
    plane_obj->ObjectSegmentation(best_plane, pcd_object);

    end = clock();
    double result = (double)(end - start)/CLOCKS_PER_SEC;
	Show_Results(pCloud_outlier, image_RGB, "seg_image");
    imageCb(image_RGB, Total_mask);
    pose_obj->Accumulate_PointCloud(pCloud_outlier, Total_mask);

    // Mask_vector[0] = red_display.clone();
    /*
    cv::imshow("purple", Total_mask[6]);
    cv::imshow("brown", Total_mask[4]);
    cv::imshow("RGB", image_RGB);
    cv::imshow("green", Total_mask[2]);
    cv::imshow("blue", Total_mask[3]);
    cv::imshow("orange", Total_mask[5]);
    cv::imshow("purple", Total_mask[6]);
    */
    // cout << "consuming time: " << result << "(s)" << endl;
}

void Show_Results(cv::Mat& pointCloud, cv::Mat RGB_image_original, std::string window_name)
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
    imshow("RGB_image_org", RGB_image_original);
    imshow("RGB_image_seg", RGB_image);
    waitKey(2);
}

void imageCb(cv::Mat& RGB_image, std::vector<cv::Mat>& Mask_vector)
{

    Mat hsv_image;
    cvtColor(RGB_image, hsv_image, COLOR_BGR2HSV); // convert BGR2HSV
    imshow("HSV_image", hsv_image);

    Mat lower_red_hue;
    Mat upper_red_hue;

    // inRange(hsv_image, Scalar(0, 150, 50), Scalar(2, 255, 180), lower_red_hue);
    // inRange(hsv_image, Scalar(150, 100, 66), Scalar(179, 255, 130), upper_red_hue);
    inRange(hsv_image, Scalar(0, 100, 65), Scalar(3, 255, 255), lower_red_hue);
    inRange(hsv_image, Scalar(178, 100, 65), Scalar(179, 255, 255), upper_red_hue);

    Mat red;
    addWeighted(lower_red_hue, 1.0, upper_red_hue, 1.0, 0.0, red);

    // Threshold for orange color
    Mat orange;
    inRange(hsv_image, Scalar(3, 100, 30), Scalar(10, 255,200), orange);

    // Threshold for yellow color
    cv::Mat yellow = cv::Mat::zeros(640, 480, CV_8UC1);
    inRange(hsv_image, Scalar(11, 130, 50), Scalar(25, 255, 255), yellow);

    // Threshold for green color
    Mat green;
    inRange(hsv_image, Scalar(50, 40, 20), Scalar(90, 255, 100), green);
    //inRange(hsv_image, Scalar(50, 100, 25), Scalar(90, 180, 60), green);

    // Threshold for blue color
    Mat blue;
    inRange(hsv_image, Scalar(90, 100, 10), Scalar(130, 255, 100), blue);
    //inRange(hsv_image, Scalar(102, 70, 20), Scalar(130, 200, 60), blue);

    // Threshold for purple color. the hue for purple is the same as red. Only difference is value.
    Mat purple;
    inRange(hsv_image, Scalar(140, 35, 30), Scalar(179, 90, 110), purple);

    // Threshold for orange color
    // Threshold for brown color. the hue for brown is the same as red and orange. Only difference is value.

    Mat lower_brown, upper_brown, brown;
    inRange(hsv_image, Scalar(0, 10, 50), Scalar(10, 50, 100), upper_brown);
    inRange(hsv_image, Scalar(160, 10, 50), Scalar(179, 50, 100), lower_brown);
    addWeighted(lower_brown, 1.0, upper_brown, 1.0, 0.0, brown);

    // morphological opening (remove small objects from the foreground)
    erode(red, red, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    dilate(red, red, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological closing (fill small holes in the foreground)
    dilate(red, red, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(red, red, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
                            
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
    dilate(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // morphological opening (remove small objects from the foreground)
    erode(green, green, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    dilate(green, green, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological closing (fill small holes in the foreground)
    dilate(green, green, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(green, green, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // morphological opening (remove small objects from the foreground)
    erode(blue, blue, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    dilate(blue, blue, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological closing (fill small holes in the foreground)
    dilate(blue, blue, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(blue, blue, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // morphological opening (remove small objects from the foreground)
    erode(purple, purple, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    dilate(purple, purple, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological closing (fill small holes in the foreground)
    dilate(purple, purple, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(purple, purple, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // morphological opening (remove small objects from the foreground)
    erode(brown, brown, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    dilate(brown, brown, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological closing (fill small holes in the foreground)
    dilate(brown, brown, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(brown, brown, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    cv::Mat red_display = cv::Mat::zeros(480, 640, CV_8UC1);
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

    for(int y=0; y < red.rows; y++)
    {
        for(int x=0; x < red.cols; x++)
        {
            if(red.at<uchar>(y,x) == 255 && y > 10 && x > 10)
            {
                // yellow.at<uchar>(y,x) = 0; 
                int cnt = 0; 
                for(int y1 = 0; y1 < 10; y1++)
                {
                    for(int x1 = 0; x1 < 10; x1++)
                    {
                        if(red.at<uchar>(y-y1,x-x1)==255)
                        {
                            cnt ++;
                        }
                    }
                }
                if(cnt<50)
                {
                    red_display.at<uchar>(y,x) = 0; 
                }
                else
                {
                    red_display.at<uchar>(y,x) = 255; 
                }
            }
            else
            {
                // cout << yellow.at<int>(y,x) << endl;
            }
        }
    }

    for(int y=0; y < green.rows; y++)
    {
        for(int x=0; x < green.cols; x++)
        {
            if(green.at<uchar>(y,x) == 255 && y > 10 && x > 10)
            {
                int cnt = 0; 
                for(int y1 = 0; y1 < 10; y1++)
                {
                    for(int x1 = 0; x1 < 10; x1++)
                    {
                        if(green.at<uchar>(y-y1,x-x1)==255)
                        {
                            cnt ++;
                        }
                    }
                }
                if(cnt<50)
                {
                    green_display.at<uchar>(y,x) = 0; 
                }
                else
                {
                    green_display.at<uchar>(y,x) = 255; 
                }
            }
            else
            {
                // cout << yellow.at<int>(y,x) << endl;
            }
        }
    }

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

    float sum_x_orange=0; 
    float sum_y_orange=0;
    int cnt_orange=0; 

    for(int y=0; y < orange.rows; y++)
    {
        for(int x=0; x < orange.cols; x++)
        {
            if(orange.at<uchar>(y,x)==255)
            {
                cnt_orange++;
                sum_x_orange += x;
                sum_y_orange += y;
            }
        }
    }

    float mean_x_orange = sum_x_orange/cnt_orange;
    float mean_y_orange = sum_y_orange/cnt_orange;

    for(int y=0; y < orange.rows; y++)
    {
        for(int x=0; x < orange.cols; x++)
        {
            if(orange.at<uchar>(y,x) == 255 && y > 10 && x > 10)
            {
                int cnt = 0; 
                for(int y1 = 0; y1 < 10; y1++)
                {
                    for(int x1 = 0; x1 < 10; x1++)
                    {
                        if(orange.at<uchar>(y-y1,x-x1)==255)
                            cnt ++;
                    }
                }
                if(cnt<50)
                {
                    orange_display.at<uchar>(y,x) = 0; 
                }
                else
                {
                    orange_display.at<uchar>(y,x) = 255; 
                }
            }
            else
            {
                // cout << yellow.at<int>(y,x) << endl;
            }
            if(std::sqrt(std::pow(x - mean_x_orange, 2) + std::pow(y-mean_y_orange,2)) > 100)
                orange_display.at<uchar>(y,x) = 0; 
        }
    }

    Mask_vector[0] = red_display.clone();
    Mask_vector[1] = yellow_display.clone();
    Mask_vector[2] = green_display.clone();
    Mask_vector[3] = blue_display.clone();
    Mask_vector[4] = brown_display.clone();
    Mask_vector[5] = orange_display.clone();
    Mask_vector[6] = purple.clone();
}
