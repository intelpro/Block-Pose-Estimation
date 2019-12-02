#include "opencv2/opencv.hpp"
#include "config.h"
cv::Mat color_segment(cv::Mat, cv::Mat &redMask, cv::Mat &orangeMask, cv::Mat &yellowMask, cv::Mat &greenMask, cv::Mat &blueMask, cv::Mat &purpleMask, cv::Mat &brownMask, cv::Mat Mask); 
using namespace cv;
cv::Mat color_segment(cv::Mat rgb_image, cv::Mat &redMask, cv::Mat &orangeMask, cv::Mat &yellowMask, cv::Mat &greenMask, cv::Mat &blueMask, cv::Mat &purpleMask, cv::Mat &brownMask, cv::Mat Mask) 
{
    Mat im = rgb_image; 
    Mat hsv_image;
    cvtColor(im, hsv_image, COLOR_BGR2HSV); // convert BGR2HSV

    if (!im.data)
    {
        printf("No image data\n");
    }
    // Threshold the HSV image for red color. Since red color is separated in HSV space, we need to combine 2 criteria.
    Mat lower_red_hue;
    Mat upper_red_hue;
    inRange(hsv_image, Scalar(0, 100, 65), Scalar(2, 255, 255), lower_red_hue);
    inRange(hsv_image, Scalar(178, 100, 65), Scalar(179, 255, 255), upper_red_hue);
    Mat red;
    addWeighted(lower_red_hue, 1.0, upper_red_hue, 1.0, 0.0, red);

    // Threshold for orange color
    Mat orange;
    inRange(hsv_image, Scalar(5, 100, 65), Scalar(10, 255 ,255), orange);

    // Threshold for yellow color
    Mat yellow;
    inRange(hsv_image, Scalar(11, 200, 50), Scalar(25, 255, 255), yellow);

    // Threshold for green color
    Mat green;
    inRange(hsv_image, Scalar(50, 40, 30), Scalar(90, 255, 255), green);

    // Threshold for blue color
    Mat blue;
    inRange(hsv_image, Scalar(102, 100, 30), Scalar(130, 255, 255), blue);

    // Threshold for purple color. the hue for purple is the same as red. Only difference is value.
    Mat purple;
    inRange(hsv_image, Scalar(131, 50, 30), Scalar(170, 255, 140), purple);

    // Threshold for brown color. the hue for brown is the same as red and orange. Only difference is value.
    Mat brown;
    inRange(hsv_image, Scalar(1, 50, 10), Scalar(15, 200, 100), brown);

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

    // get masks.
    
    redMask = red;
    orangeMask = orange;
    yellowMask = yellow;
    greenMask = green;
    blueMask = blue;
    purpleMask = purple;
    brownMask = brown;
    Mask = red + orange + yellow + green + blue + purple + brown;
    
    // combine the masks and represent it.
    Mat masked = Mat(im.rows, im.cols, CV_8UC3, Scalar(0,0,0));
    cvtColor(red, red, CV_GRAY2BGR);
    Mat red_mask = Mat(im.rows, im.cols, CV_8UC3, Scalar(0,0,255));
    red = red_mask.mul(red, 1.0/255);
    addWeighted(im, 1.0, red, 1.0, 0.0, masked);

    cvtColor(orange, orange, CV_GRAY2BGR);
    Mat orange_mask = Mat(im.rows, im.cols, CV_8UC3, Scalar(0,100,255));
    orange = orange_mask.mul(orange, 1.0/255);
    addWeighted(masked, 1.0, orange, 1.0, 0.0, masked);

    cvtColor(yellow, yellow, CV_GRAY2BGR);
    Mat yellow_mask = Mat(im.rows, im.cols, CV_8UC3, Scalar(0,255,255));
    yellow = yellow_mask.mul(yellow, 1.0/255);
    addWeighted(masked, 1.0, yellow, 1.0, 0.0, masked);


    cvtColor(green, green, CV_GRAY2BGR);
    Mat green_mask = Mat(im.rows, im.cols, CV_8UC3, Scalar(0,255,0));
    green = green_mask.mul(green, 1.0/255);
    addWeighted(masked, 1.0, green, 1.0, 0.0, masked);


    cvtColor(blue, blue, CV_GRAY2BGR);
    Mat blue_mask = Mat(im.rows, im.cols, CV_8UC3, Scalar(255,0,0));
    blue = blue_mask.mul(blue, 1.0/255);
    addWeighted(masked, 1.0, blue, 1.0, 0.0, masked);


    cvtColor(purple, purple, CV_GRAY2BGR);
    Mat purple_mask = Mat(im.rows, im.cols, CV_8UC3, Scalar(255,0,255));
    purple = purple_mask.mul(purple, 1.0/255);
    addWeighted(masked, 1.0, purple, 1.0, 0.0, masked);


    cvtColor(brown, brown, CV_GRAY2BGR);
    Mat brown_mask = Mat(im.rows, im.cols, CV_8UC3, Scalar(15,30,50));
    brown = brown_mask.mul(brown, 1.0/255);
    addWeighted(masked, 1.0, brown, 1.0, 0.0, masked);
    return masked;
}
