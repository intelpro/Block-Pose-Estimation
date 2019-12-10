#include <DominantPlane.h>
#include <ObjectPose.h>
#include <SystemHandler.h>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;


int main(int argc, char** argv)
{
    ros::init(argc, argv,"block_pose");
    SystemHandler System(argv[1]);
    ros::spin();
    return 0;
}

void SystemHandler::preprocess_image(cv::Mat& imRGB)
{
    cnt_preprocessing++;
    for (int y=0; y<height; y++)
    {
        for(int x=0; x < width; x++)
        {
            if(!(x>Crop_x_min && x<Crop_x_max && y>Crop_y_min && y<Crop_y_max))
            {
                imRGB_processed2.at<Vec3f>(y,x)[0] = 0; 
                imRGB_processed2.at<Vec3f>(y,x)[1] = 0; 
                imRGB_processed2.at<Vec3f>(y,x)[2] = 0; 
            }
            else{
                if(MeanImg_flag==1) 
                {
                    imRGB_processed2.at<Vec3f>(y,x)[0] += imRGB.at<Vec3b>(y,x)[0];
                    imRGB_processed2.at<Vec3f>(y,x)[1] += imRGB.at<Vec3b>(y,x)[1];
                    imRGB_processed2.at<Vec3f>(y,x)[2] += imRGB.at<Vec3b>(y,x)[2];
                }
            }
        }
    }
}



void SystemHandler::Run_pipeline(cv::Mat& image_RGB, cv::Mat& image_Depth)
{
    std::vector<cv::Mat> Mask_vector(8);
    std::vector<cv::Mat> Mask_vector_refined(8);

    if(system_mode==10)
    {
        std::vector<cv::Mat> Unknown_Objmask(7);

        cv::Mat pCloud = cv::Mat::zeros(height, width, CV_32FC3);
        cv::Mat pCloud_inlier = cv::Mat::zeros(height, width, CV_32FC3);

        cv::Mat pointCloud = PlaneFinder->Depth2pcd(image_Depth);
        cv::Mat pcd_outlier = cv::Mat::zeros(height, width, CV_32FC3);
        Plane::Plane_model best_plane = PlaneFinder->RunRansac(pCloud_inlier);
        cv::Mat pCloud_outlier = cv::Mat::zeros(height, width, CV_32FC3);

        for (int y=0; y<height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                pCloud_outlier.at<cv::Vec3f>(y,x) = pointCloud.at<cv::Vec3f>(y,x) - pCloud_inlier.at<cv::Vec3f>(y,x);
                if(imDepth.at<uint16_t>(y,x) == 0)
                    pCloud_outlier.at<cv::Vec3f>(y,x) = 0;
            }
        }

        cv::Mat pcd_object = cv::Mat::zeros(height, width, CV_32FC3);
        PlaneFinder->ObjectSegmentation(best_plane, pcd_object);

        cv::Mat masked_image = imRGB_processed.clone();
        Show_Results(pCloud_outlier, imRGB_processed, masked_image, "seg_image");
        ExtractObjectMask(masked_image, Unknown_Objmask);
        Identify_Object(imRGB_processed, Unknown_Objmask, Mask_vector);
        get_cleanMask(Mask_vector, Mask_vector_refined);

        cv::Mat total_mask = Mask_vector_refined[0] + Mask_vector_refined[1] + Mask_vector_refined[2] +
                            Mask_vector_refined[3] + Mask_vector_refined[4] + Mask_vector_refined[5]+ Mask_vector_refined[6];

        PoseFinder->Test_all_flag = 1;
        PoseFinder->Accumulate_PointCloud(pCloud_outlier, Mask_vector_refined);
    }

    else if(system_mode!=10)
    {
        cv::Mat object_mask = cv::Mat::zeros(height, width, CV_8UC1);

        cv::Mat pCloud = cv::Mat::zeros(height, width, CV_32FC3);
        cv::Mat pCloud_inlier = cv::Mat::zeros(height, width, CV_32FC3);

        cv::Mat pointCloud = PlaneFinder->Depth2pcd(image_Depth);
        cv::Mat pcd_outlier = cv::Mat::zeros(height, width, CV_32FC3);
        Plane::Plane_model best_plane = PlaneFinder->RunRansac(pCloud_inlier);
        cv::Mat pCloud_outlier = cv::Mat::zeros(height, width, CV_32FC3);

        for (int y=0; y<height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                pCloud_outlier.at<cv::Vec3f>(y,x) = pointCloud.at<cv::Vec3f>(y,x) - pCloud_inlier.at<cv::Vec3f>(y,x);
                if(imDepth.at<uint16_t>(y,x) == 0)
                    pCloud_outlier.at<cv::Vec3f>(y,x) = 0;
            }
        }

        cv::Mat pcd_object = cv::Mat::zeros(height, width, CV_32FC3);
        PlaneFinder->ObjectSegmentation(best_plane, pcd_object);

        cv::Mat masked_image = imRGB_processed.clone();
        Show_Results(pCloud_outlier, imRGB_processed, masked_image, "seg_image");
        ExtractObjectMask2(masked_image, object_mask);
        getOutputMask(Mask_vector, object_mask, system_mode);
        get_cleanMask(Mask_vector, Mask_vector_refined);
        PoseFinder->Test_Individual_flag = 1;
        PoseFinder->Test_all_flag = 0;
        PoseFinder->SetIndividualMode(system_mode);
        PoseFinder->Accumulate_PointCloud(pCloud_outlier, Mask_vector_refined);
    }

    if(DepthImgShow_flag==1)
    {
        double min;
        double max;
        cv::minMaxIdx(imDepth, &min, &max);
        cv::Mat adjMap;
        cv::convertScaleAbs(imDepth, adjMap, 255/0.01);
        cv::Mat falseColorsMap;
        cv::imshow("Depth image", adjMap);
        waitKey(2);
    }

    if(ColorDebug_flag==1)
    {
        cv::hconcat(imColorDebug, Mask_vector_refined[7], imColorDebug);
        cv::imshow("Color_debug", imColorDebug);
        waitKey(2);
    }

}

void SystemHandler::ExtractObjectMask(cv::Mat image_RGB, std::vector<cv::Mat>& Object_mask)
{
    Mat imGray;
    cvtColor(image_RGB, imGray, COLOR_BGR2GRAY); // convert BGR2HSV

    // morphological opening (remove small objects from the foreground)
    erode(imGray, imGray, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imGray, imGray, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    int data_cnt = 0;
    std::vector<pair<int, int>> pixel_pos;
    for(int y=0; y<height; y++)
    {
        for(int x=0; x<width; x++)
        { if(imGray.at<uint8_t>(y,x)!=0)
            {
                data_cnt++;
                pixel_pos.push_back(make_pair(int(y), int(x)));
            }
        }
    }
    Mat p = Mat::zeros(data_cnt, 2, CV_32F);
    for(int i=0; i<data_cnt; i++) {
        p.at<float>(i,0) = std::get<0>(pixel_pos[i]);
        p.at<float>(i,1) = std::get<1>(pixel_pos[i]);
    }

    int K = 7;
    Mat bestLabels, centers, clustered;
    cv::kmeans(p, K, bestLabels,
            TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 1), 3, KMEANS_PP_CENTERS, centers);

    int colors[K];
    for(int i=0; i<K; i++) {
        colors[i] = 255/(i+1);
    }


    cv::Mat object1 = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat object2 = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat object3 = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat object4 = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat object5 = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat object6 = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat object7 = cv::Mat::zeros(height, width, CV_8UC1);

    for(int i=0; i < data_cnt; i++)
    {
        int y = std::get<0>(pixel_pos[i]);
        int x = std::get<1>(pixel_pos[i]);
        if(bestLabels.at<int>(i)==0)
            object1.at<uint8_t>(y,x) = 255;
        else if(bestLabels.at<int>(i)==1)
            object2.at<uint8_t>(y,x) = 255;
        else if(bestLabels.at<int>(i)==2)
            object3.at<uint8_t>(y,x) = 255;
        else if(bestLabels.at<int>(i)==3)
            object4.at<uint8_t>(y,x) = 255;
        else if(bestLabels.at<int>(i)==4)
            object5.at<uint8_t>(y,x) = 255;
        else if(bestLabels.at<int>(i)==5)
            object6.at<uint8_t>(y,x) = 255;
        else if(bestLabels.at<int>(i)==6)
            object7.at<uint8_t>(y,x) = 255;
    }

    Object_mask[0] = object1.clone();
    Object_mask[1] = object2.clone();
    Object_mask[2] = object3.clone();
    Object_mask[3] = object4.clone();
    Object_mask[4] = object5.clone();
    Object_mask[5] = object6.clone();
    Object_mask[6] = object7.clone();
}

void SystemHandler::ExtractObjectMask2(cv::Mat image_RGB, cv::Mat& object)
{
    Mat hsv_image, gray_image;
    cvtColor(image_RGB, hsv_image, COLOR_BGR2HSV); // convert BGR2HSV
    cvtColor(image_RGB, gray_image, COLOR_BGR2GRAY); // convert BGR2GRAY

    int data_cnt = 0;
    std::vector<pair<int, int>> pixel_pos;

    // morphological opening (remove small objects from the foreground)
    dilate(gray_image, gray_image, getStructuringElement(MORPH_ELLIPSE, Size(15,15)));
    erode(gray_image, gray_image, getStructuringElement(MORPH_ELLIPSE, Size(15,15)));

    int cx=320;
    int cy=240;
    int bx=10;
    int by=10;
    int sx=cx, sy=cy;
    bool check_flg=false;
    while(!check_flg)
    {
        int lower_y = cy - by;
        int higher_y = cy + by;
        int lower_x = cx - bx; 
        int higher_x = cx + bx;
        check_flg=false;
        if(lower_y > 0 && higher_y < height && lower_x > 0 && higher_x < width)
        {
            for(int y=lower_y; y<=higher_y; y++)
            {
                for(int x=lower_x; x<=higher_x; x++)
                {
                    if(gray_image.at<uint8_t>(y, x)!=0)
                    {
                        sx=x;
                        sy=y;
                        check_flg=true;
                    }
                }
            }
            bx+=10;
            by+=10;
        }
        else
            check_flg = true;
    }

    int x=sx; 
    int y=sy;
    pixel_pos.push_back(make_pair(y,x));
    int count=0;
    int count1=1;
    while(count!=count1)
    {
        y=std::get<0>(pixel_pos[count]);
        x=std::get<1>(pixel_pos[count]);
        if(x > 0 && x < width && y < height && y > 0)
        {
            if(gray_image.at<uint8_t>(y+1,x)!=0){
                if(object.at<uint8_t>(y+1,x) != 255){
                    pixel_pos.push_back(make_pair(y+1,x));
                    object.at<uint8_t>(y+1,x) = 255;
                    count1++;
                }
            }
            if(gray_image.at<uint8_t>(y,x-1)!=0){
                if(object.at<uint8_t>(y,x-1) != 255){
                    pixel_pos.push_back(make_pair(y,x-1));
                    object.at<uint8_t>(y,x-1) = 255;
                    count1++;
                }
            }        
            if(gray_image.at<uint8_t>(y-1,x)!=0){
                if(object.at<uint8_t>(y-1,x) != 255) {
                    pixel_pos.push_back(make_pair(y-1,x));
                    object.at<uint8_t>(y-1,x) = 255;
                    count1++;
                    }
            }
            if(gray_image.at<uint8_t>(y,x+1)!=0){
                 if(object.at<uint8_t>(y,x+1) != 255) {
                    pixel_pos.push_back(make_pair(y,x+1));
                    object.at<uint8_t>(y,x+1) = 255;
                    count1++;    
                 }
            }
        }
        else
            break;
        count++;       
    }
    cv::imshow("mask", object);
}

void SystemHandler::getOutputMask(std::vector<cv::Mat>& output_mask, cv::Mat object_Mask, int color_info)
{
    cv::Mat display_window = cv::Mat::zeros(height, width, CV_8UC3);
    int index = 0;
    if(color_info==0) // red
        index = 0;
    else if(color_info==1) // brown
        index = 4;
    else if(color_info==2) // purple
        index = 6;
    else if(color_info==3) // yellow
        index = 1;
    else if(color_info==4) // green
        index = 2;
    else if(color_info==5) // blue
        index = 3;
    else if(color_info==6) // orange
        index = 5;
    for(int i=0; i<output_mask.size()-1; i++)
    {
        cv::Mat temp_mask = cv::Mat::zeros(height, width, CV_8UC1);
        if(index==i)
            output_mask[i] = object_Mask.clone();
        else
            output_mask[i] = temp_mask.clone();
    }

    dilate(object_Mask, object_Mask, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    erode(object_Mask, object_Mask, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological closing (fill small holes in the foreground)
    erode(object_Mask, object_Mask, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    dilate(object_Mask, object_Mask, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    for(int y=0; y < height; y++)
    {
        for(int x=0; x < width; x++)
        {
            if(index == 0 && object_Mask.at<uint8_t>(y,x) == 255 && Red_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 0;
                display_window.at<Vec3b>(y,x)[2] = 255;
            }

            if(index == 1 & object_Mask.at<uint8_t>(y,x) == 255 && Yellow_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 255;
                display_window.at<Vec3b>(y,x)[2] = 255;
            }

            if(index == 2 && object_Mask.at<uint8_t>(y,x) == 255 && Green_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 255;
                display_window.at<Vec3b>(y,x)[2] = 0;
            }

            if(index == 3 && object_Mask.at<uint8_t>(y,x) == 255 && Blue_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 255;
                display_window.at<Vec3b>(y,x)[1] = 0;
                display_window.at<Vec3b>(y,x)[2] = 0;
            }

            if(index == 4 && object_Mask.at<uint8_t>(y,x) == 255 && Brown_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 50;
                display_window.at<Vec3b>(y,x)[1] = 60;
                display_window.at<Vec3b>(y,x)[2] = 72;
            }

            if(index == 5 && object_Mask.at<uint8_t>(y,x) == 255 && Orange_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 165;
                display_window.at<Vec3b>(y,x)[2] = 255;
            }

            if(index == 6 && object_Mask.at<uint8_t>(y,x) == 255 && Indigo_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 100;
                display_window.at<Vec3b>(y,x)[1] = 50;
                display_window.at<Vec3b>(y,x)[2] = 40;
            }
        }
    }
    output_mask[7] = display_window.clone();
}

void SystemHandler::get_cleanMask(std::vector<cv::Mat> object_Mask, std::vector<cv::Mat>& output_mask)
{

    cv::Mat display_window = cv::Mat::zeros(height, width, CV_8UC3);
    for(int i=0; i<object_Mask.size()-1; i++)
    {
        cv::Mat hsv_image;
        cvtColor(imRGB_processed, hsv_image, COLOR_BGR2HSV); // convert BGR2HSV
        std::string color_string;
        if(i==0)
            color_string = "red";
        else if(i==1)
            color_string = "yellow";
        else if(i==2)
            color_string = "green";
        else if(i==3)
            color_string = "blue";
        else if(i==4)
            color_string = "brown";
        else if(i==5)
            color_string = "orange";
        else if(i==6)
            color_string = "purple";
        Rect rect = boundingRect(object_Mask[i]);
        Point pt1, pt2;
        pt1.x = rect.x;
        pt1.y = rect.y;
        pt2.x = rect.x + rect.width;
        pt2.y = rect.y + rect.height;
        // Draws the rect in the original image and show it
        // rectangle(red, pt1, pt2, CV_RGB(255,255,255), 1);
        int thresh_patch = 5;
        int y_lower = pt1.y-thresh_patch;
        int y_higher = pt2.y+thresh_patch;
        int x_lower = pt1.x-thresh_patch;
        int x_higher = pt2.x+thresh_patch;
        if(y_higher > height)
            y_higher = height;
        if(x_higher > width)
            x_higher = width;
        if(y_lower < 0)
            y_lower = 0;
        if(x_lower < 0)
            x_lower = 0;
        
        cv::Mat part(y_higher - y_lower, x_higher-x_lower, CV_8UC3);

        for(int y=0; y<height; y++)
        {
            for(int x=0; x<width; x++)
            {
                if(y >= y_lower && y < y_higher && x >= x_lower && x < x_higher)
                {
                    part.at<Vec3b>(y-y_lower,x-x_lower)[0] = hsv_image.at<Vec3b>(y,x)[0];
                    part.at<Vec3b>(y-y_lower,x-x_lower)[1] = hsv_image.at<Vec3b>(y,x)[1];
                    part.at<Vec3b>(y-y_lower,x-x_lower)[2] = hsv_image.at<Vec3b>(y,x)[2];
                }
            }
        }

        Mat lower_hue2;
        Mat upper_hue2;
        if(color_string=="red")
        {
            inRange(part, lower_Red_value1, lower_Red_value2, lower_hue2);
            inRange(part, upper_Red_value1, upper_Red_value2, upper_hue2);
        }
        else if(color_string=="orange")
        {
            inRange(part, Orange_value1, Orange_value2, lower_hue2);
        }
        else if(color_string=="yellow")
        {
            inRange(part, Yellow_value1, Yellow_value2, lower_hue2);
        }
        else if(color_string=="green")
        {
            inRange(part, Green_value1, Green_value2, lower_hue2);
        }
        else if(color_string=="blue")
        {
            inRange(part, Blue_value1, Blue_value2, lower_hue2);
        }
        else if(color_string=="purple")
        {
            inRange(part, lower_Indigo_value1, lower_Indigo_value2, lower_hue2);
        }
        else if(color_string=="brown")
        {
            inRange(part, upper_Brown_value1, upper_Brown_value2, lower_hue2);
            inRange(part, lower_Brown_value1, lower_Brown_value2, upper_hue2);
        }

        cv::Mat part_result;
        part_result = lower_hue2 + upper_hue2;
        cv::Mat result = cv::Mat::zeros(height, width, CV_8UC1);

        for(int y=0; y<part_result.rows; y++)
        {
            for(int x=0; x<part_result.cols; x++)
            {
                if(part_result.at<uint8_t>(y,x)==255)
                {
                    if(y+y_lower < height && x+x_lower < width)
                    {
                        result.at<uint8_t>(y+y_lower, x+x_lower)=255;
                    }
                }
            }
        }

        erode(result, result, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
        dilate(result, result, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

        // morphological closing (fill small holes in the foreground)
        dilate(result, result, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
        erode(result, result, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

        output_mask[i] = result;

        for(int y=0; y < height; y++)
        {
            for(int x=0; x < width; x++)
            {
                if(i == 0 && result.at<uint8_t>(y,x) == 255 && Red_imshow_flag==1)
                {
                    display_window.at<Vec3b>(y,x)[0] = 0;
                    display_window.at<Vec3b>(y,x)[1] = 0;
                    display_window.at<Vec3b>(y,x)[2] = 255;
                }

                if(i == 1 & result.at<uint8_t>(y,x) == 255 && Yellow_imshow_flag==1)
                {
                    display_window.at<Vec3b>(y,x)[0] = 0;
                    display_window.at<Vec3b>(y,x)[1] = 255;
                    display_window.at<Vec3b>(y,x)[2] = 255;
                }

                if(i == 2 && result.at<uint8_t>(y,x) == 255 && Green_imshow_flag==1)
                {
                    display_window.at<Vec3b>(y,x)[0] = 0;
                    display_window.at<Vec3b>(y,x)[1] = 255;
                    display_window.at<Vec3b>(y,x)[2] = 0;
                }

                if(i == 3 && result.at<uint8_t>(y,x) == 255 && Blue_imshow_flag==1)
                {
                    display_window.at<Vec3b>(y,x)[0] = 255;
                    display_window.at<Vec3b>(y,x)[1] = 0;
                    display_window.at<Vec3b>(y,x)[2] = 0;
                }

                if(i == 4 && result.at<uint8_t>(y,x) == 255 && Brown_imshow_flag==1)
                {
                    display_window.at<Vec3b>(y,x)[0] = 50;
                    display_window.at<Vec3b>(y,x)[1] = 60;
                    display_window.at<Vec3b>(y,x)[2] = 72;
                }

                if(i == 5 && result.at<uint8_t>(y,x) == 255 && Orange_imshow_flag==1)
                {
                    display_window.at<Vec3b>(y,x)[0] = 0;
                    display_window.at<Vec3b>(y,x)[1] = 165;
                    display_window.at<Vec3b>(y,x)[2] = 255;
                }

                if(i == 6 && result.at<uint8_t>(y,x) == 255 && Indigo_imshow_flag==1)
                {
                    display_window.at<Vec3b>(y,x)[0] = 100;
                    display_window.at<Vec3b>(y,x)[1] = 50;
                    display_window.at<Vec3b>(y,x)[2] = 40;
                }
            }
        }
    }
    output_mask[7] = display_window.clone();
}
void SystemHandler::Identify_Object(cv::Mat imRGB, std::vector<cv::Mat> Unknown_Objmask, std::vector<cv::Mat>& Object_mask)
{

    Mat hsv_image;
    cvtColor(imRGB_processed, hsv_image, COLOR_BGR2HSV); // convert BGR2HSV
    imshow("HSV_image", hsv_image);
    std::vector<std::pair<float, float>> hue_vec;
	std::vector<int> size_vector(Unknown_Objmask.size());
    for(int i = 0; i < Unknown_Objmask.size(); i++)
    {
        std::vector<int> hue_value;
        for(int y=0; y < height; y++)
        {
            for(int x=0; x < width; x++)
            {
                if(Unknown_Objmask[i].at<uint8_t>(y,x) == 255)
                {
                    if(hsv_image.at<Vec3b>(y,x)[0] < 160)
                        hue_value.push_back(hsv_image.at<Vec3b>(y,x)[0]);
                    else
                        hue_value.push_back(hsv_image.at<Vec3b>(y,x)[0] - 180);
                }
            }
        }
        std::sort(hue_value.begin(), hue_value.end());
        float median = hue_value[hue_value.size() / 2];
        hue_vec.push_back(make_pair(float(i), median));
    }
    std::sort(hue_vec.begin(), hue_vec.end(), sort_pair_second<float, float>());

    /*
    for(int i = 0; i < hue_vec.size(); i++)
    {
        cout << "index: " << std::get<0>(hue_vec[i]) << endl;
        cout << "avg: " << std::get<1>(hue_vec[i]) << endl;
    }

    float avg_r_arr[2];
    for(int i = 0; i < 2; i++)
    {
        float total_r = 0;
        int cnt = 0;
        for(int y=0; y < height; y++)
        {
            for(int x=0; x < width; x++)
            {
                if(Unknown_Objmask[std::get<0>(hue_vec[i])].at<uint8_t>(y,x)==255)
                {
                    total_r += imRGB.at<Vec3b>(y,x)[2];
                    cnt++;
                }
            }
        }
        float avg_r = total_r/cnt;
        avg_r_arr[i] = avg_r;
    }
    */

    cv::Mat red_new = Unknown_Objmask[std::get<0>(hue_vec[0])];
    cv::Mat orange_new = Unknown_Objmask[std::get<0>(hue_vec[1])];
    cv::Mat brown_new = Unknown_Objmask[std::get<0>(hue_vec[2])].clone();
    cv::Mat yellow_new = Unknown_Objmask[std::get<0>(hue_vec[3])].clone();
    cv::Mat green_new = Unknown_Objmask[std::get<0>(hue_vec[4])].clone();
    cv::Mat blue_new = Unknown_Objmask[std::get<0>(hue_vec[5])].clone();
    cv::Mat purple_new = Unknown_Objmask[std::get<0>(hue_vec[6])].clone();
    cv::Mat display_window = cv::Mat::zeros(height, width, CV_8UC3);
    for(int y=0; y < height; y++)
    {
        for(int x=0; x < width; x++)
        {
            if(red_new.at<uint8_t>(y,x) == 255 && Red_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 0;
                display_window.at<Vec3b>(y,x)[2] = 255;
            }
            if(orange_new.at<uint8_t>(y,x) == 255 && Orange_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 165;
                display_window.at<Vec3b>(y,x)[2] = 255;
            }

            if(brown_new.at<uint8_t>(y,x) == 255 && Brown_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 50;
                display_window.at<Vec3b>(y,x)[1] = 60;
                display_window.at<Vec3b>(y,x)[2] = 72;
            }

            if(yellow_new.at<uint8_t>(y,x) == 255 && Yellow_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 255;
                display_window.at<Vec3b>(y,x)[2] = 255;
            }

            if(green_new.at<uint8_t>(y,x) == 255 && Green_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 255;
                display_window.at<Vec3b>(y,x)[2] = 0;
            }

            if(blue_new.at<uint8_t>(y,x) == 255 && Blue_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 255;
                display_window.at<Vec3b>(y,x)[1] = 0;
                display_window.at<Vec3b>(y,x)[2] = 0;
            }

            if(purple_new.at<uint8_t>(y,x) == 255 && Indigo_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 100;
                display_window.at<Vec3b>(y,x)[1] = 50;
                display_window.at<Vec3b>(y,x)[2] = 40;
            }
        }
    }

    Object_mask[0] = red_new.clone();
    Object_mask[1] = yellow_new.clone();
    Object_mask[2] = green_new.clone();
    Object_mask[3] = blue_new.clone();
    Object_mask[4] = brown_new.clone();
    Object_mask[5] = orange_new.clone();
    Object_mask[6] = purple_new.clone();
    Object_mask[7] = display_window.clone();
}

void SystemHandler::Publish_Message()
{
    std::vector<int> occ_grid_temp;
    std::vector<int> Grid_temp; 
    std::vector<Point3D> BB_Point_temp;

    if(system_mode==10)
    {
        // red block 
        BB_Point_temp = PoseFinder->BB_info_red;
        Red_msg.Frame_id = Frame_count;
        Red_msg.Object_id = 0; 
        for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
            geometry_msgs::Point point;
            point.x = (*it).x;
            point.y = (*it).y;
            point.z = (*it).z;
            Red_msg.BB_Points.push_back(point);
        }
        pub_red.publish(Red_msg);

        // yellow block 
        BB_Point_temp = PoseFinder->BB_info_yellow;
        Yellow_msg.Frame_id = Frame_count;
        Yellow_msg.Object_id = 1;
        for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
            geometry_msgs::Point point;
            point.x = (*it).x;
            point.y = (*it).y;
            point.z = (*it).z;
            Yellow_msg.BB_Points.push_back(point);
        }
        pub_yellow.publish(Yellow_msg);

        // Green block 
        BB_Point_temp = PoseFinder->BB_info_green;
        Green_msg.Frame_id = Frame_count;
        Green_msg.Object_id = 2;
        for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
            geometry_msgs::Point point;
            point.x = (*it).x;
            point.y = (*it).y;
            point.z = (*it).z;
            Green_msg.BB_Points.push_back(point);
        }
        pub_green.publish(Green_msg);


        // blue block
        BB_Point_temp = PoseFinder->BB_info_blue;
        Blue_msg.Frame_id = Frame_count;
        Blue_msg.Object_id = 3;
        for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
            geometry_msgs::Point point;
            point.x = (*it).x;
            point.y = (*it).y;
            point.z = (*it).z;
            Blue_msg.BB_Points.push_back(point);
        }
        pub_blue.publish(Blue_msg);

        // Brown Block 
        BB_Point_temp = PoseFinder->BB_info_brown;
        Brown_msg.Frame_id = Frame_count;
        Brown_msg.Object_id = 4;
        for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
            geometry_msgs::Point point;
            point.x = (*it).x;
            point.y = (*it).y;
            point.z = (*it).z;
            Brown_msg.BB_Points.push_back(point);
        }
        pub_brown.publish(Brown_msg);

        // Orange Block
        BB_Point_temp = PoseFinder->BB_info_orange;
        Orange_msg.Frame_id = Frame_count;
        Orange_msg.Object_id = 5;
        for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
            geometry_msgs::Point point;
            point.x = (*it).x;
            point.y = (*it).y;
            point.z = (*it).z;
            Orange_msg.BB_Points.push_back(point);
        }
        pub_orange.publish(Orange_msg);

        // Indigo Block
        BB_Point_temp = PoseFinder->BB_info_Indigo;
        Indigo_msg.Frame_id = Frame_count;
        Indigo_msg.Object_id = 6;
        for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
            geometry_msgs::Point point;
            point.x = (*it).x;
            point.y = (*it).y;
            point.z = (*it).z;
            Indigo_msg.BB_Points.push_back(point);
        }
        pub_Indigo.publish(Indigo_msg);
    }
    else
    {
		// red block 
		Grid_temp = PoseFinder->red_Grid;
		occ_grid_temp = PoseFinder->red_occ_Grid;
		BB_Point_temp = PoseFinder->BB_info_red;
		std::vector<Point3D> Block_center_temp = PoseFinder->Block_center_red;
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
		Block_center_temp.clear();
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
		Grid_temp = PoseFinder->green_Grid;
		occ_grid_temp = PoseFinder->green_occ_Grid;
		BB_Point_temp = PoseFinder->BB_info_green;
		Block_center_temp = PoseFinder->Block_center_green;
		Green_msg.Frame_id = Frame_count;
		Green_msg.Object_id = 2;
		for(int i = 0; i < Grid_temp.size(); i++)
			Green_msg.Grid_size.push_back(Grid_temp[i]);
		for(int i = 0; i < occ_grid_temp.size(); i++)
			Green_msg.Occupancy_Grid.push_back(occ_grid_temp[i]);
		for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
			geometry_msgs::Point point;
			point.x = (*it).x;
			point.y = (*it).y;
			point.z = (*it).z;
			Green_msg.BB_Points.push_back(point);
		}
		for (std::vector<Point3D>::iterator it = Block_center_temp.begin(); it != Block_center_temp.end(); ++it) {
			geometry_msgs::Point point;
			point.x = (*it).x;
			point.y = (*it).y;
			point.z = (*it).z;
			Green_msg.Block_center_Points.push_back(point);
		}
		pub_green.publish(Green_msg);


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

		// Indigo Block
		Grid_temp = PoseFinder->Indigo_Grid;
		occ_grid_temp = PoseFinder->Indigo_occ_Grid;
		BB_Point_temp = PoseFinder->BB_info_Indigo;
		Block_center_temp = PoseFinder->Block_center_Indigo;
		Indigo_msg.Frame_id = Frame_count;
		Indigo_msg.Object_id = 6;
		for(int i = 0; i < Grid_temp.size(); i++)
			Indigo_msg.Grid_size.push_back(Grid_temp[i]);
		for(int i = 0; i < occ_grid_temp.size(); i++)
			Indigo_msg.Occupancy_Grid.push_back(occ_grid_temp[i]);
		for (std::vector<Point3D>::iterator it = BB_Point_temp.begin(); it != BB_Point_temp.end(); ++it) {
			geometry_msgs::Point point;
			point.x = (*it).x;
			point.y = (*it).y;
			point.z = (*it).z;
			Indigo_msg.BB_Points.push_back(point);
		}
		for (std::vector<Point3D>::iterator it = Block_center_temp.begin(); it != Block_center_temp.end(); ++it) {
			geometry_msgs::Point point;
			point.x = (*it).x;
			point.y = (*it).y;
			point.z = (*it).z;
			Indigo_msg.Block_center_Points.push_back(point);
		}
		pub_Indigo.publish(Indigo_msg);
    }

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
    // Green message clear
    Green_msg.Grid_size.clear();
    Green_msg.Occupancy_Grid.clear();
    Green_msg.BB_Points.clear();
    Green_msg.Block_center_Points.clear();
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
    // Indigo message clear
    Indigo_msg.Grid_size.clear();
    Indigo_msg.Occupancy_Grid.clear();
    Indigo_msg.BB_Points.clear();
    Indigo_msg.Block_center_Points.clear();
    PoseFinder->ClearVariable();
}

void SystemHandler::Show_Results(cv::Mat& pointCloud, cv::Mat RGB_image_original, cv::Mat RGB_masked, std::string window_name)
{
    cv::Mat RGB_image = RGB_image_original.clone();

    for (int y = 0; y < RGB_image.rows; y++)
    {
        for(int x = 0; x < RGB_image.cols; x++) {
            if (pointCloud.at<cv::Vec3f>(y, x)[0] == 0 && pointCloud.at<cv::Vec3f>(y, x)[1] == 0 && pointCloud.at<cv::Vec3f>(y, x)[2] == 0) {
                RGB_masked.at<Vec3b>(y, x)[0] = 0;
                RGB_masked.at<Vec3b>(y, x)[1] = 0;
                RGB_masked.at<Vec3b>(y, x)[2] = 0;
            }
            if(imDepth.at<uint16_t>(y,x)==0)
            {
                RGB_masked.at<Vec3b>(y, x)[0] = 0;
                RGB_masked.at<Vec3b>(y, x)[1] = 0;
                RGB_masked.at<Vec3b>(y, x)[2] = 0;
            }
        }

    }
    if(ColorDebug_flag==1)
        imColorDebug = RGB_image.clone();
    if(SegImgShow_flag)
        imshow("RGB_image_seg", RGB_masked);
    waitKey(2);
}

void SystemHandler::ColorSegmenation(cv::Mat RGB_image, std::vector<cv::Mat>& Mask_vector)
{
    Mat hsv_image;
    cvtColor(RGB_image, hsv_image, COLOR_BGR2HSV); // convert BGR2HSV
    if(HSVImgShow_flag)
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
    cv::Mat yellow;
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
    Mat lower_purple;
    Mat upper_purple;
    Mat purple(height, width, CV_8UC1);
    inRange(hsv_image, lower_Indigo_value1, lower_Indigo_value2, purple);
    //inRange(hsv_image, upper_Indigo_value1, upper_Indigo_value2, upper_purple);
    //addWeighted(lower_purple, 1.0, upper_purple, 1.0, 0.0, purple);
    //

   // Threshold for brown color. the hue for brown is the same as red and orange. Only difference is value.
    Mat lower_brown(height, width, CV_8UC1);
    Mat upper_brown(height, width, CV_8UC1);
    Mat brown(height, width, CV_8UC1);
    inRange(hsv_image, upper_Brown_value1, upper_Brown_value2, upper_brown);
    inRange(hsv_image, lower_Brown_value1, lower_Brown_value2, lower_brown);
    addWeighted(lower_brown, 1.0, upper_brown, 1.0, 0.0, brown);

    for(int y=0; y < red.rows; y++)
    {
        for(int x=0; x < red.cols; x++)
        {
            if(orange.at<uint8_t>(y,x) == 255)
                red.at<uint8_t>(y,x) = 0; 
        }
    }

    for(int y=0; y < orange.rows; y++)
    {
        for(int x=0; x < orange.cols; x++)
        {
            if(red.at<uint8_t>(y,x) == 255)
                orange.at<uint8_t>(y,x) = 0; 
        }
    }

    
    // morphological opening (remove small objects from the foreground)
    erode(red, red, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    dilate(red, red, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological closing (fill small holes in the foreground)
    dilate(red, red, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    erode(red, red, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
                            
    // morphological opening (remove small objects from the foreground)
    erode(orange, orange, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    dilate(orange, orange, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological closing (fill small holes in the foreground)
    dilate(orange, orange, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    erode(orange, orange, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    // morphological opening (remove small objects from the foreground)
    erode(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(15,15)));
    dilate(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(15,15)));

    // morphological closing (fill small holes in the foreground)
    dilate(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(30,30)));
    erode(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(30,30)));

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

    cv::Mat red_new = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat orange_new = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat yellow_new = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat green_new = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat blue_new = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat brown_new = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat purple_new = cv::Mat::zeros(height, width, CV_8UC3);

    for(int y=0; y < red.rows; y++)
    {
        for(int x=0; x < red.cols; x++)
        {
            if(orange.at<uint8_t>(y,x) == 255)
                red.at<uint8_t>(y,x) = 0; 
        }
    }

    for(int y=0; y < orange.rows; y++)
    {
        for(int x=0; x < orange.cols; x++)
        {
            if(red.at<uint8_t>(y,x) == 255)
                orange.at<uint8_t>(y,x) = 0; 
        }
    }

    cv::Mat display_window = cv::Mat::zeros(height, width, CV_8UC3);
    for(int y=0; y < display_window.rows; y++)
    {
        for(int x=0; x < display_window.cols; x++)
        {
            if(red_new.at<Vec3b>(y,x)[0] == 255 && Red_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 0;
                display_window.at<Vec3b>(y,x)[2] = 255;
            }

            if(orange_new.at<Vec3b>(y,x)[0] == 255 && Orange_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 165;
                display_window.at<Vec3b>(y,x)[2] = 255;
            }

            if(yellow_new.at<Vec3b>(y,x)[0] == 255 && Yellow_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 255;
                display_window.at<Vec3b>(y,x)[2] = 255;
            }

            if(green_new.at<Vec3b>(y,x)[0] == 255 && Green_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 0;
                display_window.at<Vec3b>(y,x)[1] = 255;
                display_window.at<Vec3b>(y,x)[2] = 0;
            }

            if(blue_new.at<Vec3b>(y,x)[0] == 255 && Blue_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 255;
                display_window.at<Vec3b>(y,x)[1] = 0;
                display_window.at<Vec3b>(y,x)[2] = 0;
            }

            if(brown_new.at<Vec3b>(y,x)[0] == 255 && Brown_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 50;
                display_window.at<Vec3b>(y,x)[1] = 60;
                display_window.at<Vec3b>(y,x)[2] = 72;
            }

            if(purple_new.at<Vec3b>(y,x)[0] == 255 && Indigo_imshow_flag==1)
            {
                display_window.at<Vec3b>(y,x)[0] = 100;
                display_window.at<Vec3b>(y,x)[1] = 50;
                display_window.at<Vec3b>(y,x)[2] = 40;
            }
        }
    }


    Mask_vector[0] = red_new.clone();
    Mask_vector[1] = yellow_new.clone();
    Mask_vector[2] = green_new.clone();
    Mask_vector[3] = blue_new.clone();
    Mask_vector[4] = brown_new.clone();
    Mask_vector[5] = orange_new.clone();
    Mask_vector[6] = purple_new.clone();
    Mask_vector[7] = display_window.clone();
}


