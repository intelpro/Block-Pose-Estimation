#include "DominantPlane.h"
#include "config.h"
//
// Created by intelpro on 3/26/19.
//
using namespace std;
using namespace cv;
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
        //cout << depth.size << endl;
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

    Plane_model DominantPlane::RunRansac(cv::Mat &pcd_inlier) {
        unsigned long rand_idx;
        float cost = 0;
        float best_cost = std::numeric_limits<float>::infinity();
        unsigned int best_N_inlier = 0;
        Plane_model best_plane;
        std::srand(time(NULL));

        // cout << "frame id:" << frame_id << " Number of pcds:" <<  pcd_concat.size() << endl;
        Plane_model plane;

        for (int i = 0; i < max_iter; i++)
        {
            cv::Mat pcd_temp_inlier = cv::Mat::zeros(height, width, CV_32FC3);
            std::vector<cv::Vec3f> sampled_pcd;
            for(int j =0; j < 3; j++)
            {
                rand_idx = int(pcd_concat.size() * (std::rand()/(double)RAND_MAX));
                sampled_pcd.push_back(pcd_concat.at(rand_idx));
                // cout << "sampled_pcd" << sampled_pcd[j] << endl;
            }
            FindPlaneFromSampledPcds(sampled_pcd, plane);
           // cout << plane.a << " " << plane.b << " "<< plane.c << " " << plane.denominator << " " << endl;
            compute_inlier_cost(plane, pcd_temp_inlier, Pointcloud);
            cost = plane.avg_distance / N_inlier;
            if(best_cost > cost)
            {
                best_cost = cost;
                best_plane = plane;
                //cout << "inlier1/" << "iter:" << i << " " << "cost:" << cost << " " << "average distance:" << plane.avg_distance << " " << endl;
                pcd_inlier = pcd_temp_inlier;
            }
            N_inlier = 0;
            pcd_temp_inlier.release();
            sampled_pcd.clear();
        }

        /*
        cv::Mat pcd_outlier = cv::Mat::zeros(height, width, CV_32FC3);
        for (int y = 0; y < height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                pcd_outlier.at<cv::Vec3f>(y, x) = Pointcloud.at<cv::Vec3f>(y, x) - pcd_inlier.at<cv::Vec3f>(y, x);
            }
        }

        cv::Mat pcd_inlier_2 = cv::Mat::zeros(height, width, CV_32FC3);
        Plane_model plane_2;
        //best_N_inlier = 0;
        float best_cost_2 = std::numeric_limits<float>::infinity();
        for (int i = 0; i < max_iter/2; i++)
        {
            cv::Mat pcd_temp_inlier = cv::Mat::zeros(height, width, CV_32FC3);
            std::vector<cv::Vec3f> sampled_pcd2;
            for(int j =0; j < 3; j++)
            {
                rand_idx = int(pcd_concat.size() * (std::rand()/(double)RAND_MAX));
                sampled_pcd2.push_back(pcd_concat.at(rand_idx));
            }
            FindPlaneFromSampledPcds(sampled_pcd2, plane_2);
            compute_inlier_cost(plane_2, pcd_temp_inlier, pcd_outlier);
            cost = plane_2.avg_distance / N_inlier;
            if(best_cost_2 > cost)
            {
                best_cost_2 = cost;
                pcd_inlier_2 = pcd_temp_inlier;
                cout << "inlier2/" << "iter:" << i << " " << "cost:" << cost << " " << "average distance:" << plane.avg_distance << " " << endl;
            }

            N_inlier = 0;
            pcd_temp_inlier.release();
            sampled_pcd2.clear();
        }

        for (int y = 0; y < height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                pcd_inlier.at<cv::Vec3f>(y, x) += pcd_inlier_2.at<cv::Vec3f>(y, x);
            }
        }

        */

        ResetValue();
        return best_plane;
    }

    void DominantPlane::Object_Segmentation(cv::Mat& pcd_inlier, cv::Mat& pcd_object)
    {
        float Threshold = 1.5;
        float differ = 0;
        float differ_2 = 0;
        for (int i=0; i<pcd_inlier.rows; i++)
        {
            for(int j=0; j<pcd_inlier.cols; j++)
            {
                float sum = 0;
                float sum_2 = 0;
                for (int k=0; k<3; k++)

                {
                    differ = Pointcloud.at<cv::Vec3f>(i, j)[k] - pcd_inlier.at<cv::Vec3f>(i, j)[k];
                    differ_2 = differ * differ;
                    sum += differ_2;

                    sum_2 += pcd_inlier.at<cv::Vec3f>(i, j)[k] * pcd_inlier.at<cv::Vec3f>(i, j)[k];
                }
                if (sum_2 == 0)
                {
                    if (sum < Threshold)
                    {
                        pcd_object.at<cv::Vec3f>(i, j) = Pointcloud.at<cv::Vec3f>(i, j);

                    }

                }
            }
        }

    }

    void DominantPlane::ObjectSegmentation(Plane_model& plane, cv::Mat& pcd_object)
    {
        float Threshold = 0.2;
        for (int i=0; i<pcd_object.rows; i++)
        {
            for(int j=0; j<pcd_object.cols; j++)
            {
                float x = Pointcloud.at<cv::Vec3f>(i, j)[0];
                float y = Pointcloud.at<cv::Vec3f>(i, j)[1];
                float z = Pointcloud.at<cv::Vec3f>(i, j)[2];
                float dist = abs(plane.a * x + plane.b * y + plane.c * z + plane.d) / plane.denominator;

                if (0.01 < dist && dist < Threshold)
                {
                    pcd_object.at<cv::Vec3f>(i, j) = Pointcloud.at<cv::Vec3f>(i, j);

                }
            }

        }

    }

    cv::Mat DominantPlane::DistantPlaneSegmentation(Plane_model plane, float distance)
    {
        cv::Mat pcd_object = cv::Mat::zeros(height, width, CV_32FC3);
        float Threshold = 0.02;
        for (int i=0; i<pcd_object.rows; i++)
        {
            for(int j=0; j<pcd_object.cols; j++)
            {
                float x = Pointcloud.at<cv::Vec3f>(i, j)[0];
                float y = Pointcloud.at<cv::Vec3f>(i, j)[1];
                float z = Pointcloud.at<cv::Vec3f>(i, j)[2];
                float dist = abs(plane.a * x + plane.b * y + plane.c * z + plane.d) / plane.denominator;

                if ( -Threshold + distance < dist && dist < Threshold + distance)
                {
                    pcd_object.at<cv::Vec3f>(i, j) = Pointcloud.at<cv::Vec3f>(i, j);

                }
            }

        }
        return pcd_object;
    }


    void DominantPlane::FaceSegmentation(Plane_model plane, cv::Mat &FirstLevel, cv::Mat &SecondLevel, cv::Mat &ThirdLevel, cv::Mat &FirstFace, cv::Mat &SecondFace, cv::Mat &ThirdFace)
    {
        cv::Mat pcd_object = cv::Mat::zeros(height, width, CV_32FC3);
        float Threshold = 0.015;
        float grid_length = 0.025;
        for (int i=0; i<pcd_object.rows; i++)
        {
            for(int j=0; j<pcd_object.cols; j++)
            {
                float x = Pointcloud.at<cv::Vec3f>(i, j)[0];
                float y = Pointcloud.at<cv::Vec3f>(i, j)[1];
                float z = Pointcloud.at<cv::Vec3f>(i, j)[2];
                float dist = abs(plane.a * x + plane.b * y + plane.c * z + plane.d) / plane.denominator;

                if ( -Threshold + grid_length < dist && dist < Threshold + grid_length) // first level
                    FirstLevel.at<cv::Vec3f>(i, j)[0] = 3.0;
                if ( -Threshold + 2 * grid_length < dist && dist < Threshold + 2 * grid_length) // second level
                    SecondLevel.at<cv::Vec3f>(i, j)[0] = 3.0;
                if ( -Threshold + 3 * grid_length < dist && dist < Threshold + 3 * grid_length) // third level
                    ThirdLevel.at<cv::Vec3f>(i, j)[0] = 3.0;
                if ( Threshold < dist && dist < grid_length - Threshold) // first face
                    FirstFace.at<cv::Vec3f>(i, j)[0] = 3.0;
                if ( Threshold + grid_length < dist && dist < - Threshold + 2 * grid_length) // second face
                    SecondFace.at<cv::Vec3f>(i, j)[0] = 3.0;
                if ( Threshold + 2 * grid_length < dist && dist < - Threshold + 3 * grid_length) // third face
                    ThirdFace. at<cv::Vec3f>(i, j)[0] = 3.0;
            }

        }
    }


//    end = clock();
//    double result = (double)(end - start)/CLOCKS_PER_SEC;

//    cout << result << endl;
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
        for (int y=0; y<pcd_input.rows; y++)
        {
            for(int x = 0; x < pcd_input.cols; x++)
            {
                cv::Vec3f temp = pcd_input.at<cv::Vec3f>(y,x);
                float dist = abs(plane.a * temp[0] + plane.b * temp[1] + plane.c * temp[2] + plane.d) / plane.denominator;
                // cout << temp[0] << " " << temp[1] << " " << temp[2] << endl;
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
Plane::Plane_model Segmentation(cv::Mat& image_RGB, cv::Mat& image_Depth, cv::Mat& FirstLevel, cv::Mat& SecondLevel, cv::Mat& ThirdLevel, cv::Mat& FirstFace, cv::Mat& SecondFace, cv::Mat& ThirdFace)
{
    float fx = 615.6707153320312;
    //float fx = 520.9;
    float fy = 615.962158203125;
    //float fy = 521.0;
    float cx = 328.0010681152344;
    //float cx = 325.1;
    float cy = 241.31031799316406;
    //float cy = 249.7;
    float scale = 1000;
    float Distance_theshold = 0.005;
    int width = 640;
    int height = 480;
    int max_iter = 300;
    Plane::DominantPlane plane(fx,fy,cx,cy, scale, Distance_theshold, max_iter, width, height);

    clock_t start, end;

    cv::Mat pCloud(height, width, CV_32FC3);
    cv::Mat pCloud_inlier(height, width, CV_32FC3);

    start = clock();
    cv::Mat pointCloud = plane.Depth2pcd(image_Depth);

    cv::Mat pcd_outlier = cv::Mat::zeros(height, width, CV_32FC3);
    Plane::Plane_model best_plane = plane.RunRansac(pCloud_inlier);

    cv::Mat pCloud_outlier = cv::Mat::zeros(height, width, CV_32FC3);

    for (int y=0; y<height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            pCloud_outlier.at<cv::Vec3f>(y,x) = pointCloud.at<cv::Vec3f>(y,x) - pCloud_inlier.at<cv::Vec3f>(y,x);
        }
    }

    cv::Mat pcd_object = cv::Mat::zeros(height, width, CV_32FC3);

    plane.ObjectSegmentation(best_plane, pcd_object);

    if (Visualize_Flag)
        Show_Results(pcd_object, image_RGB, "Dominant Plane");
    end = clock();

    double result = (double)(end - start)/CLOCKS_PER_SEC;

    //cout<< result << endl;

    /*
    float grid_length = 0.025;
    cv::Mat FirstLevel = plane.DistantPlaneSegmentation(best_plane, grid_length);
    Show_Results(FirstLevel, image_RGB, "First Level");
    cv::Mat SecondLevel = plane.DistantPlaneSegmentation(best_plane, 2.0*grid_length);
    Show_Results(SecondLevel, image_RGB, "Second Level");
    cv::Mat ThirdLevel = plane.DistantPlaneSegmentation(best_plane, 3.0*grid_length);
    Show_Results(ThirdLevel, image_RGB, "Third Level");
    */
    /*
    cv::Mat FirstLevel = cv::Mat::zeros(height, width, CV_32FC3);
    cv::Mat SecondLevel = cv::Mat::zeros(height, width, CV_32FC3);
    cv::Mat ThirdLevel = cv::Mat::zeros(height, width, CV_32FC3);
    cv::Mat FirstFace = cv::Mat::zeros(height, width, CV_32FC3);
    cv::Mat SecondFace = cv::Mat::zeros(height, width, CV_32FC3);
    cv::Mat ThirdFace = cv::Mat::zeros(height, width, CV_32FC3);
    */
    plane.FaceSegmentation(best_plane, FirstLevel, SecondLevel, ThirdLevel, FirstFace, SecondFace, ThirdFace);
    //Show_Results(FirstLevel, image_RGB, "First Level");
    //Show_Results(SecondLevel, image_RGB, "Second Level");
    //Show_Results(ThirdLevel, image_RGB, "Third Level");
    //Show_Results(FirstFace, image_RGB, "First Face");
    //Show_Results(SecondFace, image_RGB, "Second Face");
    //Show_Results(ThirdFace, image_RGB, "Third Face");
    return best_plane;
}

void Show_Results(cv::Mat& pointCloud, cv::Mat RGB_image_original, std::string window_name)
{
    cv::Mat RGB_image = RGB_image_original.clone();

    for (int y = 0; y < RGB_image.rows; y++)
    {
        for(int x = 0; x < RGB_image.cols; x++) {

            if (pointCloud.at<cv::Vec3f>(y, x)[0] == 0 && pointCloud.at<cv::Vec3f>(y, x)[1] == 0 && pointCloud.at<cv::Vec3f>(y, x)[2] == 0) {
                //cout << " I'm here~ " << endl;
                RGB_image.at<Vec3b>(y, x)[0] = 0;
                //cout << RGB_image.at<Vec3b>(y, x)[0] << RGB_image.at<Vec3b>(y, x)[1] << endl;
                RGB_image.at<Vec3b>(y, x)[1] = 0;
                RGB_image.at<Vec3b>(y, x)[2] = 0;
            }
        }

    }


    cv::Mat red_image;
    red_image = imageCb(RGB_image);

    // namedWindow( "Point_cloud", WINDOW_NORMAL);
    // imshow("Point_cloud", pCloud_inlier);

    //namedWindow("RGB_image_seg", WINDOW_NORMAL );
    //imshow("RGB_image_seg", RGB_image);
    cv::imshow(window_name, RGB_image);
    waitKey(2);

    cv::Mat pointCloud_output;
    pointCloud_output = RGB2pcl(red_image, pointCloud);

    //registration(pointCloud_output);

}

/*
void Show_Results_2(cv::Mat RGB_seg_image, cv::Mat RGB_image_original)
{
    cv::Mat RGB_image = RGB_image_original.clone();

    for (int y = 0; y < RGB_image.rows; y++)
    {
        for(int x = 0; x < RGB_image.cols; x++) {

            if (RGB_seg_image.at<Vec3b>(y, x)[0] == 0 && RGB_seg_image.at<Vec3b>(y, x)[1] == 0 && RGB_seg_image.at<Vec3b>(y, x)[2] == 0) {
            }
            else{
                RGB_image.at<Vec3b>(y, x)[0] = 0;
                //cout << RGB_image.at<Vec3b>(y, x)[0] << RGB_image.at<Vec3b>(y, x)[1] << endl;
                RGB_image.at<Vec3b>(y, x)[1] = 0;
                RGB_image.at<Vec3b>(y, x)[2] = 0;
            }
        }

    }

    namedWindow( "RGB_image_2", WINDOW_AUTOSIZE );
    imshow("RGB_image_2", RGB_image);
    waitKey(2);

}
*/

cv::Mat RGB2pcl(cv::Mat red_image, cv::Mat pointCloud)
{
    cv::Mat pointCloud_output = pointCloud.clone();

    for (int y = 0; y < red_image.rows; y++)
    {
        for(int x = 0; x < red_image.cols; x++) {

            if (red_image.at<uchar>(y, x) == 0) {
                pointCloud_output.at<cv::Vec3f>(y, x)[0] = 0;
                pointCloud_output.at<cv::Vec3f>(y, x)[1] = 0;
                pointCloud_output.at<cv::Vec3f>(y, x)[2] = 0;
            }
        }

    }

    //cv::imshow("???", pointCloud_output); // change the color
    //cv::waitKeyEx(2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = MatToPoinXYZ(pointCloud_output);

    pcl::io::savePLYFileASCII ("test_pcd.ply", *cloud);

    return pointCloud_output;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat OpencVPointCloud)
         {
             /*
             *  Function: Get from a Mat to pcl pointcloud datatype
             *  In: cv::Mat
             *  Out: pcl::PointCloud
             */

             //char pr=100, pg=100, pb=100;
             pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);//(new pcl::pointcloud<pcl::pointXYZ>);

             for(int i=0;i<OpencVPointCloud.cols;i++)
             {
                 for(int j=0; j<OpencVPointCloud.rows; j++)
                 {
                     pcl::PointXYZ point;
                     point.x = OpencVPointCloud.at<cv::Vec3f>(j,i)[0];
                     point.y = OpencVPointCloud.at<cv::Vec3f>(j,i)[1];
                     point.z = OpencVPointCloud.at<cv::Vec3f>(j,i)[2];

                     point_cloud_ptr -> points.push_back(point);

                 }
                //std::cout<<i<<endl;


                // when color needs to be added:
                //uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                //point.rgb = *reinterpret_cast<float*>(&rgb);

             }
             // point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
             // point_cloud_ptr->height = 1;

             return point_cloud_ptr;

         }




void registration(cv::Mat& pointCloud_output)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in = MatToPoinXYZ(pointCloud_output);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile<pcl::PointXYZ> ("test_ply.ply", *cloud_out);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; // RGB?

    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    //std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    //std::cout << icp.getFinalTransformation() << std::endl;

}

cv::Mat imageCb(cv::Mat& RGB_image)
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


    //cv::imshow("red mask", red); // change the color
    //cv::waitKeyEx(2);

    return red;
}
























