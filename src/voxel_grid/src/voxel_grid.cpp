#include "voxel_grid.h"
#include "config.h"
#include "voxel_grid/voxel_grid.h"

int number = 0;

using namespace cv;
using namespace cv::line_descriptor;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
clock_t START, END;
cv::Mat imRGB, imDepth, colorSeg, normalMap, objCloud, redMask, orangeMask, yellowMask, greenMask, blueMask, purpleMask, brownMask, Mask;
cv::Mat maskedRGB = cv::Mat(480, 640, CV_8UC3, Scalar(0,0,0));
static int ConsistencyFound = 0;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        //Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        
    }

    ~ImageConverter()
    {
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        START = clock();

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //cv::Mat masked = color_segment(cv_ptr->image);
        //cv::Mat lined = line_segment(masked);
        //cv::imshow(OPENCV_WINDOW, lined);

        maskedRGB = color_segment(cv_ptr->image, redMask, orangeMask, yellowMask, greenMask, blueMask, purpleMask, brownMask, Mask);
        
        if (debug_color)
        {
            imshow("color_segment", maskedRGB);
            imshow("original image", cv_ptr->image);
            cv::waitKey(1);
        }

        imRGB = cv_ptr->image; // copy the image into global variable.

        
        //int count = 0;

        //std::stringstream ss;
        //ss << count++;
        //std::string str = ss.str();
        //cv::imwrite("/home/wonjune/Images/color"+str+".png", cv_ptr->image);
        //printf("color world!");
        
        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

class DepthConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    ros::Publisher pub = nh_.advertise<voxel_grid::voxel_grid>("voxel_grid", 1000); // the last number is buffer size.

    voxel_grid::voxel_grid voxel_grid;


public:
    DepthConverter()
        : it_(nh_)
    {
        //Subscribe to input video feed and publish output video feed
        //image_sub_ = it_.subscribe("/camera/depth/image_rect_raw",1, &DepthConverter::imageCb, this);
        image_sub_ = it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &DepthConverter::imageCb, this); // publish depth image that's aligned to color images. encoding type is TYPE_16UC1.
        image_pub_ = it_.advertise("/image_converter/output_video_depth", 1);

        //cv::namedWindow(OPENCV_WINDOW2);
    }

    ~DepthConverter()
    {
        //cv::destroyWindow(OPENCV_WINDOW2);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        if (debug_color)
            return;
        static int ConsistencyCount = 0;
        if ( ConsistencyCount < ConsistencyThresh )
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); // normal depth encoding
                //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // when given option filters:=colorizer
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            
            // Update GUI Window
            //cv::imshow(OPENCV_WINDOW2, cv_ptr->image);
            //cv::waitKey(3);

            imDepth = cv_ptr->image; // copy the depth map into global variable
            
            int height = 480;
            int width = 640;

            static vector< vector< int >> PreviousGrid ( VoxelGridSize, vector<int> ( VoxelGridSize, 0 ));
            int VoxelGridFilled = 0; // to check if the algorithm outputs voxelgrid

            cv::Mat FirstLevel = cv::Mat::zeros(height, width, CV_32FC3);
            cv::Mat SecondLevel = cv::Mat::zeros(height, width, CV_32FC3);
            cv::Mat ThirdLevel = cv::Mat::zeros(height, width, CV_32FC3);
            cv::Mat FirstFace = cv::Mat::zeros(height, width, CV_32FC3);
            cv::Mat SecondFace = cv::Mat::zeros(height, width, CV_32FC3);
            cv::Mat ThirdFace = cv::Mat::zeros(height, width, CV_32FC3);
            std::vector<KeyLine> keyLines;
            std::vector<KeyLine3D> keyLines3D;

            Plane::Plane_model DominantPlane = Segmentation(imRGB, imDepth, FirstLevel, SecondLevel, ThirdLevel, FirstFace, SecondFace, ThirdFace); 

            // color info processing.
            cv::Mat BlockRegion = FirstLevel + SecondLevel + ThirdLevel;// + FirstFace + SecondFace + ThirdFace;
            cv::cvtColor( BlockRegion, BlockRegion, CV_RGB2GRAY);
            BlockRegion = BlockRegion * 255;
            BlockRegion.convertTo(BlockRegion, CV_8UC1);
            redMask = redMask.mul(BlockRegion);
            orangeMask = orangeMask.mul(BlockRegion);
            yellowMask = yellowMask.mul(BlockRegion);
            greenMask = greenMask.mul(BlockRegion);
            blueMask = blueMask.mul(BlockRegion);
            brownMask = brownMask.mul(BlockRegion);
            purpleMask = purpleMask.mul(BlockRegion);
            
            int redcount = cv::sum( redMask )[0] > 20;
            int orangecount = cv::sum( orangeMask )[0] > 20;
            int yellowcount = cv::sum( yellowMask )[0] > 20;
            int greencount = cv::sum( greenMask )[0] > 20;
            int bluecount = cv::sum( blueMask )[0] > 20;
            int browncount = cv::sum( brownMask )[0] > 20;
            int purplecount = cv::sum( purpleMask )[0] > 20;

            if (Visualize_Flag)
            {
                //cv::imshow("masked_RGB", maskedRGB);
                cv::imshow("redMask", redMask);
                cv::imshow("BlockRegion", BlockRegion);
                cv::waitKey(1);
            } 

            int prevred = 0 ;
            int prevorange  = 0;
            int prevyellow  = 0 ;
            int prevgreen  = 0;
            int prevblue = 0;
            int prevbrown = 0;
            int prevpurple = 0;
            
            // DominantPlane.a : a of the plane eqn.  ax + by +cz + d = 0
            float Z_x = DominantPlane.a/DominantPlane.denominator;
            float Z_y = DominantPlane.b/DominantPlane.denominator;
            float Z_d = DominantPlane.c/DominantPlane.denominator;
            float Norm = sqrt(Z_x*Z_x + Z_y*Z_y + Z_d*Z_d + 1e-10); // normalizing for sanity
            
            cv::Point3f Initial_Z = cv::Point3f(Z_x/Norm, Z_y/Norm, Z_d/Norm);
            //cout << "DominantPlane.a" << DominantPlane.a << "b" << DominantPlane.b << "c" << DominantPlane.c << "d" << DominantPlane.d << endl;
            //cout << "DominantPlane.denominator" << DominantPlane.denominator << endl;
            //cout << "normalized InitialZ norm: "<< sqrt(Initial_Z.x*Initial_Z.x + Initial_Z.y*Initial_Z.y + Initial_Z.z*Initial_Z.z) << endl;

            cv::Mat VoidMask = cv::Mat::ones( imRGB.size(), CV_8UC1);
            
            Get_keyLines(keyLines, imRGB, VoidMask);
            cv::Mat line_segmented_image = line_visualize(keyLines, imRGB);
            if (Visualize_Flag)
            {
                //cv::imshow(OPENCV_WINDOW, line_segmented_image);
                //cv::waitKey(1);
                cv::imshow("original image", imRGB);
                cv::waitKey(1);
            }
            int keyLinesSize = (int)keyLines.size();
            keyLines3D.reserve(sizeof(KeyLine3D) * keyLines.size() );
            Get3DLines(keyLines, imDepth, keyLines3D);
            // access 3D line points : keyLines3D[i].startPointX
            std::vector<cv::Point3f> GRIDXYZ;
            GRIDXYZ.reserve(sizeof(cv::Point3f) * 3 );
            cv::Point3f Origin;
            int Grid_found = GetGridXYZ(keyLines3D, Initial_Z, GRIDXYZ, Origin);
            // access grid X vector's x element : GRIDXYZ[0].x, access origin's x element: Origin.x
            /*
            if (use_color)
                vector<vector<vector<int> > > VoxelGrid ( VoxelGridSize, vector<vector<int> > (VoxelGridSize, vector<int> (BlockHeight,0)));    
            */
            vector<vector<int>> VoxelGrid ( VoxelGridSize, vector<int> ( VoxelGridSize, 0 ));

            if (Grid_found)
            {
                if(Visualize_Flag)
                    OriginVisualize(Origin, GRIDXYZ, imRGB);

                
                //VoxelGridVisualize(VoxelGrid, Origin, GRIDXYZ, imRGB); // don't uncomment this.
                int minX=VoxelGridSize-1;int maxX=0;int minY=VoxelGridSize-1; int maxY=0;
                int VoxelGridSizeX = VoxelGridSize; int VoxelGridSizeY = VoxelGridSize;
                VoxelGridFilled = FillVoxelGrid(VoxelGrid, Origin, GRIDXYZ, DominantPlane, FirstLevel, SecondLevel, ThirdLevel, redMask, orangeMask, yellowMask, greenMask, blueMask, brownMask, purpleMask, maskedRGB, maxX, maxY, minX, minY, VoxelGridSizeX, VoxelGridSizeY); // zero means filled, -1 means not.

                vector<vector<int>> PostProcessed ( VoxelGridSizeY, vector<int> ( VoxelGridSizeX, 0 ));
                vector<vector<int>> ConsistentGrid ( VoxelGridSize, vector<int> ( VoxelGridSize, 0 ));
                for ( int y = 0; y <= maxY - minY ; y ++) 
                {
                    for ( int x = 0; x <= maxX - minX ; x++)
                    {
                        PostProcessed[y][x] = VoxelGrid[y + minY][x + minX];
                        ConsistentGrid[y][x] = VoxelGrid[y + minY][x + minX];
                    }
                }

                //if ( (ConsistentGrid == PreviousGrid) && (redcount == prevred) && (orangecount == prevorange) && (yellowcount == prevyellow) && (greencount == prevgreen) && (bluecount == prevblue) && (browncount == prevbrown) && (purplecount == prevpurple) )
                if ( ConsistentGrid == PreviousGrid )
                    ConsistencyCount ++;

                else
                    ConsistencyCount = 0;

                PreviousGrid = ConsistentGrid;
                prevred = redcount;
                prevorange = orangecount;
                prevyellow = yellowcount;
                prevgreen = greencount;
                prevblue = bluecount;
                prevbrown = browncount;
                prevpurple = purplecount;
                // Output modified video streams
                //image_pub_.publish(cv_ptr->toImageMsg());
                voxel_grid.voxel_grid.clear();
                voxel_grid.colors.clear();
                if (VoxelGridFilled)
                {
                    for (int y = 0; y < VoxelGridSizeY; y ++ )
                    {
                        for (int x = 0; x < VoxelGridSizeX; x ++ )
                        {
                            voxel_grid.voxel_grid.push_back(PostProcessed[y][x]);
                        }
                    }
                    voxel_grid.colors.push_back(redcount);
                    voxel_grid.colors.push_back(orangecount);
                    voxel_grid.colors.push_back(yellowcount);
                    voxel_grid.colors.push_back(greencount);
                    voxel_grid.colors.push_back(bluecount);
                    voxel_grid.colors.push_back(browncount);
                    voxel_grid.colors.push_back(purplecount);
                }

                voxel_grid.VoxelGridSizeX = VoxelGridSizeX;
                voxel_grid.VoxelGridSizeY = VoxelGridSizeY;
                
            }
        //voxel_grid.VoxelGridFilled = VoxelGridFilled;
        }
        else // if consistencty is established,
        {
            ConsistencyFound = 1;
            voxel_grid.VoxelGridFound = ConsistencyFound;
            pub.publish(voxel_grid);
        }

        END = clock();
        cout << "Voxel Grid FPS: " << 1.f / ((float)(END-START)/CLOCKS_PER_SEC) << endl;
    }
};
/*
void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Color image height = %d, width = %d", msg->height, msg->width);

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Get_RGB(cv_ptrRGB);

        cv::Mat maskedRGB = color_segment(imRGB, redMask, orangeMask, yellowMask, greenMask, blueMask, purpleMask, brownMask, Mask);
        Get_keyLines(keyLines, imRGB, redMask);
        cv::Mat line_segmented_image = line_visualize(keyLines, imRGB);
        cv::imshow("line_segmented_image", line_segmented_image);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void DepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    number++;
    ROS_INFO("Depth image height = %d, width = %d", msg->height, msg->width);
    
    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        Get_Depth(cv_ptrD);
        if (imRGB.empty() || imDepth.empty())
        {
            cerr << endl << "Failed to load image at: " << endl;
            return;
        }
        else if (number%50 == 0)
        {
            Segmentation(imRGB, imDepth); // dominant plane segmentation

            number = 0;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
}
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");

    ImageConverter ic;
    DepthConverter dc;
    //ros::NodeHandle nh;
    //ros::Subscriber sub_color = nh.subscribe("/camera/color/image_raw", 100, ImageCallback);
    //ros::Subscriber sub_depth = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 100, DepthCallback);
    //Segmentation(imRGB, imDepth);

    ros::spin();
    return 0;
    
}
/*
void Get_RGB (cv_bridge::CvImageConstPtr& cv_ptr)
{
    imRGB = cv_ptr->image;
    //cout << "row of Color image" << imRGB.rows << endl;
}

void Get_Depth (cv_bridge::CvImageConstPtr& cv_ptr)
{
    imDepth = cv_ptr->image;
    //cout << "row of Depth image" << imDepth.rows << endl;
}

void Get_pCloud (cv_bridge::CvImagePtr& cv_ptr)
{
    pointCloud = cv_ptr->image;
    cout << "row of Cloud image" << pointCloud.rows << endl;
}
*/
int FillVoxelGrid(vector<vector<int> > &VoxelGrid, cv::Point3f Origin, std::vector<cv::Point3f> GRIDXYZ, Plane::Plane_model DominantPlane, cv::Mat FirstLevel, cv::Mat SecondLevel, cv::Mat ThirdLevel, cv::Mat redmask, cv::Mat orangemask, cv::Mat yellowmask, cv::Mat greenmask, cv::Mat bluemask, cv::Mat brownmask, cv::Mat purplemask, cv::Mat maskedRGB, int& maxX, int& maxY, int& minX, int& minY, int& VoxelGridSizeX, int& VoxelGridSizeY)
{
    //int size[] = {VoxelGridSize+10, VoxelGridSize+10, 8+10}; //all the +10s are just to prevent core dump.
    vector<vector<vector<vector<int> > > > VoxelGridTemp ( VoxelGridSize, vector<vector<vector<int> > > (VoxelGridSize, vector< vector<int> > (NumColor, vector<int> (BlockHeight,0))));
    //cv::Mat VoxelGridTemp(3, size, CV_16UC3, Scalar(0,0,0)); // This is a temp Mat. the last dimension is to be argmaxed later.
    //int size2[] = {VoxelGridSize+10, VoxelGridSize+10};
    //cv::Mat VoxelGrid(2, size2, CV_16UC3, Scalar(0,0,0));
    
    //int size3[] = {VoxelGridSize+10, VoxelGridSize+10, 3+10};
    //cv::Mat VoxelGridPosition(3, size3, CV_16UC2, Scalar(0,0));
    vector<vector<vector<vector<int> > > > VoxelGridPosition ( VoxelGridSize, vector<vector<vector<int > > > (VoxelGridSize, vector< vector<int > > (BlockHeight, vector<int> (2,0))));

    int patch = 7; // patch to look at from the centroid
    int redCount = 0; int orangeCount = 0; int yellowCount = 0; int greenCount = 0; int blueCount = 0; int purpleCount = 0; int brownCount = 0;
    
    // First, set the origin's level to be zero.
    float Thresh = 0.003;
    float block_size = 0.025;
    int Arg = 0;
    int Max = 0;
    float UpperVecX = DominantPlane.a;
    float UpperVecY = DominantPlane.b;
    float UpperVecZ = DominantPlane.c;
    if ( DominantPlane.c > 0 )
    {
        UpperVecX = -DominantPlane.a;
        UpperVecY = -DominantPlane.b;
        UpperVecZ = -DominantPlane.c;
    }
    float normalizer = DominantPlane.denominator / block_size; 
    cv::Point3f GridZVec = cv::Point3f(UpperVecX / normalizer, UpperVecY / normalizer, UpperVecZ / normalizer );

    normalizer = sqrt(GRIDXYZ[0].x*GRIDXYZ[0].x + GRIDXYZ[0].y*GRIDXYZ[0].y + GRIDXYZ[0].z*GRIDXYZ[0].z) / block_size;
    cv::Point3f GridXVec = cv::Point3f(GRIDXYZ[0].x/normalizer, GRIDXYZ[0].y/normalizer, GRIDXYZ[0].z/normalizer);
    normalizer = sqrt(GRIDXYZ[1].x*GRIDXYZ[1].x + GRIDXYZ[1].y*GRIDXYZ[1].y + GRIDXYZ[1].z*GRIDXYZ[1].z) / block_size;
    cv::Point3f GridYVec = cv::Point3f(GRIDXYZ[1].x/normalizer, GRIDXYZ[1].y/normalizer, GRIDXYZ[1].z/normalizer);
    float distance = ( DominantPlane.a * Origin.x + DominantPlane.b * Origin.y + DominantPlane.c * Origin.z + DominantPlane.d ) / DominantPlane.denominator;
    /*
    float normalizer_x = DominantPlane.a/DominantPlane.denominator;
    float normalizer_y = DominantPlane.b/DominantPlane.denominator;
    float normalizer_z = DominantPlane.c/DominantPlane.denominator;
    float temp_x = Origin.x - distance*normalizer_x; 
    float temp_y = Origin.y - distance*normalizer_y; 
    float temp_z = Origin.z - distance*normalizer_z; 
    cout << "temp x: " << temp_x << endl;
    cout << "temp y: " << temp_y << endl;
    cout << "temp_z: " << temp_z << endl;
    float distance_temp = abs( DominantPlane.a * temp_x + DominantPlane.b * temp_y + DominantPlane.c * temp_z + DominantPlane.d ) / DominantPlane.denominator;
    cout << "distance " << distance << endl;
    cout << "distance_temp" << distance_temp << endl;
    */ // These codes project the origin onto the plane directly.

    // Set the origin on the level of doiminant plane.
    if ( abs(abs(distance) - block_size) < Thresh )
        Origin = cv::Point3f(Origin.x - GridZVec.x, Origin.y - GridZVec.y, Origin.z - GridZVec.z);
    else if ( abs(abs(distance) - 2*block_size) < Thresh )
        Origin = cv::Point3f(Origin.x - 2*GridZVec.x, Origin.y - 2*GridZVec.y, Origin.z - 2*GridZVec.z);
    else if ( abs(abs(distance) - 3*block_size) < Thresh )
        Origin = cv::Point3f(Origin.x - 3*GridZVec.x, Origin.y - 3*GridZVec.y, Origin.z - 3*GridZVec.z);

    // make the grid vectors consistent (or aligned) with camera global coordinate.
    if ( abs( GridXVec.x ) > abs( GridYVec.x) )
    {
        if ( GridXVec.x < 0 )
            GridXVec = Point3f( -GridXVec.x, -GridXVec.y, -GridXVec.z );
        if ( GridYVec.y < 0 )
            GridYVec = Point3f( -GridYVec.x, -GridYVec.y, -GridYVec.z );
    }
    else // X and Y should be changed
    {
        Point3f GridXVec_temp;
        if ( GridYVec.x > 0 )
            GridXVec_temp = Point3f( GridYVec.x, GridYVec.y, GridYVec.z );
        else
            GridXVec_temp = Point3f( -GridYVec.x, -GridYVec.y, -GridYVec.z );
        if ( GridXVec.y > 0 )
            GridYVec = Point3f( GridXVec.x, GridXVec.y, GridXVec.z );
        else
            GridYVec = Point3f( -GridXVec.x, -GridXVec.y, -GridXVec.z );
        GridXVec = Point3f( GridXVec_temp.x, GridXVec_temp.y, GridXVec_temp.z );
    }
    /*
    // Set the right-hand coordinate by switching X and Y if needed.
    cv::Vec3f linex = cv::Vec3f(GridXVec.x, GridXVec.y, GridXVec.z);
    cv::Vec3f liney = cv::Vec3f(GridYVec.x, GridYVec.y, GridYVec.z);
    cv::Vec3f cross = (linex.cross(liney));
    float innerProd = cross[0] * GridZVec.x + cross[1] * GridZVec.y + cross[2] * GridZVec.z;
    if (innerProd < 0)
    {
        GridYVec = cv::Point3f( linex[0], linex[1], linex[2] );
        GridXVec = cv::Point3f( liney[0], liney[1], liney[2] );
    }   
    */

    distance = abs( DominantPlane.a * Origin.x + DominantPlane.b * Origin.y + DominantPlane.c * Origin.z + DominantPlane.d ) / DominantPlane.denominator;
    if (Visualize_Flag)
        cout << "Distance from origin to Plane: " << distance << endl;// this is sometimes too big. even though the above code is working. IDK Y. for now, proceed only if this is whithin the Thresh.. 
    if ( distance > Thresh ) 
        return 0;

    // For each grid X, Y, Z, calculate its corresponding position on the image..
    cv::Mat GridPosition = maskedRGB.clone();
    cv::Mat GridPosition2 = maskedRGB.clone();
    cv::Mat GridPosition3 = maskedRGB.clone();
    for ( int col = 0; col < VoxelGridSize; col++)
    {
        for ( int row = 0; row < VoxelGridSize; row++)
        {
            for ( int h = 0; h < 3; h ++)
            {
                int temp = VoxelGridPosition[(int)(VoxelGridSize/2)][(int)(VoxelGridSize/2)][0][1] ;
                cv::Point2d Projected = Project2Img(cv::Point3f(Origin.x + (col-(int)(VoxelGridSize/2))*GridXVec.x + (row-(int)(VoxelGridSize/2))*GridYVec.x + (h+1)*GridZVec.x, Origin.y + (col-(int)(VoxelGridSize/2))*GridXVec.y + (row-(int)(VoxelGridSize/2))*GridYVec.y + (h+1)*GridZVec.y, Origin.z + (col-(int)(VoxelGridSize/2))*GridXVec.z + (row-(int)(VoxelGridSize/2))*GridYVec.z + (h+1)*GridZVec.z));
                if (Projected.x > maskedRGB.cols - 1) Projected.x = maskedRGB.cols - 1;
                if (Projected.x < 0) Projected.x = 0;
                if (Projected.y > maskedRGB.rows - 1) {Projected.y = maskedRGB.rows - 1; }
                if (Projected.y < 0) Projected.y = 0;
                
                VoxelGridPosition[row][col][h][0] = (int)Projected.x;
                VoxelGridPosition[row][col][h][1] = (int)Projected.y; 
                if (h == 0)
                    if ( col == (int)(VoxelGridSize/2) && row == (int)(VoxelGridSize/2))
                        circle( GridPosition, Projected, 4, Scalar(255,255,255),4);
                    else
                        circle( GridPosition, Projected,2, Scalar(255,255,255),2);
                else if (h==1)
                    circle(GridPosition2, Projected, 2, Scalar(0,255,0), 2);
                else if (h==2)
                    circle(GridPosition3, Projected, 2, Scalar(0,0,225), 2);
                //if ( temp != VoxelGridPosition[20][20][0][1] )
                    //cout << "temp changed! " << "col" << col << " row" << row << " h" << h << " val" << VoxelGridPosition[20][20][0][1] << (int)Projected.y << endl;
            }
        }
    }

    if(Visualize_Flag)
    {
        imshow("GridPosition_firstlevel", GridPosition);
        
        //imshow("GridPosition_secondlevel", GridPosition2);
        //imshow("GridPosition_thirdlevel", GridPosition3);
        cv::waitKey(1);
    }

    //cout << Origin.x + GridZVec.x << Origin.y + GridZVec.y << Origin.z + GridZVec.z << endl;
    //cv::Point2d Projected = Project2Img(cv::Point3f(Origin.x+GridZVec.x, Origin.y+GridZVec.y, Origin.z+GridZVec.z));
    //cout << (int)Projected.y<<endl;
    //cout << VoxelGridPosition[20][20][0][0] << endl;
    //cout << VoxelGridPosition[20][20][0][1] << endl;
    //cout << VoxelGridPosition[20][20][0][1] << (int)Projected.y << endl; // this gets corrupted ..??

    /*
    // For each pixel, check where it belongs.
    
    for ( int col = 0; col < maskedRGB.cols; col++)
    {
        for ( int row = 0; row < maskedRGB.rows; row ++)
        {
            int GridX = -1;
            int GridY = -1;

            if ( FirstLevel.at<Vec3b>(row,col)[0] > 0 || FirstLevel.at<Vec3b>(row,col)[1] > 0 || FirstLevel.at<Vec3b>(row,col)[2] > 0 )
            {
                int level = 0;
                GetGridPos(VoxelGridPosition, VoxelGridSize, col, row, level, GridX, GridY); // the position in the grid of current pixel..
                if (GridX < 0 || GridY < 0) continue;

                if ( redmask.at<int>(row,col) > 0 ) {VoxelGridTemp[GridY][GridX][1][0] ++; cout<<"X"<<GridX<<"Y"<<GridY<<endl;}
                else if (orangemask.at<int>(row,col) > 0 ) VoxelGridTemp[GridY][GridX][2][0]++;
                else if (yellowmask.at<int>(row,col) > 0 ) VoxelGridTemp[GridY][GridX][3][0]++;
                else if (greenmask.at<int>(row,col) > 0 ) VoxelGridTemp[GridY][GridX][4][0]++;
                else if (bluemask.at<int>(row, col) > 0 ) VoxelGridTemp[GridY][GridX][5][0]++;
                else if (purplemask.at<int>(row, col) > 0) VoxelGridTemp[GridY][GridX][6][0]++;
                else if (brownmask.at<int>(row, col) > 0 ) VoxelGridTemp[GridY][GridX][7][0]++;
            }
            else if ( SecondLevel.at<Vec3b>(row,col)[0] > 0 || SecondLevel.at<Vec3i>(row,col)[1] > 0 || SecondLevel.at<Vec3b>(row,col)[2] > 0)
            {
                int level = 1;
                
                GetGridPos(VoxelGridPosition, VoxelGridSize, col, row, level, GridX, GridY);
                if (GridX < 0 || GridY < 0) continue;

                if ( redmask.at<int>(row,col) > 0)  VoxelGridTemp[GridY][GridX][1][1] ++;
                else if (orangemask.at<int>(row,col) > 0 ) VoxelGridTemp[GridY][GridX][2][1]++;
                else if (yellowmask.at<int>(row,col) > 0 ) VoxelGridTemp[GridY][GridX][3][1]++;
                else if (greenmask.at<int>(row,col) > 0 ) VoxelGridTemp[GridY][GridX][4][1]++;
                else if (bluemask.at<int>(row, col) > 0 ) VoxelGridTemp[GridY][GridX][5][1]++;
                else if (purplemask.at<int>(row, col) > 0) VoxelGridTemp[GridY][GridX][6][1]++;
                else if (brownmask.at<int>(row, col) > 0 ) VoxelGridTemp[GridY][GridX][7][1]++;
            }
            else if ( ThirdLevel.at<Vec3b>(row,col)[0] > 0 || ThirdLevel.at<Vec3i>(row,col)[1]>0 || ThirdLevel.at<Vec3b>(row,col)[2]>0)
            {
                int level = 2;
                
                GetGridPos(VoxelGridPosition, VoxelGridSize, col, row, level, GridX, GridY);
                if (GridX < 0 || GridY < 0) continue;

                if ( redmask.at<int>(row,col) > 0 )  VoxelGridTemp[GridY][GridX][1][2] ++;
                else if (orangemask.at<int>(row,col) > 0 ) VoxelGridTemp[GridY][GridX][2][2]++;
                else if (yellowmask.at<int>(row,col) > 0 ) VoxelGridTemp[GridY][GridX][3][2]++;
                else if (greenmask.at<int>(row,col) > 0 ) VoxelGridTemp[GridY][GridX][4][2]++;
                else if (bluemask.at<int>(row, col) > 0 ) VoxelGridTemp[GridY][GridX][5][2]++;
                else if (purplemask.at<int>(row, col) > 0) VoxelGridTemp[GridY][GridX][6][2]++;
                else if (brownmask.at<int>(row, col) > 0 ) VoxelGridTemp[GridY][GridX][7][2]++;
           }
            else continue;
        }
    }
    //cout<< "asfdd"<<(VoxelGridTemp.at<Vec3b>(21,21,0)[2] < 1) << endl;  // works
    //cout << "ASDF" << (int)redmask.at<int>(200,200) << endl;

    // Take ArgMax for each voxel grid cell..
    for ( int col = 0; col < VoxelGridSize; col++)
    {
        for ( int row = 0; row < VoxelGridSize; row++)
        {
            for ( int h = 0; h < 3; h++)
            {
                //cout << "asdfsdfas" << (int)VoxelGridTemp.at<Vec3b>(row,col,1)[h] << endl; // no value is here..
                if (VoxelGridTemp[row][col][1][h] > Max){Max = VoxelGridTemp[row][col][1][h]; Arg = 1;}
                if (VoxelGridTemp[row][col][2][h] > Max){Max = VoxelGridTemp[row][col][2][h]; Arg = 2;}
                if (VoxelGridTemp[row][col][3][h] > Max){Max = VoxelGridTemp[row][col][3][h]; Arg = 3;}
                if (VoxelGridTemp[row][col][4][h] > Max){Max = VoxelGridTemp[row][col][4][h]; Arg = 4;}
                if (VoxelGridTemp[row][col][5][h] > Max){Max = VoxelGridTemp[row][col][5][h]; Arg = 5;}
                if (VoxelGridTemp[row][col][6][h] > Max){Max = VoxelGridTemp[row][col][6][h]; Arg = 6;}
                if (VoxelGridTemp[row][col][7][h] > Max){Max = VoxelGridTemp[row][col][7][h]; Arg = 7;}

                VoxelGrid[row][col][h] = Arg;
                Max, Arg = 0, 0;
            }
        }
    }
    
    cout << VoxelGrid[20][20][0] << endl;
    */

    //cout << VoxelGrid << endl;

    // Then, for each grid, check if anything exists....
    for (int x=0; x<VoxelGridSize; x++) // the origin will lie at (20,20)
    {
        for (int y=0; y<VoxelGridSize; y++)
        {
            for (int z=0; z<BlockHeight; z++)
            {
                //get the 2D mask of current block.. the 2D current block is defined as: [x,x+1], [y,y+1], [z+1]
                cv::Point3f StartPoint = cv::Point3f(Origin.x + (x-(int)(VoxelGridSize/2))*GridXVec.x + (y-(int)(VoxelGridSize/2))*GridYVec.x+ (z+1)*GridZVec.x, Origin.y +(x-(int)(VoxelGridSize/2))*GridXVec.y + (y-(int)(VoxelGridSize/2))*GridYVec.y + (z+1)*GridZVec.y, Origin.z + (x-(int)(VoxelGridSize/2))*GridXVec.z + (y-(int)(VoxelGridSize/2))*GridYVec.z + (z+1)*GridZVec.z);
                cv::Point3f EndPointX = cv::Point3f(StartPoint.x + GridXVec.x, StartPoint.y + GridXVec.y, StartPoint.z + GridXVec.z);
                cv::Point3f EndPointY = cv::Point3f(StartPoint.x + GridYVec.x, StartPoint.y + GridYVec.y, StartPoint.z + GridYVec.z);
                cv::Point3f EndPointXY = cv::Point3f(EndPointX.x + GridYVec.x, EndPointX.y + GridYVec.y, EndPointX.z + GridYVec.z);

                cv::Point2d start = Project2Img(StartPoint);
                cv::Point2d endx = Project2Img(EndPointX);
                cv::Point2d endy = Project2Img(EndPointY);
                cv::Point2d endxy = Project2Img(EndPointXY);

                cv::Point2i Centroid = cv::Point2i( (int)((start.x + endx.x + endy.x + endxy.x)/4), (int)((start.y + endx.y + endy.y + endxy.y)/4) );
                //cout << maskedRGB.at<Vec3b>(240,320) <<  (maskedRGB.at<Vec3b>(240,320) == Vec3b(255,69,54)) << endl; // works
                /*
                if ( x == 20 && y == 20 && z == 0)
                {
                    cout << "Centroid:" << Centroid.x << ", "<< Centroid.y<<endl;
                    cv::Mat Centroidim = maskedRGB.clone();
                    circle( Centroidim, Centroid,2, Scalar(255,0,0),2);
                    imshow("Centroid", Centroidim);
                    cv::waitKey(1);

                cout << "red" << (int)redmask.at<uchar>(Centroid.y, Centroid.x) << endl << "orange" << (int)orangemask.at<uchar>(Centroid.y, Centroid.x)<<endl << "yellow" << (int)yellowmask.at<uchar>(Centroid.y, Centroid.x) <<endl<< "green" << (int)greenmask.at<uchar>(Centroid.y, Centroid.x) << endl<<"blue" << (int)bluemask.at<uchar>(Centroid.y, Centroid.x)<<endl << "purple" << (int)purplemask.at<uchar>(Centroid.y, Centroid.x) << endl<<"brown" << (int)brownmask.at<uchar>(Centroid.y, Centroid.x)<<endl;

                cout << (int)redmask.at<uchar>(186,377)<< (int)redmask.at<uchar>(377,186) << endl;
                }
                    */

                if (Centroid.x < 0 || Centroid.x >= maskedRGB.cols || Centroid.y < 0 || Centroid.y >= maskedRGB.rows) continue;

                for (int col=Centroid.x - patch; col<= Centroid.x + patch; col++)
                {
                    for (int row = Centroid.y - patch; row<= Centroid.y + patch; row++)
                    {

                        if ( col >= maskedRGB.cols || col < 0) continue;
                        if ( row >= maskedRGB.rows || row < 0) continue;
                        if ( z == 0 && FirstLevel.at<Vec3f>(row, col)[0] == 0 && FirstLevel.at<Vec3f>(row,col)[1] == 0 && FirstLevel.at<Vec3f>(row,col)[2] == 0) continue;
                        if ( z == 1 && SecondLevel.at<Vec3f>(row, col)[0] == 0 && SecondLevel.at<Vec3f>(row,col)[1] == 0 && SecondLevel.at<Vec3f>(row,col)[2] == 0) continue;
                        if ( z == 2 && ThirdLevel.at<Vec3f>(row, col)[0] == 0 && ThirdLevel.at<Vec3f>(row, col)[1] == 0 && ThirdLevel.at<Vec3f>(row,col)[2] == 0) continue;

                        if (use_color)
                        {
                            if (redmask.at<uchar>(row,col) > 0 && purplemask.at<uchar>(row,col) == 0 ) redCount++; // redmask is overlapped with purplemask
                            else if (orangemask.at<uchar>(row,col) > 0) orangeCount++;
                            else if (yellowmask.at<uchar>(row,col) > 0) yellowCount++;
                            else if (greenmask.at<uchar>(row,col) > 0) greenCount++;
                            else if (bluemask.at<uchar>(row,col) > 0) blueCount++;
                            else if (purplemask.at<uchar>(row,col) > 0) purpleCount++;
                            else if (brownmask.at<uchar>(row,col) > 0) brownCount++;
                            //cout << redCount << endl; // not working 
                        }
                        else
                        {
                            redCount++;
                        }
                    }
                }



                if (Visualize_Flag)
                {
                    //imshow("redmask", redmask);
                    imshow("first level", FirstLevel);
                    imshow("second level", SecondLevel);
                    //imshow("second level", SecondLevel);
                    imshow("third level", ThirdLevel);
                    cv::waitKey(1);
                }
                //int Index = ArgMax(redCount, orangeCount, yellowCount, greenCount, blueCount, purpleCount, brownCount);
                if (use_color)
                {
                    if (redCount > Max) { Max = redCount; Arg = 1; }
                    if (orangeCount > Max) { Max = orangeCount; Arg = 2; }
                    if (yellowCount > Max) { Max = yellowCount; Arg = 3; }
                    if (greenCount > Max ) { Max = greenCount ; Arg = 4; }
                    if (blueCount > Max ) { Max = blueCount ; Arg = 5; }
                    if (purpleCount > Max ) { Max = purpleCount; Arg = 6; }
                    if (brownCount > Max ) { Max = brownCount ; Arg = 7; }
                }
                else
                {
                    if (redCount>1) // if there is more than 1 filled pixel in the window
                        Arg = 1;
                }

                //cout << "asdfasd" << endl; // this works ..
                //cout << "index: " <<Index << endl; // accessing Index is the reason for core dumped !!
                redCount = 0; orangeCount = 0; yellowCount = 0; greenCount = 0; blueCount = 0; purpleCount = 0; brownCount = 0;
                // made it 100x100 because it seems if I set it to 40x40, the actual memory it sets is 32x32.
                // If I try to access (35,35) and change the value, error occurs. also, there is a trash value in that pixel.
                //VoxelGrid.at<cv::Vec3d>(50,50)[2] = 3;
                //cout << " Mat ::: " << VoxelGrid.at<cv::Vec3d>(50,50)[2]<<endl;
                //cout << "hello" << Arg << endl;
                /*
                if (use_color)
                    VoxelGrid[y][x][z] = Arg;
                */
                if (Arg == 1)
                {
                    VoxelGrid[y][x] = z+1;
                    if (minX > x)
                        minX = x;
                    if (maxX < x)
                        maxX = x;
                    if (minY > y)
                        minY = y;
                    if (maxY < y)
                        maxY = y;
                }
                /*
                if ( y == 20 && x == 20 && z == 0 && Visualize_Flag)
                    cout << VoxelGrid[y][x][z] << endl;
                */
                
                Arg = 0; Max=0;
            }
        }
    }
    //cout<< "Centroid Color" << VoxelGrid[20][20][0] << endl;
    

    /*
    if(Visualize_Flag && use_color)
    {
        // visualize the output... in the most terrible way possible.
        cv::Mat Visualize = imRGB.clone();
        cv::Mat Visualize2 = imRGB.clone();
        cv::Mat Visualize3 = imRGB.clone();
        for (int x = 0; x < VoxelGridSize-1; x++)
        {
            for (int y=0; y < VoxelGridSize-1; y++)
            {
                if (VoxelGrid[y][x][0] == 1)
                    circle(Visualize, cv::Point2i((VoxelGridPosition[y][x][0][0]+VoxelGridPosition[y][x+1][0][0]+VoxelGridPosition[y+1][x][0][0]+VoxelGridPosition[y+1][x+1][0][0])/4, (VoxelGridPosition[y][x][0][1]+VoxelGridPosition[y][x+1][0][1]+VoxelGridPosition[y+1][x][0][1]+VoxelGridPosition[y+1][x+1][0][1])/4), 3, Scalar(0,0,255),3);
                if (VoxelGrid[y][x][0] == 2)
                    circle(Visualize, cv::Point2i((VoxelGridPosition[y][x][0][0]+VoxelGridPosition[y][x+1][0][0]+VoxelGridPosition[y+1][x][0][0]+VoxelGridPosition[y+1][x+1][0][0])/4, (VoxelGridPosition[y][x][0][1]+VoxelGridPosition[y][x+1][0][1]+VoxelGridPosition[y+1][x][0][1]+VoxelGridPosition[y+1][x+1][0][1])/4), 3, Scalar(0,100,255),3);
                if (VoxelGrid[y][x][0] == 3)
                    circle(Visualize, cv::Point2i((VoxelGridPosition[y][x][0][0]+VoxelGridPosition[y][x+1][0][0]+VoxelGridPosition[y+1][x][0][0]+VoxelGridPosition[y+1][x+1][0][0])/4, (VoxelGridPosition[y][x][0][1]+VoxelGridPosition[y][x+1][0][1]+VoxelGridPosition[y+1][x][0][1]+VoxelGridPosition[y+1][x+1][0][1])/4), 3, Scalar(0,255,255),3);
                if (VoxelGrid[y][x][0] == 4)
                    circle(Visualize, cv::Point2i((VoxelGridPosition[y][x][0][0]+VoxelGridPosition[y][x+1][0][0]+VoxelGridPosition[y+1][x][0][0]+VoxelGridPosition[y+1][x+1][0][0])/4, (VoxelGridPosition[y][x][0][1]+VoxelGridPosition[y][x+1][0][1]+VoxelGridPosition[y+1][x][0][1]+VoxelGridPosition[y+1][x+1][0][1])/4), 3, Scalar(0,255,0),3);
                if (VoxelGrid[y][x][0] == 5)
                    circle(Visualize, cv::Point2i((VoxelGridPosition[y][x][0][0]+VoxelGridPosition[y][x+1][0][0]+VoxelGridPosition[y+1][x][0][0]+VoxelGridPosition[y+1][x+1][0][0])/4, (VoxelGridPosition[y][x][0][1]+VoxelGridPosition[y][x+1][0][1]+VoxelGridPosition[y+1][x][0][1]+VoxelGridPosition[y+1][x+1][0][1])/4), 3, Scalar(255,0,0),3);
                if (VoxelGrid[y][x][0] == 6)
                    circle(Visualize, cv::Point2i((VoxelGridPosition[y][x][0][0]+VoxelGridPosition[y][x+1][0][0]+VoxelGridPosition[y+1][x][0][0]+VoxelGridPosition[y+1][x+1][0][0])/4, (VoxelGridPosition[y][x][0][1]+VoxelGridPosition[y][x+1][0][1]+VoxelGridPosition[y+1][x][0][1]+VoxelGridPosition[y+1][x+1][0][1])/4), 3, Scalar(255,0,255),3);
                if (VoxelGrid[y][x][0] == 7)
                    circle(Visualize, cv::Point2i((VoxelGridPosition[y][x][0][0]+VoxelGridPosition[y][x+1][0][0]+VoxelGridPosition[y+1][x][0][0]+VoxelGridPosition[y+1][x+1][0][0])/4, (VoxelGridPosition[y][x][0][1]+VoxelGridPosition[y][x+1][0][1]+VoxelGridPosition[y+1][x][0][1]+VoxelGridPosition[y+1][x+1][0][1])/4), 3, Scalar(40,40,100),3);

                if (VoxelGrid[y][x][1] == 1)
                    circle(Visualize2, cv::Point2i((VoxelGridPosition[y][x][1][0]+VoxelGridPosition[y][x+1][1][0]+VoxelGridPosition[y+1][x][1][0]+VoxelGridPosition[y+1][x+1][1][0])/4, (VoxelGridPosition[y][x][1][1]+VoxelGridPosition[y][x+1][1][1]+VoxelGridPosition[y+1][x][1][1]+VoxelGridPosition[y+1][x+1][1][1])/4), 3, Scalar(0,0,255),3);
                if (VoxelGrid[y][x][1] == 2)
                    circle(Visualize2, cv::Point2i((VoxelGridPosition[y][x][1][0]+VoxelGridPosition[y][x+1][1][0]+VoxelGridPosition[y+1][x][1][0]+VoxelGridPosition[y+1][x+1][1][0])/4, (VoxelGridPosition[y][x][1][1]+VoxelGridPosition[y][x+1][1][1]+VoxelGridPosition[y+1][x][1][1]+VoxelGridPosition[y+1][x+1][1][1])/4), 3, Scalar(0,100,255),3);
                if (VoxelGrid[y][x][1] == 3)
                    circle(Visualize2, cv::Point2i((VoxelGridPosition[y][x][1][0]+VoxelGridPosition[y][x+1][1][0]+VoxelGridPosition[y+1][x][1][0]+VoxelGridPosition[y+1][x+1][1][0])/4, (VoxelGridPosition[y][x][1][1]+VoxelGridPosition[y][x+1][1][1]+VoxelGridPosition[y+1][x][1][1]+VoxelGridPosition[y+1][x+1][1][1])/4), 3, Scalar(0,255,255),3);
                if (VoxelGrid[y][x][1] == 4)
                    circle(Visualize2, cv::Point2i((VoxelGridPosition[y][x][1][0]+VoxelGridPosition[y][x+1][1][0]+VoxelGridPosition[y+1][x][1][0]+VoxelGridPosition[y+1][x+1][1][0])/4, (VoxelGridPosition[y][x][1][1]+VoxelGridPosition[y][x+1][1][1]+VoxelGridPosition[y+1][x][1][1]+VoxelGridPosition[y+1][x+1][1][1])/4), 3, Scalar(0,255,0),3);
                if (VoxelGrid[y][x][1] == 5)
                    circle(Visualize2, cv::Point2i((VoxelGridPosition[y][x][1][0]+VoxelGridPosition[y][x+1][1][0]+VoxelGridPosition[y+1][x][1][0]+VoxelGridPosition[y+1][x+1][1][0])/4, (VoxelGridPosition[y][x][1][1]+VoxelGridPosition[y][x+1][1][1]+VoxelGridPosition[y+1][x][1][1]+VoxelGridPosition[y+1][x+1][1][1])/4), 3, Scalar(255,0,0),3);
                if (VoxelGrid[y][x][1] == 6)
                    circle(Visualize2, cv::Point2i((VoxelGridPosition[y][x][1][0]+VoxelGridPosition[y][x+1][1][0]+VoxelGridPosition[y+1][x][1][0]+VoxelGridPosition[y+1][x+1][1][0])/4, (VoxelGridPosition[y][x][1][1]+VoxelGridPosition[y][x+1][1][1]+VoxelGridPosition[y+1][x][1][1]+VoxelGridPosition[y+1][x+1][1][1])/4), 3, Scalar(255,0,255),3);
                if (VoxelGrid[y][x][1] == 7)
                    circle(Visualize2, cv::Point2i((VoxelGridPosition[y][x][1][0]+VoxelGridPosition[y][x+1][1][0]+VoxelGridPosition[y+1][x][1][0]+VoxelGridPosition[y+1][x+1][1][0])/4, (VoxelGridPosition[y][x][1][1]+VoxelGridPosition[y][x+1][1][1]+VoxelGridPosition[y+1][x][1][1]+VoxelGridPosition[y+1][x+1][1][1])/4), 3, Scalar(40,40,100),3);

                if (VoxelGrid[y][x][2] == 1)
                    circle(Visualize3, cv::Point2i((VoxelGridPosition[y][x][2][0]+VoxelGridPosition[y][x+1][2][0]+VoxelGridPosition[y+1][x][2][0]+VoxelGridPosition[y+1][x+1][2][0])/4, (VoxelGridPosition[y][x][2][1]+VoxelGridPosition[y][x+1][2][1]+VoxelGridPosition[y+1][x][2][1]+VoxelGridPosition[y+1][x+1][2][1])/4), 3, Scalar(0,0,255),3);
                if (VoxelGrid[y][x][2] == 2)
                    circle(Visualize3, cv::Point2i((VoxelGridPosition[y][x][2][0]+VoxelGridPosition[y][x+1][2][0]+VoxelGridPosition[y+1][x][2][0]+VoxelGridPosition[y+1][x+1][2][0])/4, (VoxelGridPosition[y][x][2][1]+VoxelGridPosition[y][x+1][2][1]+VoxelGridPosition[y+1][x][2][1]+VoxelGridPosition[y+1][x+1][2][1])/4), 3, Scalar(0,100,255),3);
                if (VoxelGrid[y][x][2] == 3)
                    circle(Visualize3, cv::Point2i((VoxelGridPosition[y][x][2][0]+VoxelGridPosition[y][x+1][2][0]+VoxelGridPosition[y+1][x][2][0]+VoxelGridPosition[y+1][x+1][2][0])/4, (VoxelGridPosition[y][x][2][1]+VoxelGridPosition[y][x+1][2][1]+VoxelGridPosition[y+1][x][2][1]+VoxelGridPosition[y+1][x+1][2][1])/4), 3, Scalar(0,255,255),3);
                if (VoxelGrid[y][x][2] == 4)
                    circle(Visualize3, cv::Point2i((VoxelGridPosition[y][x][2][0]+VoxelGridPosition[y][x+1][2][0]+VoxelGridPosition[y+1][x][2][0]+VoxelGridPosition[y+1][x+1][2][0])/4, (VoxelGridPosition[y][x][2][1]+VoxelGridPosition[y][x+1][2][1]+VoxelGridPosition[y+1][x][2][1]+VoxelGridPosition[y+1][x+1][2][1])/4), 3, Scalar(0,255,0),3);
                if (VoxelGrid[y][x][2] == 5)
                    circle(Visualize3, cv::Point2i((VoxelGridPosition[y][x][2][0]+VoxelGridPosition[y][x+1][2][0]+VoxelGridPosition[y+1][x][2][0]+VoxelGridPosition[y+1][x+1][2][0])/4, (VoxelGridPosition[y][x][2][1]+VoxelGridPosition[y][x+1][2][1]+VoxelGridPosition[y+1][x][2][1]+VoxelGridPosition[y+1][x+1][2][1])/4), 3, Scalar(255,0,0),3);
                if (VoxelGrid[y][x][2] == 6)
                    circle(Visualize3, cv::Point2i((VoxelGridPosition[y][x][2][0]+VoxelGridPosition[y][x+1][2][0]+VoxelGridPosition[y+1][x][2][0]+VoxelGridPosition[y+1][x+1][2][0])/4, (VoxelGridPosition[y][x][2][1]+VoxelGridPosition[y][x+1][2][1]+VoxelGridPosition[y+1][x][2][1]+VoxelGridPosition[y+1][x+1][2][1])/4), 3, Scalar(255,0,255),3);
                if (VoxelGrid[y][x][2] == 7)
                    circle(Visualize3, cv::Point2i((VoxelGridPosition[y][x][2][0]+VoxelGridPosition[y][x+1][2][0]+VoxelGridPosition[y+1][x][2][0]+VoxelGridPosition[y+1][x+1][2][0])/4, (VoxelGridPosition[y][x][2][1]+VoxelGridPosition[y][x+1][2][1]+VoxelGridPosition[y+1][x][2][1]+VoxelGridPosition[y+1][x+1][2][1])/4), 3, Scalar(40,40,100),3);
            }
        }
        
        imshow("Visualize_firstlevel", Visualize);
        imshow("Visualize_secondlevel", Visualize2);
        imshow("Visualize_thirdlevel", Visualize3);
        cv::waitKey(1);
        cv::Mat image_RGB = imRGB.clone();
        Show_Results(FirstLevel, image_RGB, "FirstLevel");
    }
    */
    VoxelGridSizeX = maxX - minX + 1;
    VoxelGridSizeY = maxY - minY + 1;
    return 1;
}

int ArgMax( int red, int orange, int yellow, int green, int blue, int purple, int brown)
{
    int Max = 0;
    int Arg = 0;
    if (red > Max) { Max = red; Arg = 1; }
    if (orange > Max) { Max = orange; Arg = 2; }
    if (yellow > Max) { Max = yellow; Arg = 3; }
    if (green > Max ) { Max = green ; Arg = 4; }
    if (blue > Max ) { Max = blue ; Arg = 5; }
    if (purple > Max ) { Max = purple; Arg = 6; }
    if (brown > Max ) { Max = brown ; Arg = 7; }

    return Arg;
}

void GetGridPos(vector<vector<vector<vector<int>>>> VoxelGridPosition, int VoxelGridSize, int col, int row, int level, int &GridX, int &GridY)
{
    for (int X = 0; X < VoxelGridSize - 1; X ++)
    {
        for ( int Y = 0; Y < VoxelGridSize - 1; Y++)
        {
            if ((VoxelGridPosition[Y][X][level][0] - col)*(VoxelGridPosition[Y][X+1][level][0] - col) < 0 && (VoxelGridPosition[Y][X][level][1] - row)*(VoxelGridPosition[Y+1][X][level][1] - row) < 0)  {GridX = X; GridY = Y; }// cout<<"X"<<(int)VoxelGridPosition.at<Vec2i>(Y,X,level)[0] << "~~~~~" <<  col << endl;}
            
        }
    }
}

