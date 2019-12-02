#include "opencv2/opencv.hpp"
#include <opencv2/line_descriptor.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/viz.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/line_descriptor.hpp>
#include <math.h>
using namespace cv;
using namespace cv::line_descriptor;
using namespace std;
cv::Mat line_segment(cv::Mat);
void Get_keyLines(std::vector<KeyLine> &keyLines, cv::Mat imRGB, cv::Mat mask);
cv::Mat line_visualize(std::vector<KeyLine> keyLines, cv::Mat imRGB);
struct KeyLine3D
{
    float startPointX; 
    float startPointY;
    float startPointD;
    float endPointX;
    float endPointY;
    float endPointD;
    float directionVectorX;
    float directionVectorY;
    float directionVectorD;
};
struct GridXYZ3D
{
    cv::Vec3f GridX;
    cv::Vec3f GridY;
    cv::Vec3f GridZ;
};

void Get3DLines(std::vector<KeyLine>, cv::Mat, std::vector<KeyLine3D>&);
int GetGridXYZ(std::vector<KeyLine3D> keyLines3D, cv::Point3f Initial_Z, std::vector<cv::Point3f> &GRIDXYZ, cv::Point3f &Origin);
int LinesContactChecker(KeyLine3D, KeyLine3D, float, cv::Point3f &);

cv::Mat line_segment(cv::Mat image)
{
    Ptr<LSDDetector> bd =  LSDDetector::createLSDDetector();
        cv::Mat mask = cv::Mat::ones( image.size(), CV_8UC1);
        vector<KeyLine> lines;
        cv::Mat output = image;
        bd->detect(image, lines, 2, 1, mask);
        if(output.channels() == 1)
            cvtColor(output, output, COLOR_GRAY2BGR);
        for (size_t i = 0; i < lines.size(); i++)
        {
            KeyLine kl = lines[i];
            if( kl.octave == 0)
            {
                // get a random color
                int R = ( rand() % (int) ( 255 + 1 ) );
                int G = ( rand() % (int) ( 255 + 1 ) );
                int B = ( rand() % (int) ( 255 + 1 ) );

                // get extremes of line
                Point pt1 = Point2f( kl.startPointX, kl.startPointY);
                Point pt2 = Point2f( kl.endPointX, kl.endPointY);

                // draw a line
                line( output, pt1, pt2, Scalar(B, G, R) , 3);
                //line( masked, pt1, pt2, Scalar(B, G, R) , 3);
            }
        }
    return output;
}

void Get_keyLines(std::vector<KeyLine> &keyLines, cv::Mat imRGB, cv::Mat mask)
{
    Ptr<LSDDetector> bd = LSDDetector::createLSDDetector();
    vector<KeyLine> lines;
    bd->detect(imRGB, lines, 2, 1, mask);
    keyLines = lines;
}

cv::Mat line_visualize(std::vector<KeyLine> keyLines, cv::Mat imRGB)
{
    cv::Mat line_image = imRGB.clone();
    if (imRGB.channels() == 1)
        cvtColor(imRGB, imRGB, COLOR_GRAY2BGR);
    for (size_t i = 0; i < keyLines.size(); i++)
    {
        KeyLine kl = keyLines[i];
        if (kl.octave == 0)
        {
            int R = ( rand() % (int) ( 255 + 1 ) );
            int G = ( rand() % (int) ( 255 + 1 ) );
            int B = ( rand() % (int) ( 255 + 1 ) );

            // get extremes of line
            Point pt1 = Point2f( kl.startPointX, kl.startPointY);
            Point pt2 = Point2f( kl.endPointX, kl.endPointY);

            // draw a line
            line( line_image, pt1, pt2, Scalar(B, G, R) , 3);
        }
    }
    return line_image;
}

void Get3DLines(std::vector<KeyLine> keyLines, cv::Mat imDepth, std::vector<KeyLine3D>& keyLines3D)
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

    for (size_t i = 0; i < (size_t)keyLines.size(); i++)
    {
        KeyLine kl = keyLines[i];
        if (kl.octave == 0)
        {
            KeyLine3D temp; 

            temp.startPointD = imDepth.at<uint16_t>(kl.startPointY, kl.startPointX) / scale; // depth
            temp.startPointX = (kl.startPointX - cx) * temp.startPointD / (fx);
            temp.startPointY = (kl.startPointY - cy) * temp.startPointD / (fy);
            
            temp.endPointD = imDepth.at<uint16_t>(kl.endPointY, kl.endPointX) / scale;
            temp.endPointX = (kl.endPointX - cx) * temp.endPointD / (fx);
            temp.endPointY = (kl.endPointY - cy) * temp.endPointD / (fy);
            temp.directionVectorX = temp.endPointX- temp.startPointX;
            temp.directionVectorY = temp.endPointY - temp.startPointY;
            temp.directionVectorD = temp.endPointD - temp.startPointD;

            //cout << "X" << temp.directionVectorX << "Y" << temp.directionVectorY << "D" << temp.directionVectorD << endl; // very small numbers are here...??

            float VectorNorm = sqrt(temp.directionVectorX*temp.directionVectorX + temp.directionVectorY*temp.directionVectorY + temp.directionVectorD*temp.directionVectorD + 1e-30 );
            temp.directionVectorX = temp.directionVectorX / VectorNorm;
            temp.directionVectorY = temp.directionVectorY / VectorNorm;
            temp.directionVectorD = temp.directionVectorD / VectorNorm; 

            VectorNorm = temp.directionVectorX*temp.directionVectorX + temp.directionVectorY*temp.directionVectorY + temp.directionVectorD*temp.directionVectorD;
            if ( VectorNorm == 1 ) // sanity check. for some reason, sometimes this is 0.
                keyLines3D.push_back(temp);

        }
    }
}       

int GetGridXYZ(std::vector<KeyLine3D> keyLines3D, cv::Point3f Initial_Z, std::vector<cv::Point3f> &GRIDXYZ, cv::Point3f &Origin)
{
    cv::Point3f X;
    cv::Point3f Y;
    float grid_distance = 0.025;
    int x_num = 0;
    int y_num = 0;
    
    int GRID_FOUND = 0;
    float Thresh = 0.15; // inner product threshold
    float DistThresh = 0.003; // endpoints distance threshold
    float innerProd;
    float gridDistThresh = 0.001; // grid distance threshold
    int Find_num = 1;
    
    for (size_t i = 0; i < (size_t)keyLines3D.size(); i++)
    {
        KeyLine3D kl = keyLines3D[i];
        innerProd = kl.directionVectorX * Initial_Z.x + kl.directionVectorY * Initial_Z.y + kl.directionVectorD * Initial_Z.z ; // get the inner product with Z
        if ( abs(innerProd) > 1+1e-10 ) cout << "first inner prod exceeds 1" << endl;

        //cout << "kl1.directionVectorX:" << kl.directionVectorX<< "Y: " << kl.directionVectorY << "D: " << kl.directionVectorD << endl;
        if ( -Thresh < innerProd && innerProd < Thresh ) // if the line is perpendicular to Z,
        {
            //check if there is another line that is connected to the line being considered
            for (size_t j=0; j<(size_t)keyLines3D.size(); j++)
            {
                if (i==j) continue;
                KeyLine3D kl2 = keyLines3D[j];
                innerProd = kl2.directionVectorX * Initial_Z.x + kl2.directionVectorY * Initial_Z.y + kl2.directionVectorD * Initial_Z.z ; // get the inner product with Z
                //cout << "kl2 norm: " << sqrt(kl2.directionVectorX*kl2.directionVectorX+kl2.directionVectorY*kl2.directionVectorY+kl2.directionVectorD*kl2.directionVectorD) << endl;
                //cout << "Initial_Z norm: " << sqrt(Initial_Z.x*Initial_Z.x+Initial_Z.y*Initial_Z.y+Initial_Z.z*Initial_Z.z) << endl;
                //cout << "kl2.directionVectorX:" << kl2.directionVectorX<< "Y: " << kl2.directionVectorY << "D: " << kl2.directionVectorD << endl;
                //cout << "second inner Prod: " << innerProd << endl;
                if ( abs(innerProd) > 1+1e-10 ) cout << "second inner prod exceeds 1: " << innerProd << endl;
                if ( abs(innerProd) < Thresh ) // if the line is perpendicular to Z,
                {
                    innerProd = kl.directionVectorX * kl2.directionVectorX + kl.directionVectorY * kl2.directionVectorY + kl.directionVectorD * kl2.directionVectorD; // inner prod with each other
                    //cout << "inner prod btw lines : " << innerProd << endl;
                    if ( abs(innerProd) > 1 ) cout << "inner prod btw lines exceeds 1" << endl;

                    if ( abs(innerProd) < Thresh ) // if the line is perpendicular to each other,
                    {

                        if ( LinesContactChecker(kl, kl2, DistThresh, Origin)) // if the lines are in contact,
                        {
                            /*
                            cv::Vec3f line1, line2;
                            line1 = cv::Vec3f(kl.directionVectorX, kl.directionVectorY, kl.directionVectorD);
                            line2 = cv::Vec3f(kl2.directionVectorX, kl2.directionVectorY, kl2.directionVectorD);
                            cv::Vec3f cross = (line1.cross(line2));
                            innerProd = cross[0] * Initial_Z.x + cross[1] * Initial_Z.y + cross[2] * Initial_Z.z;
                            // did cross product to ensure right-hand coordinate. But, this approach doesn't work.
                            */
                            for (size_t k=0; k<(size_t)keyLines3D.size(); k++)
                            {
                                if (k == i || k == j) continue;
                                
                                KeyLine3D kl3 = keyLines3D[k];

                                innerProd = kl3.directionVectorX*Initial_Z.x + kl3.directionVectorY*Initial_Z.y + kl3.directionVectorD*Initial_Z.z;
                                if ( abs( innerProd ) > Thresh ) continue; // if the line has z component, continue.
                                
                                innerProd = kl2.directionVectorX*kl3.directionVectorX + kl2.directionVectorY*kl3.directionVectorY + kl2.directionVectorD*kl3.directionVectorD;
                                if ( abs( innerProd ) < Thresh ) // if the hypothesis and kl2 are orthogonal, or, if the hypothesis is aligned to kl,
                                {
                                    float distance = abs( kl2.directionVectorX * ( kl3.startPointX - kl.startPointX ) + kl2.directionVectorY * ( kl3.startPointY - kl.startPointY ) + kl2.directionVectorD * ( kl3.startPointD - kl.startPointD ) );
                                    //cout << endl << "distance along kl direction: " << distance << endl;
                                    if ( fmod( distance , grid_distance ) < gridDistThresh ) // if the distance is in the grid,
                                    {
                                        x_num ++ ; // found a match in x direction ( x = kl direction )
                                    }
                                }
                                innerProd = kl.directionVectorX*kl3.directionVectorX + kl.directionVectorY*kl3.directionVectorY + kl.directionVectorD*kl3.directionVectorD;
                                if ( abs( innerProd ) < Thresh ) // if the hypothesis and kl are orthogonal, or, if the hypothesis is aligned to kl2,
                                {
                                    float distance = abs( kl.directionVectorX * ( kl3.startPointX - kl2.startPointX ) + kl.directionVectorY * ( kl3.startPointY - kl2.startPointY ) + kl.directionVectorD * ( kl3.startPointD - kl2.startPointD ) );
                                    //cout << endl << "distance along kl2 direction: " << distance << endl;
                                    //cout << fmod(distance,grid_distance) << endl;
                                    if ( fmod( distance , grid_distance ) < gridDistThresh ) // if the distance is in the grid,
                                    {
                                        y_num ++ ; // found a match in y direction ( y = kl2 direction )
                                    }
                                }
                                if ( x_num >= Find_num || y_num >= Find_num )
                                {

                                    GRIDXYZ.push_back(cv::Point3f(kl.directionVectorX, kl.directionVectorY, kl.directionVectorD));
                                    GRIDXYZ.push_back(cv::Point3f(kl2.directionVectorX, kl2.directionVectorY, kl2.directionVectorD));
                                    GRIDXYZ.push_back(Initial_Z);
                                    GRID_FOUND ++;
                                    return 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    //if (~GRID_FOUND) cout<< "no grid found! " <<endl;
    //cout << "x_num, y_num: " << x_num << ", " << y_num << endl;
    return 0;
}

int LinesContactChecker(KeyLine3D kl, KeyLine3D kl2, float DistThresh, cv::Point3f &Origin)
{
    if (sqrt(pow(kl.startPointX - kl2.startPointX, 2) + pow(kl.startPointY - kl2.startPointY,2) + pow(kl.startPointD - kl2.startPointD,2)) < DistThresh)
    {
        Origin = cv::Point3f(kl.startPointX, kl.startPointY, kl.startPointD);
        return 1;
    }
    else if (sqrt(pow(kl.startPointX - kl2.endPointX, 2) + pow(kl.startPointY - kl2.endPointY,2) + pow(kl.startPointD - kl2.endPointD,2)) < DistThresh)
    {
        Origin = cv::Point3f(kl.startPointX, kl.startPointY, kl.startPointD);
        return 1;
    }
    else if (sqrt(pow(kl.endPointX - kl2.startPointX, 2) + pow(kl.endPointY - kl2.startPointY,2) + pow(kl.endPointD - kl2.startPointD,2)) < DistThresh)
    {
        Origin = cv::Point3f(kl.endPointX, kl.endPointY, kl.endPointD);
        return 1;
    }
    else if (sqrt(pow(kl.endPointX - kl2.endPointX, 2) + pow(kl.endPointY - kl2.endPointY,2) + pow(kl.endPointD - kl2.endPointD,2)) < DistThresh )
    {
        Origin = cv::Point3f(kl.endPointX, kl.endPointY, kl.endPointD);
        return 1;
    }
    else { return 0; }
}

cv::Point2d Project2Img(cv::Point3f Point)
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

    float Z = Point.z;
    
    int X = (fx * Point.x / Z) + cx;
    int Y = (fy * Point.y / Z) + cy;

    return cv::Point2d(X,Y);
}
void OriginVisualize(cv::Point3f Origin, std::vector<cv::Point3f> GRIDXYZ, cv::Mat imRGB)
{
    cout << "origin 3d x, y, z: " << Origin.x << ", " << Origin.y << ", " << Origin.z << endl;
    cv::Mat originImage = imRGB.clone();
    cv::Point2f origin = Project2Img(Origin);
    cout << "origin x, y :" << origin.x << ", " << origin.y << endl;
    
    cv::Point3f Destin = cv::Point3f(GRIDXYZ[0].x - Origin.x, GRIDXYZ[0].y-Origin.y, GRIDXYZ[0].z - Origin.z);
    cv::Point2f destin = Project2Img(Destin);
    line(originImage, origin, destin, Scalar(255, 0, 0) , 3);

    Destin = cv::Point3f(GRIDXYZ[1].x - Origin.x,GRIDXYZ[1].y - Origin.y, GRIDXYZ[1].z - Origin.z);
    destin = Project2Img(Destin);
    line(originImage, origin, destin, Scalar(0, 255, 0) , 3);

    Destin = cv::Point3f(GRIDXYZ[2].x - Origin.x, GRIDXYZ[2].y - Origin.y, GRIDXYZ[2].z - Origin.z);
    destin = Project2Img(Destin);
    line(originImage, origin, destin, Scalar(0, 0, 255) , 3);

    imshow("GRIDXYZ", originImage);
    cv::waitKey(1);
}
