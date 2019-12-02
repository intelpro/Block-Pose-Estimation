#include "ros/ros.h"
#include "voxel_grid/voxel_grid.h"
#include "voxel_grid.h"
using namespace std;
using namespace cv;

vector<vector<vector<int>>> ConvertMsg2Vector(voxel_grid::voxel_grid::ConstPtr&, vector<vector<vector<int>>>&);
void VGCallback(const voxel_grid::voxel_grid::ConstPtr& msg)
{
    vector<vector<vector<int>>> VoxelGrid( msg->VoxelGridSize, vector<vector<int>>(VoeclGridSize, vector<int> (BlockHeight,0)));
    
    ConvertMsg2Vector(msg, VoxelGrid);
    cout << "origin color: " << VoxelGrid[20][20][0] << endl;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "subscriber");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("voxel_grid", 1000, VGCallback);
    ros::spin();
    return 0;
}

vector<vector<vector<int>>> ConvertMsg2Vector(voxel_grid::voxel_grid::ConstPtr& msg, vector<vector<vector<int>>>& VoxelGrid)
{
    for (y=0; y<msg->VoxelGridSize; y++)
    {
        for (x=0; x<msg->VoxelGridSize; x++)
        {
            for (z=0; z<BlockHeight; z++)
            {
                VoxelGrid[y][x][z] = msg->voxel_grid[ z + msg->BlockHeight*x + msg->BlockHeight*msg->VoxelGridSize*y]
            }
        }
    }

    return 0;
}

