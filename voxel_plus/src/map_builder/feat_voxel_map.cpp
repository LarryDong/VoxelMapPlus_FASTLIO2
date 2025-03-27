#include "feat_voxel_map.h"
#include "my_ros_debugger.hpp"

using namespace std;
using namespace lio;


////////////////////////////////////////////////////////////////////////
//////////////////////// Voxel Grid
////////////////////////////////////////////////////////////////////////
uint64_t FeatVoxelGrid::count_ = 0;

FeatVoxelGrid::FeatVoxelGrid(VoxelKey _position):
    position_(_position.x, _position.y, _position.z)
{

    group_id_ = FeatVoxelGrid::count_++;
    is_feat_extracted_ = false;
    extract_feat_threshold_ = 25;               // default: 25 poins to extract feat.

    ROS_INFO_STREAM("FeatVoxelGrid is inited. ID: " << group_id_);
    
    ;
}




////////////////////////////////////////////////////////////////////////
//////////////////////// Voxel Map
////////////////////////////////////////////////////////////////////////

FeatVoxelMap::FeatVoxelMap(void){
    ROS_WARN("FeatVoxelMap is inited.");
}

void FeatVoxelMap::buildFeatVoxelMap(const VoxelMap*){
    // assign voxel map's data into my own.
    ROS_WARN("Build FeatVoxelMap");
}


void FeatVoxelMap::printInfo(void){
    ROS_WARN("Print FeatVoxelMap info: ");
    cout << "-------------------------------------------" << endl;
    cout << "Total FeatVoxelMap: " << my_feat_map_.size() << endl;
}



