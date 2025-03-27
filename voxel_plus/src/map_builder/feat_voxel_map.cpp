#include "feat_voxel_map.h"
#include "my_ros_debugger.hpp"

using namespace std;
using namespace lio;


int g_debug_featVoxelMap_init_cnt = 20;      // when to create FeatVoxelMap


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
    my_featmap_.clear();
}

void FeatVoxelMap::buildFeatVoxelMap(const std::shared_ptr<VoxelMap>& map){
    // assign voxel map's data into my own.
    ROS_WARN("in `buildFeatVoxelMap`");
    printInfo();

    for(const auto& item : map->featmap){
        // my_feat_map_.
        VoxelKey position = item.first;
        cout << "VoxelKey for map->featmap: " << item.second->group_id << endl;
        auto my_item = std::make_shared<FeatVoxelGrid>(position);
        my_featmap_[position] = my_item;
    }

    ROS_INFO("build done.");
    printInfo();
}


void FeatVoxelMap::printInfo(void){
    ROS_WARN("Print FeatVoxelMap info: ");
    cout << "-------------------------------------------" << endl;
    cout << "Total FeatVoxelMap: " << my_featmap_.size() << endl;
}



