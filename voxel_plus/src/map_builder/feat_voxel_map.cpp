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
    
    vector<V3D>().swap(temp_points_);
    temp_points_.reserve(4*extract_feat_threshold_);

#ifdef DEBUG_INFO
    ROS_DEBUG_STREAM("FeatVoxelGrid is inited. ID: " << group_id_);
#endif
}

void FeatVoxelGrid::addPoint(const V3D& p){
    temp_points_.push_back(p);
}

void FeatVoxelGrid::printInfo(void){
    // cout <<"------------------ FeatVoxelGrid info ------------------" << endl;
    cout << "[FeatVoxelGrid info] Id: " << group_id_ << ", key: " << position_ << ", curr_points_num: " << temp_points_.size() << ", has_feature?: " << is_feat_extracted_ << endl;
}


////////////////////////////////////////////////////////////////////////
//////////////////////// Voxel Map
////////////////////////////////////////////////////////////////////////

FeatVoxelMap::FeatVoxelMap(void){
    ROS_WARN("FeatVoxelMap is inited.");
    my_featmap_.clear();
    save_folder_ = "/home/larrydong/Desktop/voxel_output/";
}

void FeatVoxelMap::saveToFile(void){
    cout << "Write voxel info into folder: " << save_folder_ << endl;
    VoxelKey::Hasher hasher;
    string summary_folder = save_folder_ + "summary.csv";
    std::ofstream out_summary(summary_folder);
    if (!out_summary) {
        // std::cerr << "[ERROR] Cannot open file: " << filename << std::endl;
        my_ros_utility::color_cout("[VoxelMap Save Error] Cannot open file: " + summary_folder);
        return;
    }
    out_summary << "hash-key,number-of-points,position-x,position-y,position-z" << endl;
    for(auto item:my_featmap_){
        string hash_key = std::to_string(hasher(item.first));
        string filename = save_folder_ + hash_key + ".csv";
        std::ofstream out_file(filename);
        if (!out_file) {
            // std::cerr << "[ERROR] Cannot open file: " << filename << std::endl;
            my_ros_utility::color_cout("[VoxelMap Save Error] Cannot open file: " + filename);
            return;
        }
        std::vector<V3D> pts = item.second->temp_points_;
        for(auto p:pts){
            out_file << p[0] << "," << p[1] << "," << p[2] << endl;
        }
        cout << "--> Written " << pts.size() << " points into voxel file: " << filename << endl;
        out_file.close();

        // write to summary file
        // hash, num-of-points, position
        out_summary << hash_key <<"," << pts.size() <<"," << item.second->position_ << endl;
    }
    out_summary.close();

    cout << "[INFO] FeatVoxelMap save to file done. Total: " << my_featmap_.size() << endl;
}



void FeatVoxelMap::printInfo(void){
    cout << "-------------------------------------------------------------" << endl;
    ROS_WARN("Print FeatVoxelMap info: ");
    cout << "Total FeatVoxelMap: " << my_featmap_.size() << endl;
    cout << "All VoxelGrid info: " << endl;
    for(auto vg:my_featmap_){
        vg.second->printInfo();
    }
    cout << "-------------------------------------------------------------" << endl;
}

// build map from vector3d points
void FeatVoxelMap::buildFeatVoxelMap(const std::vector<V3D>& points){
#ifdef DEBUG_INFO
    ROS_WARN("In `buildFeatVoxelMap`, using points.");
#endif

    for(const V3D& p : points){
        VoxelKey k = calcVoxelKey(p);
        auto it = my_featmap_.find(k);
        if(it == my_featmap_.end()){
#ifdef DEBUG_INFO
            cout <<"Create a new voxel grid on: " << k << endl;
#endif
            my_featmap_[k] = std::make_shared<FeatVoxelGrid>(k);
        }
        my_featmap_[k]->addPoint(p);
    }
}

// build map from a PCL pointcloud.
void FeatVoxelMap::buildFeatVoxelMap(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& map_ptr){
    std::vector<V3D> pts;
    for(const pcl::PointXYZINormal& p : map_ptr->points)
        pts.push_back(V3D(p.x, p.y, p.z));
    buildFeatVoxelMap(pts);
}



// Not used codes.

#if 0
void FeatVoxelMap::buildFeatVoxelMap(const std::shared_ptr<VoxelMap>& map){
    // assign voxel map's data into my own.
    ROS_WARN("in `buildFeatVoxelMap`");
    printInfo();
    
    // featmap is `std::unordered_map<VoxelKey, std::shared_ptr<VoxelGrid>, VoxelKey::Hasher>`
    for(const auto& item : map->featmap){
        VoxelKey position = item.first;
        // cout << "VoxelKey for map->featmap: " << item.second->group_id << endl;
        auto my_item = std::make_shared<FeatVoxelGrid>(position);
        my_featmap_[position] = my_item;
    }

    ROS_INFO("build done.");
    printInfo();
}
#endif

