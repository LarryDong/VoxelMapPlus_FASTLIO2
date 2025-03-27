
// feat_voxel_map defined by DONGYAN.

#pragma once
#include <vector>
#include <Eigen/Eigen>
#include <unordered_map>
#include <iostream>

#include "voxel_map.h"

typedef Eigen::Vector3d V3D;

using namespace lio;




class FeatVoxelGrid{
public:
    FeatVoxelGrid(VoxelKey _position);

public: 
    static uint64_t count_;
    uint64_t group_id_;
    std::vector<V3D> temp_points_;      // points used for voxel feature extraction
    VoxelKey position_;
    bool is_feat_extracted_;            // if feature extracted, set to true
    int extract_feat_threshold_;        // how many points are neede to extract voxel feat. Default: 25?
};



typedef std::unordered_map<VoxelKey, std::shared_ptr<FeatVoxelGrid>, VoxelKey::Hasher> MyFeatMap;


class FeatVoxelMap{
public:
    FeatVoxelMap(void);
    void buildFeatVoxelMap(const VoxelMap*);
    void printInfo(void);

public:
    // void printInfo(bool verbose=false){return;}
    MyFeatMap my_feat_map_;

public:

};

