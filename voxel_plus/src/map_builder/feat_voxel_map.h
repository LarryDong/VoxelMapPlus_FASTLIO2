
// feat_voxel_map defined by DONGYAN.

#pragma once
#include <vector>
#include <Eigen/Eigen>
#include <unordered_map>
#include <iostream>

#include <ros/ros.h>
#include "voxel_map.h"

typedef Eigen::Vector3d V3D;

using namespace lio;




class FeatVoxelGrid{
public:
    FeatVoxelGrid(VoxelKey _position);
    void printInfo(void);
    void addPoint(const V3D& p);

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
    // void buildFeatVoxelMap(const std::shared_ptr<VoxelMap>& map);    // use VoxelMap to create my featMap. Issue: No points info.
    void buildFeatVoxelMap(const std::vector<V3D> &pts);
    void printInfo(void);

public:
    MyFeatMap my_featmap_;

private:
    inline VoxelKey calcVoxelKey(const Eigen::Vector3d &point){
        const double voxel_size = 0.5;
        Eigen::Vector3d idx = (point / voxel_size).array().floor();
        return VoxelKey(static_cast<int64_t>(idx(0)), static_cast<int64_t>(idx(1)), static_cast<int64_t>(idx(2)));
    }

public:

};

