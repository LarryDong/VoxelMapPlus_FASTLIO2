
// feat_voxel_map defined by DONGYAN.

#pragma once
#include <vector>
#include <Eigen/Eigen>
#include <unordered_map>
#include <iostream>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "voxel_map.h"
#include "p2v_model.h"


typedef Eigen::Vector3d V3D;

using namespace lio;




class FeatVoxelGrid{
public:
    FeatVoxelGrid(VoxelKey _position);
    void printInfo(void);
    void addPoint(const V3D& p);
    // void extractFeature(void);

public: 
    static uint64_t count_;
    double voxel_size_;
    uint64_t group_id_;
    std::vector<V3D> temp_points_;      // points used for voxel feature extraction
    std::vector<V3D> normalized_points_; // normalized points, based on the lower_boundary.
    VoxelKey position_;
    V3D lower_boundary_;                 // lower boundary of voxel, based on the "position"
    bool is_feat_extracted_;            // if feature extracted, set to true
    int extract_feat_threshold_;        // how many points are neede to extract voxel feat. Default: 25?
    std::vector<double> voxel_feature_; // PointNet's output
};



typedef std::unordered_map<VoxelKey, std::shared_ptr<FeatVoxelGrid>, VoxelKey::Hasher> MyFeatMap;


class FeatVoxelMap{
public:
    FeatVoxelMap(std::string model, double valid_weight_threshold = 0.8);
    // void buildFeatVoxelMap(const std::shared_ptr<VoxelMap>& map);    // use VoxelMap to create my featMap. Issue: No points info.

    void buildFeatVoxelMap(const std::vector<V3D> &pts);
    void buildFeatVoxelMap(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& map_ptr);
    void printInfo(const std::string &info = "default");
    void saveToFile(void);

    void update(const std::vector<V3D> &pts);

    inline VoxelKey calcVoxelKey(const Eigen::Vector3d &point){
        const double voxel_size = 0.5;
        Eigen::Vector3d idx = (point / voxel_size).array().floor();
        return VoxelKey(static_cast<int64_t>(idx(0)), static_cast<int64_t>(idx(1)), static_cast<int64_t>(idx(2)));
    }

    bool buildResidualByPointnet(ResidualData &data, std::shared_ptr<FeatVoxelGrid> voxel_grid);



public:
    MyFeatMap my_featmap_;
    P2VModel p2v_model_;
    bool is_inited_;                    // if the whole feature map is inited?
    double valid_weight_threshold_;     // default: 50

private:
    std::string save_folder_;           // give a folder to save the output;

public:

};

