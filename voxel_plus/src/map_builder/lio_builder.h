#pragma once
#include "ieskf.h"
#include "commons.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include "voxel_map.h"
#include "feat_voxel_map.h"
#include "my_viewer.h"

using namespace std;

namespace lio
{
    enum LIOStatus
    {
        IMU_INIT,
        MAP_INIT,
        LIO_MAPPING

    };
    struct LIOConfig
    {
        int opti_max_iter = 5;
        double na = 0.01;
        double ng = 0.01;
        double nba = 0.0001;
        double nbg = 0.0001;
        int imu_init_num = 20;
        Eigen::Matrix3d r_il = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p_il = Eigen::Vector3d::Zero();
        bool gravity_align = true;
        bool estimate_ext = false;

        double scan_resolution = 0.1;
        double voxel_size = 0.5;
        int update_size_thresh = 10;
        int max_point_thresh = 100;
        double plane_thresh = 0.01;

        double ranging_cov = 0.04;
        double angle_cov = 0.1;

        double merge_thresh_for_angle = 0.1;
        double merge_thresh_for_distance = 0.04;
        int map_capacity = 100000;


        // dongyan new config
        string model_file = "model_file.pt";        // p2v model file, xxx.pt
        double init_time = 2.0f;                    // wait for some seconds to switch to p2v prediction.
        double valid_weight_threshold = 0.8f;       // threshold > this value's weight can contribute to the residual
        int batch_size = 8;                         // batch size for prediction.
        int prediction_skip = 1;                    // how many frames to skip for prediction.

        bool use_offline_rosbag = false;
        string offline_rosbag_file = "empty.bag";

        double r_info_scale = 5000;
    };


    struct LIODataGroup
    {
        IMUData last_imu;
        std::vector<IMUData> imu_cache;
        std::vector<Pose> imu_poses_cache;
        Eigen::Vector3d last_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_gyro = Eigen::Vector3d::Zero();
        double last_cloud_end_time = 0.0;
        double gravity_norm;
        kf::Matrix12d Q = kf::Matrix12d::Identity();
        std::vector<ResidualData> residual_info;
    };

    class LIOBuilder
    {
    public:
        LIOBuilder() = default;

        void loadConfig(LIOConfig &_config);

        bool initializeImu(std::vector<IMUData> &imus);

        void undistortCloud(SyncPackage &package);

        void process(SyncPackage &package);

        void sharedUpdateFunc(const kf::State &state, kf::SharedState &shared_state, ScanRegisterViewer& my_viewer, bool update_view);       // state is not changed. So change to const.
        void sharedUpdateFunc_p2v(const kf::State &state, kf::SharedState &shared_state, ScanRegisterViewer& my_viewer, bool update_view);       // new!

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr lidarToWorld(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);

        pcl::PointCloud<pcl::PointXYZINormal>::Ptr lidarToBody(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);


    public:
        kf::IESKF kf;
        LIOConfig config;
        LIODataGroup data_group;
        LIOStatus status = LIOStatus::IMU_INIT;
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr lidar_cloud;
        pcl::VoxelGrid<pcl::PointXYZINormal> scan_filter;
        std::shared_ptr<VoxelMap> map;

        // Dongyan New
        LIODataGroup data_group_p2v_;
        std::shared_ptr<FeatVoxelMap> map_p2v_;     // 


    };

} // namespace lio
