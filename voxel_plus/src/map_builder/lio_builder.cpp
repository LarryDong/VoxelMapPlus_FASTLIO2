#include "lio_builder.h"
#include "my_ros_debugger.hpp"

#include "feat_voxel_map.h"
#include "my_viewer.h"

#include <omp.h>
#include <iostream>

int g_scan_cnt = -1;
extern int g_debug_featVoxelMap_init_cnt;
extern pcl::PointCloud<pcl::PointXYZINormal>::Ptr g_global_pc;
extern ScanRegisterViewer my_viewer;
vector<Eigen::Vector3d> g_p2v_, g_p2plane_;

namespace lio
{
    void LIOBuilder::loadConfig(LIOConfig &_config)
    {
        config = _config;
        status = LIOStatus::IMU_INIT;
        data_group.Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * config.ng;
        data_group.Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * config.na;
        data_group.Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * config.nbg;
        data_group.Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * config.nba;
        if (config.scan_resolution > 0.0)
            scan_filter.setLeafSize(config.scan_resolution, config.scan_resolution, config.scan_resolution);

        map = std::make_shared<VoxelMap>(config.max_point_thresh, config.update_size_thresh, config.plane_thresh, config.voxel_size, config.map_capacity);
        

        // Init my map.
        map_p2v_ = std::make_shared<FeatVoxelMap>(_config.model_file, _config.valid_weight_threshold);


        VoxelGrid::merge_thresh_for_angle = config.merge_thresh_for_angle;
        VoxelGrid::merge_thresh_for_distance = config.merge_thresh_for_distance;

        lidar_cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
        kf.set_share_function(
            [this](kf::State &s, kf::SharedState &d, ScanRegisterViewer &v, bool b)
            { sharedUpdateFunc(s, d, v, b); });
        kf.set_share_function_p2v(
            [this](kf::State &s, kf::SharedState &d, ScanRegisterViewer &v, bool b)
            { sharedUpdateFunc_p2v(s, d, v, b); });
        data_group.residual_info.resize(10000);
    }

    bool LIOBuilder::initializeImu(std::vector<IMUData> &imus)
    {
        data_group.imu_cache.insert(data_group.imu_cache.end(), imus.begin(), imus.end());
        if (data_group.imu_cache.size() < config.imu_init_num)      // imu_init_num=20. 2-second init.
            return false;
        Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
        for (const auto &imu : data_group.imu_cache)
        {
            acc_mean += imu.acc;
            gyro_mean += imu.gyro;
        }
        acc_mean /= static_cast<double>(data_group.imu_cache.size());
        gyro_mean /= static_cast<double>(data_group.imu_cache.size());
        data_group.gravity_norm = acc_mean.norm();
        kf.x().rot_ext = config.r_il;
        kf.x().pos_ext = config.p_il;
        kf.x().bg = gyro_mean;
        if (config.gravity_align)
        {
            kf.x().rot = (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), Eigen::Vector3d(0.0, 0.0, -1.0)).matrix());
            kf.x().initG(Eigen::Vector3d(0, 0, -1.0));
        }
        else
        {
            kf.x().initG(-acc_mean);
        }
        kf.P().setIdentity();
        kf.P().block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 0.00001;
        kf.P().block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 0.00001;
        kf.P().block<3, 3>(15, 15) = Eigen::Matrix3d::Identity() * 0.0001;
        kf.P().block<3, 3>(18, 18) = Eigen::Matrix3d::Identity() * 0.0001;
        kf.P().block<2, 2>(21, 21) = Eigen::Matrix2d::Identity() * 0.00001;
        data_group.last_imu = imus.back();
        return true;
    }

    void LIOBuilder::undistortCloud(SyncPackage &package)
    {
        data_group.imu_cache.clear();
        data_group.imu_cache.push_back(data_group.last_imu);
        data_group.imu_cache.insert(data_group.imu_cache.end(), package.imus.begin(), package.imus.end());

        const double imu_time_begin = data_group.imu_cache.front().timestamp;
        const double imu_time_end = data_group.imu_cache.back().timestamp;
        const double cloud_time_begin = package.cloud_start_time;
        const double cloud_time_end = package.cloud_end_time;
        std::sort(package.cloud->points.begin(), package.cloud->points.end(), [](pcl::PointXYZINormal &p1, pcl::PointXYZINormal &p2) -> bool
                  { return p1.curvature < p2.curvature; });

        data_group.imu_poses_cache.clear();
        data_group.imu_poses_cache.emplace_back(0.0, data_group.last_acc, data_group.last_gyro,
                                                kf.x().vel, kf.x().pos, kf.x().rot);

        Eigen::Vector3d acc_val, gyro_val;
        double dt = 0.0;
        kf::Input inp;

        for (auto it_imu = data_group.imu_cache.begin(); it_imu < (data_group.imu_cache.end() - 1); it_imu++)
        {
            IMUData &head = *it_imu;
            IMUData &tail = *(it_imu + 1);

            if (tail.timestamp < data_group.last_cloud_end_time)
                continue;
            gyro_val = 0.5 * (head.gyro + tail.gyro);
            acc_val = 0.5 * (head.acc + tail.acc);

            acc_val = acc_val * 9.81 / data_group.gravity_norm;

            if (head.timestamp < data_group.last_cloud_end_time)
                dt = tail.timestamp - data_group.last_cloud_end_time;
            else
                dt = tail.timestamp - head.timestamp;

            inp.acc = acc_val;
            inp.gyro = gyro_val;

            kf.predict(inp, dt, data_group.Q);

            data_group.last_gyro = gyro_val - kf.x().bg;
            data_group.last_acc = kf.x().rot * (acc_val - kf.x().ba) + kf.x().g;

            double offset = tail.timestamp - cloud_time_begin;
            data_group.imu_poses_cache.emplace_back(offset, data_group.last_acc, data_group.last_gyro, kf.x().vel, kf.x().pos, kf.x().rot);
        }

        dt = cloud_time_end - imu_time_end;
        kf.predict(inp, dt, data_group.Q);

        data_group.last_imu = package.imus.back();
        data_group.last_cloud_end_time = cloud_time_end;

        Eigen::Matrix3d cur_rot = kf.x().rot;
        Eigen::Vector3d cur_pos = kf.x().pos;
        Eigen::Matrix3d cur_rot_ext = kf.x().rot_ext;
        Eigen::Vector3d cur_pos_ext = kf.x().pos_ext;

        auto it_pcl = package.cloud->points.end() - 1;
        for (auto it_kp = data_group.imu_poses_cache.end() - 1; it_kp != data_group.imu_poses_cache.begin(); it_kp--)
        {
            auto head = it_kp - 1;
            auto tail = it_kp;

            Eigen::Matrix3d imu_rot = head->rot;
            Eigen::Vector3d imu_pos = head->pos;
            Eigen::Vector3d imu_vel = head->vel;
            Eigen::Vector3d imu_acc = tail->acc;
            Eigen::Vector3d imu_gyro = tail->gyro;

            for (; it_pcl->curvature / double(1000) > head->offset; it_pcl--)
            {
                dt = it_pcl->curvature / double(1000) - head->offset;
                Eigen::Vector3d point(it_pcl->x, it_pcl->y, it_pcl->z);
                Eigen::Matrix3d point_rot = imu_rot * Sophus::SO3d::exp(imu_gyro * dt).matrix();
                Eigen::Vector3d point_pos = imu_pos + imu_vel * dt + 0.5 * imu_acc * dt * dt;
                Eigen::Vector3d p_compensate = cur_rot_ext.transpose() * (cur_rot.transpose() * (point_rot * (cur_rot_ext * point + cur_pos_ext) + point_pos - cur_pos) - cur_pos_ext);
                it_pcl->x = p_compensate(0);
                it_pcl->y = p_compensate(1);
                it_pcl->z = p_compensate(2);

                if (it_pcl == package.cloud->points.begin())
                    break;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr LIOBuilder::lidarToWorld(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZINormal>);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = (kf.x().rot * kf.x().rot_ext).cast<float>();
        transform.block<3, 1>(0, 3) = (kf.x().rot * kf.x().pos_ext + kf.x().pos).cast<float>();
        pcl::transformPointCloud(*cloud, *cloud_world, transform);
        return cloud_world;
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr LIOBuilder::lidarToBody(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_body(new pcl::PointCloud<pcl::PointXYZINormal>);
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = kf.x().rot_ext.cast<float>();
        transform.block<3, 1>(0, 3) = kf.x().pos_ext.cast<float>();
        pcl::transformPointCloud(*cloud, *cloud_body, transform);
        return cloud_body;
    }

    void LIOBuilder::process(SyncPackage &package)
    {
        ROS_INFO_ONCE("LIOBuilder::process.");
        
        map->printInfo(false);
        

        /////////////////////////////////  my own FeatVoxelMap  /////////////////////////////////
        g_scan_cnt++;
        const int lidar_frequency = 10;
        if(g_scan_cnt == config.init_time * lidar_frequency){
            ROS_WARN_STREAM("Get scan: " << g_scan_cnt << ", now create my own FeatVoxelMap");
            map_p2v_->buildFeatVoxelMap(g_global_pc);
            map_p2v_->printInfo();
            map_p2v_->is_inited_ = true;
            // map_p2v_->saveToFile();
            // std::abort();
        }


        /////////////////////////////////  my own FeatVoxelMap  /////////////////////////////////


        if (status == LIOStatus::IMU_INIT)
        {
            if (initializeImu(package.imus))
            {
                status = LIOStatus::MAP_INIT;
                data_group.last_cloud_end_time = package.cloud_end_time;
                ROS_INFO("--> IMU Inited. State-> MAP_INIT");
            }
        }
        else if (status == LIOStatus::MAP_INIT)
        {   
            undistortCloud(package);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_world = lidarToWorld(package.cloud);
            std::vector<PointWithCov> pv_list;
            Eigen::Matrix3d r_wl = kf.x().rot * kf.x().rot_ext;
            Eigen::Vector3d p_wl = kf.x().rot * kf.x().pos_ext + kf.x().pos;

            std::vector<V3D> v_pts;

            for (size_t i = 0; i < point_world->size(); i++)
            {
                PointWithCov pv;
                pv.point = Eigen::Vector3d(point_world->points[i].x, point_world->points[i].y, point_world->points[i].z);
                Eigen::Vector3d point_body(package.cloud->points[i].x, package.cloud->points[i].y, package.cloud->points[i].z);
                Eigen::Matrix3d point_cov;
                calcBodyCov(point_body, config.ranging_cov, config.angle_cov, point_cov);
                Eigen::Matrix3d point_crossmat = Sophus::SO3d::hat(point_body);

                point_cov = r_wl * point_cov * r_wl.transpose() +
                            point_crossmat * kf.P().block<3, 3>(kf::IESKF::R_ID, kf::IESKF::R_ID) * point_crossmat.transpose() +
                            kf.P().block<3, 3>(kf::IESKF::P_ID, kf::IESKF::P_ID);
                pv.cov = point_cov;
                pv_list.push_back(pv);

                V3D p_world = V3D(point_world->points[i].x, point_world->points[i].y, point_world->points[i].z);
                v_pts.push_back(p_world);
            }

            map->build(pv_list);

            status = LIOStatus::LIO_MAPPING;
            ROS_INFO("--> Map Inited. State-> LIO_MAPPING");

        }
        else        // LIOStatus::LIO_MAPPING
        {
            undistortCloud(package);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr world_ptr = lidarToWorld(package.cloud);
            if (config.scan_resolution > 0.0)       // 用于降采样。默认0.1m
            {
                scan_filter.setInputCloud(package.cloud);
                scan_filter.filter(*lidar_cloud);
            }
            else
            {
                pcl::copyPointCloud(*package.cloud, *lidar_cloud);
            }
            int size = lidar_cloud->size();

            vector<V3D> v_points_world;
            for (int i = 0; i < size; i++)
            {
                data_group.residual_info[i].point_lidar = Eigen::Vector3d(lidar_cloud->points[i].x, lidar_cloud->points[i].y, lidar_cloud->points[i].z);
                calcBodyCov(data_group.residual_info[i].point_lidar, config.ranging_cov, config.angle_cov, data_group.residual_info[i].cov_lidar);
            }
            
            // In this function, calculate the residual and update the state.
            bool use_p2v = false;
            if(map_p2v_->is_inited_){        // if inited, switch to p2v prediction.
                use_p2v = true;
                ROS_WARN_ONCE("--> Using Switch to p2v prediction...");
            }
            else{
                ROS_WARN("--> Waiting for init. Using original prediction.");
            }

            my_viewer.reset();
            kf.update(false, my_viewer);
            // TODO: Get p2v, and p2plane direction from kf.update.
            // TODO: Set kf a new value to test the direction.
            my_viewer.publishPointAndMatch(package.cloud_end_time);


            pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_world = lidarToWorld(lidar_cloud);
            std::vector<PointWithCov> pv_list;
            std::vector<V3D> v_list;                // save my own format.
            Eigen::Matrix3d r_wl = kf.x().rot * kf.x().rot_ext;
            Eigen::Vector3d p_wl = kf.x().rot * kf.x().pos_ext + kf.x().pos;
            for (int i = 0; i < size; i++)
            {
                PointWithCov pv;
                pv.point = Eigen::Vector3d(point_world->points[i].x, point_world->points[i].y, point_world->points[i].z);
                Eigen::Matrix3d cov = data_group.residual_info[i].cov_lidar;
                Eigen::Matrix3d point_crossmat = Sophus::SO3d::hat(data_group.residual_info[i].point_lidar);
                pv.cov = r_wl * cov * r_wl.transpose() +
                         point_crossmat * kf.P().block<3, 3>(kf::IESKF::R_ID, kf::IESKF::R_ID) * point_crossmat.transpose() +
                         kf.P().block<3, 3>(kf::IESKF::P_ID, kf::IESKF::P_ID);
                pv_list.push_back(pv);
                
                v_list.push_back(pv.point);
            }
            map->update(pv_list);

            // My featmap update
            if(map_p2v_->is_inited_){
                // ROS_WARN("map_p2v_ update");
                // map_p2v_->printInfo("Before update.");
                map_p2v_->update(v_list);
                // ROS_WARN("map_p2v_ update done");
                // map_p2v_->printInfo("After update.");
            }
            

        }
    }

    //~ Calculate residual, and H, b
    //~ only `update_viewer` for the first iteration
    void LIOBuilder::sharedUpdateFunc(const kf::State &state, kf::SharedState &shared_state, ScanRegisterViewer& my_viewer, bool update_viewer)
    {
        Eigen::Matrix3d r_wl = state.rot * state.rot_ext;
        Eigen::Vector3d p_wl = state.rot * state.pos_ext + state.pos;
        int size = lidar_cloud->size();

        std::cout << "Input pc size: " << size << std::endl;

// #define MP_EN
#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (int i = 0; i < size; i++)
        {   
            data_group.residual_info[i].point_world = r_wl * data_group.residual_info[i].point_lidar + p_wl;
            
            Eigen::Vector3d p_world = data_group.residual_info[i].point_world;
            pcl::PointXYZINormal p_new;
            p_new.x = p_world[0];
            p_new.y = p_world[1];
            p_new.z = p_world[2];

            if(update_viewer){
                my_viewer.pc_world_.points.push_back(p_new);        // save all points
            }

            Eigen::Matrix3d point_crossmat = Sophus::SO3d::hat(data_group.residual_info[i].point_lidar);
            data_group.residual_info[i].cov_world = r_wl * data_group.residual_info[i].cov_lidar * r_wl.transpose() +
                                                    point_crossmat * kf.P().block<3, 3>(kf::IESKF::R_ID, kf::IESKF::R_ID) * point_crossmat.transpose() +
                                                    kf.P().block<3, 3>(kf::IESKF::P_ID, kf::IESKF::P_ID);
            VoxelKey position = map->index(data_group.residual_info[i].point_world);
            auto iter = map->featmap.find(position);
            if (iter != map->featmap.end())         //~ find query-point's corresponding voxel in map
            {
                // Paper, Section III.C eq(9), calculate `residual`, in residual_info.  
                map->buildResidual(data_group.residual_info[i], iter->second);      //~ `iter->second` is query-point's voxel
                
                // save to viewer.
                if(update_viewer){
                    ResidualData data = data_group.residual_info[i];
                    if(!data.is_valid)      // skip not-valid scans. 1) not plane; 2) too large p2plane
                        continue;
                    Eigen::Vector3d p2m = data.plane_mean - data.point_world;
                    Eigen::Vector3d p2plane = p2m.dot(data.plane_norm) * data.plane_norm;
                    my_viewer.p2plane_.push_back(p2plane);
                    my_viewer.pc_world_in_voxel_.points.push_back(p_new);
                }
            }
            else{
                continue;
            }
        }
        // cout << "''''''''''''''''''''''''''''''''" << endl;
        // cout << my_viewer.pc_world_in_voxel_.points.size() << endl;

        shared_state.H.setZero();
        shared_state.b.setZero();
        Eigen::Matrix<double, 1, 12> J;     // J, eq(13), but not exactly same. J's state: [pos, rot, ext_pos, ext_rot]
        Eigen::Matrix<double, 1, 6> Jn;     // Jn, eq(11): [(w_p − q)T , −nT]

        int effect_num = 0;

        for (int i = 0; i < size; i++)
        {
            if (!data_group.residual_info[i].is_valid)  // 只有对应voxel是平面时，才是true
                continue;
            effect_num++;
            J.setZero();

            Eigen::Vector3d plane_norm = data_group.residual_info[i].plane_norm;            //~ plane_norm  --> P2V

            // Jn, eq(11)'s first two (1x3 vector) part, corresponding to \Sigma_{ni, qi}
            Jn.block<1, 3>(0, 0) = (data_group.residual_info[i].point_world - data_group.residual_info[i].plane_mean).transpose();
            Jn.block<1, 3>(0, 3) = -plane_norm.transpose();
            
            //~ r_cov  -->  P2V's weight
            // r_cov is \Sigma_{wi} in eq(10), di~N{0, Sigma_{wi}}. Sigma_{wi} has 2/3 part: Sigma_{ni,qi} (plane uncertainty) and Sigma_p (point uncertainty).
            double r_cov = Jn * data_group.residual_info[i].plane_cov * Jn.transpose();                                         // calculate Jn*Sigma_{ni,qi}*Jn', the first 2 part
            r_cov += plane_norm.transpose() * r_wl * data_group.residual_info[i].cov_lidar * r_wl.transpose() * plane_norm;     // calculate n^T * Sigma_p * n

            double r_info = r_cov < 0.0002 ? 5000 : 1.0 / r_cov;            // r_info = 1/r_cov --> information matrix = 1/covariance

            // cout << "[Debug] r_info for original method: " << r_info << endl;        // 0.0x 到 5000 都出现过
            J.block<1, 3>(0, 0) = plane_norm.transpose();
            J.block<1, 3>(0, 3) = -plane_norm.transpose() * state.rot * Sophus::SO3d::hat(state.rot_ext * data_group.residual_info[i].point_lidar + state.pos_ext);
            if (config.estimate_ext)
            {
                J.block<1, 3>(0, 6) = -plane_norm.transpose() * r_wl * Sophus::SO3d::hat(data_group.residual_info[i].point_lidar);
                J.block<1, 3>(0, 9) = plane_norm.transpose() * state.rot;
            }
            shared_state.H += J.transpose() * r_info * J;
            shared_state.b += J.transpose() * r_info * data_group.residual_info[i].residual;
        }
        if (effect_num < 1)
            std::cout << "NO EFFECTIVE POINT";
        // std::cout << "==================: " << effect_num << std::endl;
    }

    /////////////////////////////////////////////////////////////////////////////////  P2V  /////////////////////////////////////////////////////////////////////////////////
    // new update function using FeatVoxelMap providing p2v and cov.
    void LIOBuilder::sharedUpdateFunc_p2v(const kf::State &state, kf::SharedState &shared_state, ScanRegisterViewer& my_viewer, bool update_viewer){       // `func_p2v_`

        ROS_WARN_ONCE("Call `sharedUpdateFunc_p2v` (this only output once)");

        my_ros_utility::MyTimer timer;
        timer.tic();

        Eigen::Matrix3d r_wl = state.rot * state.rot_ext;
        Eigen::Vector3d p_wl = state.rot * state.pos_ext + state.pos;
        int lidar_points_size = lidar_cloud->size();

        const int BATCH_SIZE = config.batch_size;

        vector<Eigen::Vector3d> batch_queries;
        vector<vector<Eigen::Vector3d>> batch_voxel_points;
        vector<ResidualData*> batch_residuals;
        vector<std::unordered_map<VoxelKey, std::shared_ptr<FeatVoxelGrid>, VoxelKey::Hasher>::iterator> batch_iters;       // save iter, for color assign. (so urgly)

        int do_prediction_cnt = 0;                          // record how many times the prediction is called. (not the "valid" prediction)
        int skip_cnt = 0;
        #pragma omp parallel for                            // parallel computation.
        for(int i=0; i<lidar_points_size; ++i){
            data_group.residual_info[i].point_world = r_wl * data_group.residual_info[i].point_lidar + p_wl;
            // change to my new feat_map
            VoxelKey position = map_p2v_->calcVoxelKey(data_group.residual_info[i].point_world);
            auto iter = map_p2v_->my_featmap_.find(position);
            if (iter != map_p2v_->my_featmap_.end()){

#define USING_BATCH
#ifndef USING_BATCH
                // TODO: One-time prediction
                //~ get p2v, weight. If not enough points, no-prediction; enough points, predict. weight>threshold, data.is_valid=true.
                bool has_predicted = map_p2v_->buildResidualByPointnet(data_group.residual_info[i], iter->second);
                if(has_predicted)
                    do_prediction_cnt++;
                // weight is assgined back to voxels. Only for NO-BATCH prediction, because BATCH prediction is complex to assign weight back.
                const double weight_lower_bound = map_p2v_->valid_weight_threshold_;
                iter->second->voxel_weight_ = (data_group.residual_info[i].weight - weight_lower_bound)/(1-weight_lower_bound); // normalize the voxel_weight
#else

                // Change to batch-predicting.
                std::shared_ptr<FeatVoxelGrid> voxel_grid = iter->second;
                ResidualData* res = &(data_group.residual_info[i]);
                res->is_valid = false;                              // set to `true` if: enough points in voxel, and weight > threshold

                if(voxel_grid->temp_points_.size() < voxel_grid->extract_feat_threshold_)       // skip small voxels.
                    continue;

                if(skip_cnt++ % config.prediction_skip != 0)            // skip some points to speed-up
                    continue;   

                V3D query_point;
                query_point = res->point_world - voxel_grid->lower_boundary_;

                // normalize the points.        TODO: normalize only once. Not verified
                // std::vector<V3D> points;
                // points.reserve(voxel_grid->temp_points_.size());
                // for (auto p : voxel_grid->temp_points_){
                //     points.push_back(p - voxel_grid->lower_boundary_);
                // }

                batch_queries.push_back(query_point);
                batch_voxel_points.push_back(voxel_grid->normalized_points_);
                batch_residuals.push_back(res);
                batch_iters.push_back(iter);
#endif
            }
            
            // calculate the weight/residual if we got a BATCH size of voxel.
            if(batch_queries.size() >= BATCH_SIZE){
                do_prediction_cnt += BATCH_SIZE;

                vector<V3D> batch_p2v_pred(BATCH_SIZE);
                vector<double> batch_weight(BATCH_SIZE);
                
                my_ros_utility::MyTimer timer;
                timer.tic();
                map_p2v_->p2v_model_.batchPredictP2V(batch_voxel_points, batch_queries, batch_p2v_pred, batch_weight, BATCH_SIZE);
                double t = timer.toc();

                // cout << "Predict a batch ( " << BATCH_SIZE <<" ), total time: " << t << "s." << endl;
                // assign `is_valid` based on the weight.
                for(int index_w=0; index_w < BATCH_SIZE; ++index_w){
                    if(batch_weight[index_w] > map_p2v_->valid_weight_threshold_)
                        batch_residuals[index_w]->is_valid = true;
                    batch_iters[index_w]->second->voxel_weight_ = batch_weight[index_w];        // assign weight color back to voxels.
                }

                // clear the batch
                batch_queries.clear();
                batch_voxel_points.clear();
                batch_residuals.clear();
                batch_iters.clear();
            }

        }
        double t0 = timer.toc();

        ///////////////////////////////////////////////// Check Build residual result /////////////////////////////////////////////////
        int valid_residual_cnt = 0;
        for(int i=0; i<data_group.residual_info.size(); ++i){
            const ResidualData& res = data_group.residual_info[i];
            if(res.is_valid){
                valid_residual_cnt++;
            }
        }
        cout << "[sharedUpdateFunc_p2v] Total residual number: " << lidar_points_size <<", residual calculation time: " << t0 << endl;
        cout << "[sharedUpdateFunc_p2v] Total prediction cnt : " << do_prediction_cnt << ", average prediction time: " << t0 / do_prediction_cnt << endl;
        cout << "[sharedUpdateFunc_p2v] Valid residual number: " << valid_residual_cnt << endl;
        /////////////////////////////////////////////////


        shared_state.H.setZero();
        shared_state.b.setZero();
        Eigen::Matrix<double, 1, 12> J;     // J, eq(13), but not exactly same. J's state: [pos, rot, ext_pos, ext_rot]
        // Eigen::Matrix<double, 1, 6> Jn;      // Jn is not necessary, which cov is provided by model.

        int effect_num = 0;

        for (int i = 0; i < lidar_points_size; i++)
        {
            if (!data_group.residual_info[i].is_valid)  // 只有对应voxel是平面时，才是true
                continue;
            effect_num++;
            J.setZero();
            
            double weight = data_group.residual_info[i].weight;     // weight: 0.8 ~ 1, if valid.
            // double r_info = (weight-0.8)*5; // normalized to (0,1)
            double fen_mu = (1-map_p2v_->valid_weight_threshold_);
            double fen_zi = weight - map_p2v_->valid_weight_threshold_;
            assert(fen_zi > 0);
            double r_info = fen_zi / fen_mu;    // normalized to (0,1)
            
            r_info = r_info * 5000;         // scale-up. TODO: nor verified.

            J.block<1, 3>(0, 0) = data_group.residual_info[i].p2v.transpose();
            J.block<1, 3>(0, 3) = -data_group.residual_info[i].p2v.transpose() * state.rot * Sophus::SO3d::hat(state.rot_ext * data_group.residual_info[i].point_lidar + state.pos_ext);
            // if (config.estimate_ext)
            // {
            //     J.block<1, 3>(0, 6) = -plane_norm.transpose() * r_wl * Sophus::SO3d::hat(data_group.residual_info[i].point_lidar);
            //     J.block<1, 3>(0, 9) = plane_norm.transpose() * state.rot;
            // }
            shared_state.H += J.transpose() * r_info * J;
            double residual = data_group.residual_info[i].p2v.norm();
            // cout << "P2v residual: " << residual << endl;
            shared_state.b += J.transpose() * r_info * residual;
        }
        if (effect_num < 1)
            std::cout << "NO EFFECTIVE POINT" << std::endl;
        // std::cout << "==================: " << effect_num << std::endl;
    }
}