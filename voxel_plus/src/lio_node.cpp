#include <ros/ros.h>
#include <queue>
#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/lio_builder.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include "interface/PointCloudWithOdom.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

// for offline rosbag play.
#include <rosbag/bag.h>
#include <rosbag/view.h>
#define OFFLINE_ROSBAG      // cmusing offline rosbag loader.

#include <iostream>
#include "map_builder/my_viewer.h"

#include <random>


pcl::PointCloud<pcl::PointXYZINormal>::Ptr g_global_pc(new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr g_fullvoxelmap_pc(new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::VoxelGrid<pcl::PointXYZINormal> g_ds_filter;
ScanRegisterViewer my_viewer;

double g_ros_running_time, g_slam_init_time=-1;



inline void addScanToWorldMap(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& point_world, float leaf_size = 0.05f){
    *g_global_pc += *point_world;
    g_ds_filter.setInputCloud(g_global_pc);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    g_ds_filter.filter(*temp_cloud);
    g_global_pc->swap(*temp_cloud);
}


struct NodeConfig
{
    std::string lidar_topic;
    std::string imu_topic;
    std::string map_frame;
    std::string body_frame;
    double range_min = 0.5;
    double range_max = 20.0;
    int filter_num = 3;
    bool publish_voxel_map = false;
    int publish_voxel_num = 1000;
};

struct NodeGroupData
{
    double last_imu_time = 0.0;
    double last_lidar_time = 0.0;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    std::deque<lio::IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    bool lidar_pushed = false;
};

class LIONode
{
public:
    LIONode(ros::NodeHandle& nh_input) : nh(nh_input)
    {
        loadConfig();
        initPublishers();
        map_builder.loadConfig(lio_config);
        
        // use offline rosbag play based on configure.
        if(lio_config.use_offline_rosbag){
            traj_out_file_.open(trajectory_save_filename_, ios::out);
            if (traj_out_file_.is_open()){
                cout << "Trajectory will save into: " << trajectory_save_filename_ << endl;
            }
            else
                ROS_ERROR_STREAM("Error. Cannot open file to save trajectory: " << trajectory_save_filename_);
            
            initRosbagPlayerAndRun();
        }
        else{
            initSubScribers();
            main_loop = nh.createTimer(ros::Duration(0.02), &LIONode::mainCB, this);
            voxel_map_loop = nh.createTimer(ros::Duration(5.0), &LIONode::voxelTimerCB, this, false, false);        // CB: CallBack
        }

        // 这行代码会启动之前创建的定时器 voxel_map_loop。启动后，定时器开始按照设定的 5 秒间隔调用回调函数 LIONode::voxelTimerCB
        // This loop shows the marker at a low frequency.
        // if (config.publish_voxel_map)
        //     voxel_map_loop.start();
    }

    void loadConfig()
    {
        nh.param<std::string>("lidar_topic", config.lidar_topic, "/livox/lidar");
        nh.param<std::string>("imu_topic", config.imu_topic, "/livox/imu");

        nh.param<std::string>("body_frame", config.body_frame, "body");
        nh.param<std::string>("map_frame", config.map_frame, "map");

        nh.param<int>("filter_num", config.filter_num, 3);
        nh.param<double>("range_min", config.range_min, 0.5);
        nh.param<double>("range_max", config.range_max, 20.0);
        nh.param<bool>("publish_voxel_map", config.publish_voxel_map, false);
        nh.param<int>("publish_voxel_num", config.publish_voxel_num, 1000);

        nh.param<double>("scan_resolution", lio_config.scan_resolution, 0.1);
        nh.param<double>("voxel_size", lio_config.voxel_size, 0.4);

        nh.param<double>("merge_thresh_for_angle", lio_config.merge_thresh_for_angle, 0.1);
        nh.param<double>("merge_thresh_for_distance", lio_config.merge_thresh_for_distance, 0.04);

        nh.param<int>("max_point_thresh", lio_config.max_point_thresh, 100);
        nh.param<int>("update_size_thresh", lio_config.update_size_thresh, 10);

        nh.param<bool>("use_offline_rosbag", lio_config.use_offline_rosbag, false);
        nh.param<string>("offline_rosbag_file", lio_config.offline_rosbag_file, "default.bag");

        nh.param<string>("trajectory_save_filename", trajectory_save_filename_, "/default/trajectory.txt");



        nh.param<int>("map_capacity", lio_config.map_capacity, 100000);
        nh.param<bool>("gravity_align", lio_config.gravity_align, true);
        nh.param<int>("imu_init_num", lio_config.imu_init_num, 20);
        nh.param<double>("na", lio_config.na, 0.01);
        nh.param<double>("ng", lio_config.ng, 0.01);
        nh.param<double>("nbg", lio_config.nbg, 0.0001);
        nh.param<double>("nba", lio_config.nba, 0.0001);
        nh.param<int>("opti_max_iter", lio_config.opti_max_iter, 5);
        std::vector<double> r_il, p_il;
        nh.param<std::vector<double>>("r_il", r_il, std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
        assert(r_il.size() == 9);
        lio_config.r_il << r_il[0], r_il[1], r_il[2], r_il[3], r_il[4], r_il[5], r_il[6], r_il[7], r_il[8];

        nh.param<std::vector<double>>("p_il", p_il, std::vector<double>{0, 0, 0});
        assert(p_il.size() == 3);
        lio_config.p_il << p_il[0], p_il[1], p_il[2];


        // Load my own parameters.
        nh.param<std::string>("model_file", lio_config.model_file, "/default/model.pt");
        nh.param<double>("init_time", lio_config.init_time, 2.0);
        nh.param<double>("valid_weight_threshold", lio_config.valid_weight_threshold, 0.8);
        nh.param<int>("batch_size", lio_config.batch_size, 4);
        nh.param<int>("prediction_skip", lio_config.prediction_skip, 1);
        nh.param<double>("r_info_scale", lio_config.r_info_scale, 1);






        ///////////////////////////////////////  Output config  ///////////////////////////////////////
        ROS_WARN("-------------------- CONFIG --------------------");
        ROS_INFO_STREAM("Model file: " << lio_config.model_file);
        ROS_INFO_STREAM("Map init time: " << lio_config.init_time <<" s.");
        ROS_INFO_STREAM("Valid weight threshold: " << lio_config.valid_weight_threshold);
        ROS_INFO_STREAM("Prediction batch size : " << lio_config.batch_size);
        ROS_INFO_STREAM("Prediction skip       : " << lio_config.prediction_skip);
        ROS_INFO_STREAM("Trajectory save into  : " << trajectory_save_filename_);
        
        if(lio_config.use_offline_rosbag)
            ROS_INFO_STREAM("Use offline rosbag! Bagfile: " << lio_config.offline_rosbag_file);
        ROS_WARN("-------------------- CONFIG --------------------");


    }

    void initSubScribers()
    {
        lidar_sub = nh.subscribe(config.lidar_topic, 10000, &LIONode::lidarCB, this);
        imu_sub = nh.subscribe(config.imu_topic, 10000, &LIONode::imuCB, this);
    }

    // Load a rosbag and play.
    // ISSUE: This player automatically start the main loop.
    void initRosbagPlayerAndRun(){
        const string rosbag_path = lio_config.offline_rosbag_file;
        ROS_WARN_STREAM("Load rosbag from: " << rosbag_path);
        try {
            rosbag::Bag bag(rosbag_path, rosbag::bagmode::Read);
            std::vector<std::string> topics = {config.lidar_topic, config.imu_topic};
            rosbag::View view(bag, rosbag::TopicQuery(topics));
            const double rosbag_begin_time = view.getBeginTime().toSec();
            
            ros::Time::init();  // 重要！确保ros时间系统初始化
            
            int frame_cnt = 0;
            for(const rosbag::MessageInstance& m : view) {

                if(!ros::ok()){      // stop when ctrl+C
                    traj_out_file_.close();
                    cout << "Exist. Save trajectory into: " << trajectory_save_filename_ << endl;
                    break;
                }

                if(m.getTopic() == config.lidar_topic) {
                    auto msg = m.instantiate<livox_ros_driver2::CustomMsg>();
                    if(msg) lidarCB(msg);
                }
                else if(m.getTopic() == config.imu_topic) {
                    auto msg = m.instantiate<sensor_msgs::Imu>();
                    if(msg) imuCB(msg);
                }
                
                // 处理同步和主循环
                ros::spinOnce();
                if(syncPackage()) {
                    processData();  // 将mainCB中的处理逻辑提取出来
                    double ros_current_time = sync_pack.cloud_start_time;
                    double dt = ros_current_time - rosbag_begin_time;
                    ROS_INFO_STREAM("--> Frame index: " << frame_cnt++ <<", rosbag time: " << dt << " s.");
                }
            }
            bag.close();
        } catch(rosbag::BagException& e) {
            ROS_ERROR("Failed to load rosbag: %s", e.what());
        }    
    }

    void initPublishers()
    {
        odom_pub = nh.advertise<nav_msgs::Odometry>("slam_odom", 1000);
        trajectory_pub = nh.advertise<nav_msgs::Path>("slam_trajectory", 1000);
        body_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("body_cloud", 1000);
        world_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("world_cloud", 1000);
        pub_full_map = nh.advertise<sensor_msgs::PointCloud2>("full_map", 1000);
        voxel_map_pub = nh.advertise<visualization_msgs::MarkerArray>("voxel_map", 1000);
        pointcloud_with_odom_pub = nh.advertise<interface::PointCloudWithOdom>("cloud_with_odom", 1000);

        pub_featvoxelmap_ = nh.advertise<sensor_msgs::PointCloud2>("feat_voxel_map", 1000);
    }

    void imuCB(const sensor_msgs::Imu::ConstPtr msg)
    {
        g_ros_running_time = msg->header.stamp.toSec();
        if(g_slam_init_time < 0)        // init time.
            g_slam_init_time = g_ros_running_time;

        std::lock_guard<std::mutex> lock(group_data.imu_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < group_data.last_imu_time)
        {
            ROS_WARN("IMU TIME SYNC ERROR");
            group_data.imu_buffer.clear();
        }
        group_data.last_imu_time = timestamp;
        group_data.imu_buffer.emplace_back(Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                                           Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                           timestamp);
    }

    // livox custom msg.
    void lidarCB(const livox_ros_driver2::CustomMsg::ConstPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        livox2pcl(msg, cloud, config.filter_num, config.range_min, config.range_max);
        std::lock_guard<std::mutex> lock(group_data.lidar_mutex);
        double timestamp = msg->header.stamp.toSec();
        if (timestamp < group_data.last_lidar_time)
        {
            ROS_WARN("LIDAR TIME SYNC ERROR");
            group_data.lidar_buffer.clear();
        }
        group_data.last_lidar_time = timestamp;
        group_data.lidar_buffer.emplace_back(timestamp, cloud);
    }


    bool syncPackage()
    {
        if (group_data.imu_buffer.empty() || group_data.lidar_buffer.empty())
            return false;
        // 同步点云数据
        if (!group_data.lidar_pushed)
        {
            sync_pack.cloud = group_data.lidar_buffer.front().second;
            sync_pack.cloud_start_time = group_data.lidar_buffer.front().first;
            sync_pack.cloud_end_time = sync_pack.cloud_start_time + sync_pack.cloud->points.back().curvature / double(1000.0);
            group_data.lidar_pushed = true;
        }
        // 等待IMU的数据
        if (group_data.last_imu_time < sync_pack.cloud_end_time)
            return false;

        sync_pack.imus.clear();

        // 同步IMU的数据
        // IMU的最后一帧数据的时间小于点云最后一个点的时间
        while (!group_data.imu_buffer.empty() && (group_data.imu_buffer.front().timestamp < sync_pack.cloud_end_time))
        {
            sync_pack.imus.push_back(group_data.imu_buffer.front());
            group_data.imu_buffer.pop_front();
        }
        group_data.lidar_buffer.pop_front();
        group_data.lidar_pushed = false;
        return true;
    }


    void mainCB(const ros::TimerEvent &e)
    {
        if (!syncPackage())
            return;
        processData();
    }

    void processData(){
        
        // MAIN FUNCTION
        map_builder.process(sync_pack);

        if (map_builder.status != lio::LIOStatus::LIO_MAPPING)
            return;
        state = map_builder.kf.x();

        br.sendTransform(eigen2Transform(state.rot, state.pos, config.map_frame, config.body_frame, sync_pack.cloud_end_time));
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr body_cloud = map_builder.lidarToBody(sync_pack.cloud);
        // publishCloud(body_cloud_pub, body_cloud, config.body_frame, sync_pack.cloud_end_time);
        publishCloud(body_cloud_pub, body_cloud, config.map_frame, sync_pack.cloud_end_time);       //~ change to map_frame, because body_cloud is in global 'map' frame
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr world_cloud = map_builder.lidarToWorld(sync_pack.cloud);
        publishCloud(world_cloud_pub, world_cloud, config.map_frame, sync_pack.cloud_end_time);
        
        // publish full pointcloud
        addScanToWorldMap(world_cloud);     // add world_cloud to g_global_pc, and then downsampled.
        publishCloud(pub_full_map, g_global_pc, config.map_frame, sync_pack.cloud_end_time);


        //////////////////////////////////////////////////////  publish full featvoxelmap  //////////////////////////////////////////////////////
        g_fullvoxelmap_pc->points.resize(0);
        auto feat_map = map_builder.map_p2v_->my_featmap_;
        for(const auto& pair : feat_map){
            int n_points = pair.second->temp_points_.size();
            double intensity = 0.0f;            // intensity: 0.25, 0.5, 0.75, 1

            // intensity by number of points.
            intensity = float(n_points) / 100.0f;
            if (intensity>1)
                intensity = 1;

            // intensity by weight.
            intensity = pair.second->voxel_weight_;

            // intensity by random color
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<float> dis(0.0f, 1.0f);
            intensity = dis(gen);

            for(const auto& v3d_point : pair.second->temp_points_){
                pcl::PointXYZINormal point;
                point.x = v3d_point[0];
                point.y = v3d_point[1];
                point.z = v3d_point[2];
                point.intensity = intensity;
                g_fullvoxelmap_pc->points.push_back(point);
            }
        }
        pub_featvoxelmap_.publish(pcl2msg(g_fullvoxelmap_pc, config.map_frame, sync_pack.cloud_end_time));

        // publish the odometry
        publishOdom(config.map_frame, sync_pack.cloud_end_time);

        // publish the trajectory
        publishTrajectory(config.map_frame, sync_pack.cloud_end_time);

        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        

        
        // this message type cannot be viewed by rviz.
        publishCloudWithOdom(body_cloud, config.map_frame, config.body_frame, sync_pack.cloud_end_time);

        
        //~ Save trajectory into files.
        double current_ts = sync_pack.cloud_end_time;
        Eigen::Vector3d pos = state.pos;
        Eigen::Quaterniond quat(state.rot);

        traj_out_file_ << std::fixed << std::setprecision(4) << current_ts << " "
                    << pos[0] << " "
                    << pos[1] << " "
                    << pos[2] << " "
                    << quat.x() << " "
                    << quat.y() << " "
                    << quat.z() << " "
                    << quat.w() << endl;
    }


    void voxelTimerCB(const ros::TimerEvent &event)
    {
        std::shared_ptr<lio::VoxelMap> voxel_map = map_builder.map;
        if (voxel_map->cache.size() < 10)
            return;
        if (voxel_map_pub.getNumSubscribers() < 1)
            return;
        voxel_map_pub.publish(voxel2MarkerArray(voxel_map, config.map_frame, ros::Time::now().toSec(), config.publish_voxel_num, voxel_map->voxel_size * 0.8));
    }

    void publishCloud(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, std::string &frame_id, const double &time)
    {
        if (pub.getNumSubscribers() < 1)
            return;
        pub.publish(pcl2msg(cloud, frame_id, time));
    }


    //~ publish odom only
    void publishOdom(const std::string& frame_id, const double &time){
        nav_msgs::Odometry odom_msg;

        odom_msg.header.stamp = ros::Time().fromSec(time);
        odom_msg.header.frame_id = frame_id;

        // Position
        odom_msg.pose.pose.position.x = state.pos.x();
        odom_msg.pose.pose.position.y = state.pos.y();
        odom_msg.pose.pose.position.z = state.pos.z();
        
        Eigen::Quaterniond q(state.rot);

        // Orientation
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();

        odom_pub.publish(odom_msg);
    
    }

    void publishTrajectory(const std::string& frame_id, const double& time) {
        // 1. 创建位姿点
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(time);
        pose_stamped.header.frame_id = frame_id;  // 使用全局坐标系
        
        // 2. 填充位置
        pose_stamped.pose.position.x = state.pos.x();
        pose_stamped.pose.position.y = state.pos.y();
        pose_stamped.pose.position.z = state.pos.z();
        
        // 3. 填充姿态（直接复用 publishOdom 中的四元数）
        Eigen::Quaterniond q(state.rot);
        pose_stamped.pose.orientation.w = q.w();
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        
        // 4. 添加到路径
        path_msg.poses.push_back(pose_stamped);
        path_msg.header.stamp = ros::Time::now();  // 更新路径时间戳
        path_msg.header.frame_id = frame_id;
        
        // 5. 发布轨迹
        trajectory_pub.publish(path_msg);
    }
    


    void publishCloudWithOdom(pcl::PointCloud<pcl::PointXYZINormal>::Ptr _cloud, std::string &_frame_id, std::string &_child_frame, double _timestamp)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pub_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*_cloud, *pub_cloud);

        if (pointcloud_with_odom_pub.getNumSubscribers() < 1)
            return;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*pub_cloud, cloud_msg);
        if (_timestamp < 0)
            cloud_msg.header.stamp = ros::Time().now();
        else
            cloud_msg.header.stamp = ros::Time().fromSec(_timestamp);
        cloud_msg.header.frame_id = _child_frame;
        
        interface::PointCloudWithOdom msg;
        msg.header.stamp = ros::Time().fromSec(_timestamp);
        msg.header.frame_id = _frame_id;
        msg.cloud = cloud_msg;
        msg.pose.pose.position.x = state.pos.x();
        msg.pose.pose.position.y = state.pos.y();
        msg.pose.pose.position.z = state.pos.z();
        Eigen::Quaterniond q(state.rot);
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg.pose.covariance.data()) = map_builder.kf.P().block<6, 6>(0, 0);
        pointcloud_with_odom_pub.publish(msg);
    }

public:
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster br;
    NodeConfig config;
    NodeGroupData group_data;
    lio::SyncPackage sync_pack;
    ros::Timer main_loop;
    ros::Timer voxel_map_loop;

    std::string map_frame;
    std::string body_frame;

    ros::Subscriber lidar_sub;
    ros::Subscriber imu_sub;

    ros::Publisher odom_pub;
    ros::Publisher trajectory_pub;
    ros::Publisher body_cloud_pub;
    ros::Publisher world_cloud_pub;

    ros::Publisher pub_full_map;

    ros::Publisher voxel_map_pub;
    ros::Publisher pointcloud_with_odom_pub;

    lio::LIOBuilder map_builder;

    lio::LIOConfig lio_config;

    kf::State state;

    ros::Publisher pub_featvoxelmap_;

    //~ trajectory saver
    nav_msgs::Path path_msg;
    std::string trajectory_save_filename_;
    std::ofstream traj_out_file_;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_node");
    ros::NodeHandle nh("~");

    // init ds filter
    double leaf_size = 0.05;
    g_ds_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

    double line_width=0.1;
    int skip_cnt = 10;
    nh.param<int>("skip_cnt", skip_cnt, 1);
    nh.param<double>("line_width", line_width, 0.1);

    my_viewer.initViewer(nh, line_width, skip_cnt);

    LIONode lio_node(nh);
    ros::spin();
    return 0;
}


