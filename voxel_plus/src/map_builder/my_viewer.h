
#ifndef MY_VIEWER_H_
#define MY_VIEWER_H_

#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;


class ScanRegisterViewer{
    public:
        ScanRegisterViewer() = default;
        void initViewer(ros::NodeHandle& nh);
        void setScanPoint(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr);
        void setP2Plane(const std::vector<Eigen::Vector3d>& p2p);
        void setP2Voxel(const std::vector<Eigen::Vector3d>& p2v);
        void publishPointAndMatch(double ts);
        void reset(void);

    public:
        pcl::PointCloud<pcl::PointXYZINormal> pc_world_;
        std::vector<bool> point_in_voxel_, p2plane_valid_, p2v_valid_;
        std::vector<Eigen::Vector3d> p2plane_;
        std::vector<Eigen::Vector3d> p2v_;
        ros::Publisher pub_scan_before_ikf_;
        ros::Publisher pub_marker_;
    };



#endif
