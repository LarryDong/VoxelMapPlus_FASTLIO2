
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
        void initViewer(ros::NodeHandle& nh, double match_line_width, int skip_cnt);
        // void setScanPoint(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr);
        void setP2Plane(const std::vector<Eigen::Vector3d>& p2p);
        void setP2Voxel(const std::vector<Eigen::Vector3d>& p2v);
        void publishPointAndMatch(double ts);
        void reset(void);

    public:
        pcl::PointCloud<pcl::PointXYZINormal> pc_world_;                    // scan using last-state
        pcl::PointCloud<pcl::PointXYZINormal> pc_world_in_voxel_;           // scan points near voxels
        std::vector<bool> p2plane_valid_, p2v_valid_;                       // if valid match.
        std::vector<Eigen::Vector3d> p2plane_;                      // point to plane
        std::vector<Eigen::Vector3d> p2v_;                          // point to voxel
        ros::Publisher pub_scan_before_ikf_;                        // publish scan before the iKF iteration
        ros::Publisher pub_marker_;                                 // publish markers, p2plane and p2v.
    
    private:
        double match_line_width_;
        int skip_cnt_;
};



#endif
