
#ifndef MY_VIEWER_H_
#define MY_VIEWER_H_

#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;


namespace tool{

inline void pclXYZIN2EigenV3d(const pcl::PointXYZINormal &pcl_point, Eigen::Vector3d &v3d){
    v3d << pcl_point.x, pcl_point.y, pcl_point.z;
}
inline void eigenV3d2GeomsgPoint(const Eigen::Vector3d &v3d, geometry_msgs::Point &point){
    point.x = v3d[0];
    point.y = v3d[1];
    point.z = v3d[2];
}

}



class ScanRegisterViewer{
    public:
        ScanRegisterViewer() = default;
        void initViewer(ros::NodeHandle& nh, double match_line_width, int skip_cnt);

        void publishPointAndMatch(double ts);

        inline void reset(void){
            Eigen::Vector3d tmp;
            p2plane_.resize(0);
            p2v_.resize(0);
            pc_world_.points.resize(0);
            pc_world_in_voxel_.points.resize(0);
            pc_world_in_voxel_p2v_.points.resize(0);
            is_good_p2v_.resize(0);
        }

        inline void setSaveFolder(const std::string& file_folder){
            save_folder_ = file_folder;
        }

        inline void saveP2V(int scan_idx, int query_idx, const Eigen::Vector3d& query, const std::vector<Eigen::Vector3d>&  voxel_points){
            string filename = save_folder_ + std::to_string(scan_idx) + "-" + to_string(query_idx) + ".csv";
            ofstream out(filename);
            if(!out.is_open()){
                cout << "can't open file: " << filename << endl;
                return ;
            }

            // voxel_points: 50 voxel points, 1 query point, 1 p2v.
            assert(voxel_points.size() == 52);
            for(int i=0; i<52; ++i){
                auto voxel = voxel_points[i];
                out << voxel[0] << "," << voxel[1] << "," << voxel[2] << endl;
                out << flush;
                // cout << voxel[0] << "," << voxel[1] << "," << voxel[2] << endl;
            }
            out.close();
        }

        void saveAllP2V(int scan_idx, bool is_p2v);

    public:
        pcl::PointCloud<pcl::PointXYZINormal> pc_world_;                    // scan using last-state
        pcl::PointCloud<pcl::PointXYZINormal> pc_world_in_voxel_;           // scan points near voxels
        pcl::PointCloud<pcl::PointXYZINormal> pc_world_in_voxel_p2v_;       // scan points near voxels, for p2v
        // std::vector<bool> p2plane_valid_, p2v_valid_;                       // if valid match.
        std::vector<Eigen::Vector3d> p2plane_;                      // point to plane
        std::vector<Eigen::Vector3d> p2v_;                          // point to voxel
        std::vector<bool> is_good_p2v_;
        ros::Publisher pub_scan_before_ikf_;                        // publish scan before the iKF iteration
        ros::Publisher pub_p2plane_marker_, pub_p2v_marker;         // publish markers, p2plane and p2v.
    
    private:
        double match_line_width_;
        int skip_cnt_;
        // std::ofstream file_output_;
        string save_folder_;

};



#endif
