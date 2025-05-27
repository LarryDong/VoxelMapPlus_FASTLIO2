
#include "my_viewer.h"
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/////////////////////////////////////         Viewer

void ScanRegisterViewer::initViewer(ros::NodeHandle& nh){
    pub_scan_before_ikf_ = nh.advertise<sensor_msgs::PointCloud2>("/scan_before_ikf", 1000);
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("/match_marker", 1000);

}


void ScanRegisterViewer::setScanPoint(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr){
    pc_world_.clear();
    pc_world_.width = ptr->width;
    pc_world_.height = ptr->height;

    for (const auto& pt : *ptr) {
        pcl::PointXYZINormal new_pt;
        new_pt.x = pt.x;
        new_pt.y = pt.y;
        new_pt.z = pt.z;
        new_pt.intensity = pt.intensity;
        // 法向量字段（normal_x/normal_y/normal_z）需要另外计算或置零
        new_pt.normal_x = 0;
        new_pt.normal_y = 0;
        new_pt.normal_z = 0;
        pc_world_.push_back(new_pt);
    }

    ROS_WARN_STREAM("RegisterViewer size: " << pc_world_.points.size());
}


visualization_msgs::Marker createMarker(int id, int type, const std::string& frame_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
}



void ScanRegisterViewer::publishPointAndMatch(double timestamp){

    // check data size.
    ROS_WARN("Check data size.");
    cout <<"full point cloud size: " << pc_world_.points.size() << endl;
    cout <<"p2plane size         : " << p2plane_.size() << endl;
    cout <<"p2v size             : " << p2v_.size() << endl;
    // TODO:

    double N = pc_world_.points.size();
    
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pc_world_, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time().fromSec(timestamp);
    pub_scan_before_ikf_.publish(msg);

    // show marker
    
    int id_idx = 0;

    // clean the marker.
    visualization_msgs::MarkerArray clear_markers;
    visualization_msgs::Marker clear_marker;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    clear_marker.header.frame_id = "map";
    clear_markers.markers.push_back(clear_marker);
    pub_marker_.publish(clear_markers);

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker lines = createMarker(id_idx, visualization_msgs::Marker::LINE_LIST, "map");
    lines.scale.x = 0.05; // Line width
    lines.color.r = 1;
    lines.color.g = 0;
    lines.color.b = 0;
    lines.color.a = 1;


    const int skip_cnt = 10;
    for(int i=0; i<N; ++i){
        // skip not valid p2plane
        if(p2plane_valid_[i]!= true || point_in_voxel_[i]!= true) 
            continue;

        if(i%skip_cnt == 0)
            continue;

        Eigen::Vector3d scan_point;
        pcl::PointXYZINormal pcl_point = pc_world_.points[i];
        scan_point[0] = pcl_point.x;
        scan_point[1] = pcl_point.y;
        scan_point[2] = pcl_point.z;

        Eigen::Vector3d plane_point = scan_point + p2plane_[i];
        geometry_msgs::Point msg_scan_point, msg_plane_point;
        msg_scan_point.x = scan_point[0];
        msg_scan_point.y = scan_point[1];
        msg_scan_point.z = scan_point[2];

        msg_plane_point.x = plane_point[0];
        msg_plane_point.y = plane_point[1];
        msg_plane_point.z = plane_point[2];
        lines.points.push_back(msg_scan_point);
        lines.points.push_back(msg_plane_point);
    }

    marker_array.markers.push_back(lines);
    cout << "Marker number: " << lines.points.size() << endl;

    pub_marker_.publish(marker_array);
}


// void setP2Plane(const std::vector<Eigen::Vector3d>& p2p);
// void setP2Voxel(const std::vector<Eigen::Vector3d>& p2v);

void ScanRegisterViewer::setP2Plane(const std::vector<Eigen::Vector3d>& p2p){
    p2plane_.reserve(p2p.size());
    p2plane_.clear();
    for(auto p:p2p){
        p2plane_.push_back(p);
    }
}


void ScanRegisterViewer::reset(void){
    Eigen::Vector3d tmp;
    p2plane_.resize(0);
    p2v_.resize(0);
    pc_world_.points.resize(0);
    point_in_voxel_.resize(0);
    p2plane_valid_.resize(0);
    p2v_valid_.resize(0);
}

