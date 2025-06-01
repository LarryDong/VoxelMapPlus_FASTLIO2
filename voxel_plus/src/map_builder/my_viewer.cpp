
#include "my_viewer.h"
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

extern int g_scan_cnt;

/////////////////////////////////////         Viewer

void ScanRegisterViewer::initViewer(ros::NodeHandle& nh, double line_width, int skip_cnt){
    pub_scan_before_ikf_ = nh.advertise<sensor_msgs::PointCloud2>("/scan_before_ikf", 1000);
    pub_p2plane_marker_ = nh.advertise<visualization_msgs::MarkerArray>("/p2plane_marker", 1000);
    pub_p2v_marker = nh.advertise<visualization_msgs::MarkerArray>("/p2v_marker", 1000);
    match_line_width_ = line_width;
    skip_cnt_ = skip_cnt;

    ROS_WARN_STREAM("Init my viewer: ");
    cout << "line width: " << match_line_width_ << endl;
    cout << "skip_cnt: " << skip_cnt_ << endl;

    setSaveFolder("/home/larry/Desktop/p2v_debug/");
}


// void ScanRegisterViewer::setScanPoint(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr){
//     pc_world_.clear();
//     pc_world_.width = ptr->width;
//     pc_world_.height = ptr->height;

//     for (const auto& pt : *ptr) {
//         pcl::PointXYZINormal new_pt;
//         new_pt.x = pt.x;
//         new_pt.y = pt.y;
//         new_pt.z = pt.z;
//         new_pt.intensity = pt.intensity;
//         // 法向量字段（normal_x/normal_y/normal_z）需要另外计算或置零
//         new_pt.normal_x = 0;
//         new_pt.normal_y = 0;
//         new_pt.normal_z = 0;
//         pc_world_.push_back(new_pt);
//     }

//     ROS_WARN_STREAM("RegisterViewer size: " << pc_world_.points.size());
// }


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

    static int scan_id = 0;
    // cout  << "Scan id: " << scan_id++ << endl;

    // double N = pc_world_.points.size();
    double N = pc_world_in_voxel_.points.size();
    
    // Publish scan before ikf iteration
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
    pub_p2plane_marker_.publish(clear_markers);
    pub_p2v_marker.publish(clear_markers);

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker p2p_lines = createMarker(id_idx++, visualization_msgs::Marker::LINE_LIST, "map");
    p2p_lines.scale.x = match_line_width_;          // Line width
    p2p_lines.color.r = 1;
    p2p_lines.color.g = 0;
    p2p_lines.color.b = 0;
    p2p_lines.color.a = 1;

    for(int i=0; i<N; ++i){

        if(i % skip_cnt_ == 0)
            continue;

        Eigen::Vector3d scan_point;
        tool::pclXYZIN2EigenV3d(pc_world_in_voxel_.points[i], scan_point);

        Eigen::Vector3d p2plane_end_point = scan_point + p2plane_[i];
        geometry_msgs::Point msg_scan_point, msg_end_point;

        tool::eigenV3d2GeomsgPoint(p2plane_end_point, msg_end_point);
        tool::eigenV3d2GeomsgPoint(scan_point, msg_scan_point);


        // Bug debug.
        // double dx = abs(msg_end_point.x - msg_scan_point.x);
        // double dy = abs(msg_end_point.y - msg_scan_point.y);
        // if(dx > 0.5 || dy > 0.5) {
        //     cout << "dx: " << dx <<", dy: " << dy << endl;
        //     cout <<"Plane point: " << p2plane_end_point.transpose() <<", scan point: " << scan_point.transpose() << endl;
        //     cout << "p2plane: " << p2plane_[i].transpose() << endl;
        // }

        p2p_lines.points.push_back(msg_scan_point);
        p2p_lines.points.push_back(msg_end_point);
    }
    marker_array.markers.push_back(p2p_lines);
    pub_p2plane_marker_.publish(marker_array);



    // TODO: publish p2v markers.
    visualization_msgs::MarkerArray p2v_marker_array;
    visualization_msgs::Marker p2v_lines = createMarker(id_idx++, visualization_msgs::Marker::LINE_LIST, "map");
    visualization_msgs::Marker p2v_lines2 = createMarker(id_idx++, visualization_msgs::Marker::LINE_LIST, "map");
    p2v_lines.scale.x = match_line_width_;          // Line width
    p2v_lines.color.r = 0;
    p2v_lines.color.g = 1;
    p2v_lines.color.b = 0;
    p2v_lines.color.a = 1;
    p2v_lines2.scale.x = match_line_width_;          // Line width
    p2v_lines2.color.r = 0;
    p2v_lines2.color.g = 1;
    p2v_lines2.color.b = 0;
    p2v_lines2.color.a = 1;


    for(int i=0; i<pc_world_in_voxel_p2v_.size(); ++i){
        if (i % skip_cnt_ == 0)
            continue;

        Eigen::Vector3d scan_point;
        tool::pclXYZIN2EigenV3d(pc_world_in_voxel_p2v_.points[i], scan_point);

        Eigen::Vector3d p2v_end_point = scan_point + p2v_[i];
        geometry_msgs::Point msg_scan_point, msg_end_point;
        
        tool::eigenV3d2GeomsgPoint(p2v_end_point, msg_end_point);
        tool::eigenV3d2GeomsgPoint(scan_point, msg_scan_point);
        
        if(is_good_p2v_[i]){
            p2v_lines.points.push_back(msg_scan_point);
            p2v_lines.points.push_back(msg_end_point);
        }
        else{
            p2v_lines2.points.push_back(msg_scan_point);
            p2v_lines2.points.push_back(msg_end_point);
        }
    }
    p2v_marker_array.markers.push_back(p2v_lines);
    p2v_marker_array.markers.push_back(p2v_lines2);
    pub_p2v_marker.publish(p2v_marker_array);

    ROS_INFO_STREAM("Published p2p marker: " << p2p_lines.points.size() / 2 << ", p2v marker: " << p2v_lines.points.size() / 2);
}



