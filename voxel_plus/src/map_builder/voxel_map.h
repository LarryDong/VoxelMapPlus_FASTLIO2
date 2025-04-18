#pragma once
#include <list>
#include <vector>
#include <memory>
#include <cstdint>
#include <Eigen/Eigen>
#include <unordered_map>
#include <chrono>
#include <iostream>

#define HASH_P 116101
#define MAX_N 10000000000

namespace lio
{

    class VoxelKey
    {
    public:
        int64_t x, y, z;
        VoxelKey(int64_t _x = 0, int64_t _y = 0, int64_t _z = 0) : x(_x), y(_y), z(_z) {}

        bool operator==(const VoxelKey &other) const
        {
            return (x == other.x && y == other.y && z == other.z);
        }
        struct Hasher
        {
            int64_t operator()(const VoxelKey &k) const
            {
                return ((((k.z) * HASH_P) % MAX_N + (k.y)) * HASH_P) % MAX_N + (k.x);
            }
        };

        // Dongyan Added
        friend std::ostream &operator<<(std::ostream &os, const VoxelKey &key){
            os << "[ " << key.x << ", " << key.y << ", " << key.z << " ]";
            return os;
        }
    };

    struct PointWithCov
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d point;
        Eigen::Matrix3d cov;
    };

    struct Plane
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        Eigen::Matrix3d ppt = Eigen::Matrix3d::Zero();
        Eigen::Vector3d norm = Eigen::Vector3d::Zero();
        Eigen::Matrix<double, 6, 6> cov;
        int n = 0;
    };

    struct ResidualData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d point_lidar;
        Eigen::Vector3d point_world;
        Eigen::Vector3d plane_mean;     // center of the plane, for any p2plane vector.
        Eigen::Vector3d plane_norm;
        Eigen::Matrix<double, 6, 6> plane_cov;
        Eigen::Matrix3d cov_lidar;
        Eigen::Matrix3d cov_world;
        bool is_valid = false;
        double residual = 0.0;

        // p2v new state
        Eigen::Vector3d p2v = Eigen::Vector3d(0,0,0);       // then length of p2v is same to `residual`, the direction of p2v is `plane_norm`
        double weight = 0.0;        // weight predicted by p2v-net -->  {plane_cov + point_cov(R=0.0001)}
    };

    class VoxelMap;

    class VoxelGrid
    {
    public:
        VoxelGrid(int _max_point_thresh, int _update_point_thresh, double _plane_thresh, VoxelKey _position, VoxelMap *_map);

        void updatePlane();

        void addToPlane(const PointWithCov &pv);

        void addPoint(const PointWithCov &pv);

        void pushPoint(const PointWithCov &pv);

        void merge();

    public:
        static uint64_t count;              // This count is static, only "group_id" is valid for index.
        uint64_t group_id;

        int max_point_thresh;
        int update_point_thresh;
        double plane_thresh;
        bool is_init;
        bool is_plane;
        bool update_enable;
        int newly_add_point;
        bool merged;
        std::vector<PointWithCov> temp_points;
        VoxelKey position;
        VoxelMap *map;                      // TODO: Why VoxelGrid has a VoxelMap??? Seems not used?
        std::shared_ptr<Plane> plane;
        Eigen::Vector3d center;
        std::list<VoxelKey>::iterator cache_it;
        static double merge_thresh_for_angle;
        static double merge_thresh_for_distance;
    };

    // unordered_map<key, value, hash>，其中hash将key映射为hash。对于自定义类型的key，需要指定hash方式。
    typedef std::unordered_map<VoxelKey, std::shared_ptr<VoxelGrid>, VoxelKey::Hasher> Featmap;

    class VoxelMap
    {
    public:
        VoxelMap(int _max_point_thresh, int _update_point_thresh, double _plane_thresh, double _voxel_size, int capacity = 2000000);

        VoxelKey index(const Eigen::Vector3d &point);

        void build(std::vector<PointWithCov> &pvs);

        void update(std::vector<PointWithCov> &pvs);

        bool buildResidual(ResidualData &data, std::shared_ptr<VoxelGrid> voxel_grid);

        // DONGYAN ADDED
        void printInfo(bool verbose=false);

    public:
        int max_point_thresh;
        int update_point_thresh;
        double plane_thresh;
        double voxel_size;
        Featmap featmap;
        std::list<VoxelKey> cache;
        int capacity;
    };

} // namespace lio
