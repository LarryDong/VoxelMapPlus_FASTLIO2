# 个人修改

## 安装配置过程
1. Sophus编译时遇到cmake版本过低，需要：官方下载最新版本cmake，然后编译install，然后修改PATH环境指到最新的cmake: PATH: "export PATH=/usr/local/bin:$PATH  
2. `std_matcher` 需要GTSAM，安装GTSAM时版本遇到些问题。因为用不到回环，所以CATKIN_IGNORE掉了这个launch
3. livox_ros_driver2，安装后会出现cmake配置文件找不到问题，所以索性直接装到对应的workspace下面；但是执行脚本文件时，不知道为啥会自动编译整个workspace，所以保证其他没有问题。
4. VoxelMapPlus_FASTLIO2代码CMakeLists给的c++是14，和我安装的sophus版本需要c++17，所以进行了修改。否则会报错：error: ‘std::optional’ has not been declared
5. Compile this code without having anaconda environment!!!


# FastLIO2 With VoxelMapPlus And STD
![image](https://github.com/liangheming/VoxelMapPlus_FASTLIO2/blob/main/resources/demo.png)

## 主要工作
1. 基于[FASTLIO2](https://github.com/hku-mars/FAST_LIO)和[VoxelMap](https://github.com/hku-mars/VoxelMap)
2. 参考[VoxelMapPlus](https://github.com/uestc-icsp/VoxelMapPlus_Public)
3. 目前暂时支持MID_360的传感器
4. 平面主要还是用均值和法向量进行参数化，而不是使用(ax+by+z+d=0,x+by+cz+d=0,ax+y+cz+d=0)建模
5. voxel与voxel之间主要基于法向量夹角(余弦相似度)，以及原点到平面距离(欧式距离)这两个参数进行融合
6. 地图的内存管理采用LRU(last recently used)的规则，优先释放最长时间没有使用的voxel
7. 重写了[STD](https://github.com/hku-mars/STD)的代码，去除最大特征个数的限制，简化了部分代码，去除对ceres的依赖。
8. 基于[STD](https://github.com/hku-mars/STD) 完成回环检测和相应的位姿图优化功能；
## 环境说明
```text
系统版本: ubuntu20.04
机器人操作系统: ros1-noetic
```

## 编译依赖
1. livox_ros_driver2
2. pcl
3. sophus
4. eigen

### 1.安装 LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 2.安装 livox_ros_driver2
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
./build.sh ROS1
```

### 3. 安装Sophus
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```
**新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错**

## DEMO 数据
```text
链接: https://pan.baidu.com/s/1eEnc5fWYCQJxVCaEw7buiA?pwd=cwd4 提取码: cwd4 
--来自百度网盘超级会员v7的分享
```

## 启动脚本
建图线程
```shell
roslaunch voxel_plus mapping.launch
rosbag play your_bag.bag
```
建图加回环
```
roslaunch std_matcher pgo.launch
rosbag play your_bag.bag
```
## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [VoxelMap](https://github.com/hku-mars/VoxelMap)
3. [VoxelMapPlus](https://github.com/uestc-icsp/VoxelMapPlus_Public)
4. [STD](https://github.com/hku-mars/STD)

## 对于STD检测回环的一些想法
1. 描述子提取不够稳定(网格的划分，网格遍历的顺序，甚至是特征值的调用方式，都会导致类似场景的描述子提取的不一致性);
2. 无论是STD还是SC，大多利于360的平放的lidar，对斜放的，小FOV的lidar都不是很友好;
3. 整体上召回率还是较低(对于回环优化来说是一件好事);
4. 还是存在误检的情况,误检测的问题还是需要多方校验的，谨慎进行回环优化[LTAOM](https://github.com/hku-mars/LTAOM) 有相关优化，但是个人感觉代价较大;
