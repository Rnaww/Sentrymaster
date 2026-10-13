#pragma once
#include <cstddef>
#include "common.h"
#include "grid_node.hpp"

// 全局路径规划器，同时维护障碍物地图
class GlobalPathPlanner{
private:
    // 网格地图迭代标记，节省清空内存的时间
    int iter_flag;
    // 二值数组地图
    uint8_t* binary_grid_map;
    // 灰度数组地图
    uint8_t* gray_grid_map;
    // 跨越地形二值数组地图
    uint8_t* cross_grid_map;
    // 最终障碍物数组地图
    uint8_t* obs_grid_map;
    // 三维网格地图
    GridNodePtr*** GridNodeMap_;
    // 静态欧几里德距离场
    double* static_esdf;
    // 地形行驶、车体高度变化和留边代价。
    cv::Mat terrain_cost;
    // A*搜索开集
    std::priority_queue<OpenGridNode, std::vector<OpenGridNode>, Compare> open_lists_;

    // 障碍物判定参数
    double obs_low, obs_high, gray_low, gray_high;
    // 膨胀半径
    double dilate_radius;
    // ESDF更新包围框半边长
    double esdf_update_box_half_size;
    // ESDF搜索半径
    double esdf_search_radius;
    // 最小截取路径长度
    double interupt_path_min_length;
    // 车体边长
    int robot_pixel_size;
    // 退打目标点最小距离网格数量
    int back_goal_search_grid_num;

public:
    // 地图分辨率
    double resolution_;
    // 动态欧几里德距离场
    double* esdf;
    // 地图大小
    Eigen::Vector3i GL_SIZE_;
    // 地图边界点
    Eigen::Vector3d gl_low_, gl_upp_;
    // 地图中心
    Eigen::Vector3d center_;

    GlobalPathPlanner(rclcpp::Node *node){
        // 初始化地图参数
        center_ = Eigen::Vector3d::Zero();
        resolution_ = node->declare_parameter("globalPlanner.resolution", 0.0);
        std::vector<double> mapBound = node->get_parameter("MapBound").as_double_array();
        gl_low_ << mapBound[0], mapBound[2], mapBound[4];
        gl_upp_ << mapBound[1], mapBound[3], mapBound[5];
        GL_SIZE_.x() = (gl_upp_.x() - gl_low_.x()) / resolution_;
        GL_SIZE_.y() = (gl_upp_.y() - gl_low_.y()) / resolution_;
        GL_SIZE_.z() = (gl_upp_.z() - gl_low_.z()) / resolution_;
        info_log("GL_SIZE_: " , GL_SIZE_.transpose());
        iter_flag = 0;

        // 初始化数组地图
        binary_grid_map = new uint8_t[GL_SIZE_(0) * GL_SIZE_(1) * GL_SIZE_(2)];
        gray_grid_map = new uint8_t[GL_SIZE_(0) * GL_SIZE_(1) * GL_SIZE_(2)];
        obs_grid_map = new uint8_t[GL_SIZE_(0) * GL_SIZE_(1) * GL_SIZE_(2)];
        cross_grid_map = new uint8_t[GL_SIZE_(0) * GL_SIZE_(1) * GL_SIZE_(2)];
        memset(binary_grid_map, 0, GL_SIZE_(0) * GL_SIZE_(1) * GL_SIZE_(2) * sizeof(uint8_t));
        memset(gray_grid_map, 0, GL_SIZE_(0) * GL_SIZE_(1) * GL_SIZE_(2) * sizeof(uint8_t));
        memset(cross_grid_map, 0, GL_SIZE_(0) * GL_SIZE_(1) * GL_SIZE_(2) * sizeof(uint8_t));
        memset(obs_grid_map, 0, GL_SIZE_(0) * GL_SIZE_(1) * GL_SIZE_(2) * sizeof(uint8_t));
        std::string binary_map_path = node->declare_parameter("globalPlanner.PreMapPath", "");
        loadBinaryMap(binary_map_path);
        std::string gray_map_path = node->declare_parameter("globalPlanner.PreDeepPath", "");
        loadGrayMap(gray_map_path);
        std::string cross_map_path = node->declare_parameter("globalPlanner.PreCrossPath", "");
        loadCrossMap(cross_map_path);

        // 初始化三维网格地图
        GridNodeMap_ = new GridNodePtr**[GL_SIZE_(0)];
        for (int i = 0; i < GL_SIZE_(0); i++) {
            GridNodeMap_[i] = new GridNodePtr*[GL_SIZE_(1)];
            for (int j = 0; j < GL_SIZE_(1); j++) {
                GridNodeMap_[i][j] = new GridNodePtr[GL_SIZE_(2)];
                for (int k = 0; k < GL_SIZE_(2); k++) {
                    Eigen::Vector3i tmpIdx(i, j, k);
                    GridNodeMap_[i][j][k] = new GridNode(tmpIdx);
                }
            }
        }
        resetObs();

        // 参数声明
        robot_pixel_size = node->declare_parameter("globalPlanner.robot_pixel_size", 0);
        obs_low = node->declare_parameter("globalPlanner.obs_low", 0.0);
        obs_high = node->declare_parameter("globalPlanner.obs_high", 0.0);
        gray_low = node->declare_parameter("globalPlanner.gray_low", 0.0);
        gray_high = node->declare_parameter("globalPlanner.gray_high", 0.0);
        dilate_radius = node->declare_parameter("globalPlanner.dilate_radius", 0.0);
        esdf_update_box_half_size = node->declare_parameter("globalPlanner.esdf_update_box_half_size", 0.0);
        esdf_search_radius = node->declare_parameter("globalPlanner.esdf_search_radius", 0.0);
        interupt_path_min_length = node->declare_parameter("globalPlanner.interupt_path_min_length", 0.0);
        back_goal_search_grid_num = node->declare_parameter<int>("globalPlanner.back_goal_search_grid_num", 20);

        // 初始化esdf
        const std::size_t esdf_size = static_cast<std::size_t>(GL_SIZE_(0)) * static_cast<std::size_t>(GL_SIZE_(1)) *
                                      static_cast<std::size_t>(GL_SIZE_(2));
        static_esdf = new double[esdf_size];
        esdf = new double[esdf_size];
        memset(static_esdf, 0, esdf_size * sizeof(double));
        memset(esdf, 0, esdf_size * sizeof(double));
        buildESDF();
    }

    ~GlobalPathPlanner(){
        for (int x = 0; x < GL_SIZE_(0); x++){
            for (int y = 0; y < GL_SIZE_(1); y++){
                for (int z = 0; z < GL_SIZE_(2); z++) delete GridNodeMap_[x][y][z];
                delete[] GridNodeMap_[x][y];
            }
            delete[] GridNodeMap_[x];
        }
        delete[] GridNodeMap_;
        delete[] binary_grid_map;
        delete[] gray_grid_map;
        delete[] cross_grid_map;
        delete[] obs_grid_map;
        delete[] static_esdf;
        delete[] esdf;
    }

    // 二值图片转换成网格地图障碍物
    bool loadBinaryMap(const std::string &path);
    // 灰度图片转换成网格地图障碍物
    bool loadGrayMap(const std::string &path);
    // 二值图片转换成跨越地形障碍物图
    bool loadCrossMap(const std::string &path);

    // 构建ESDF
    void buildESDF();
    // 更新ESDF
    void updateESDF(Eigen::Vector3d &start_pos, std::vector<Eigen::Vector3d> &dy_pc);

    // 重置障碍物数组地图
    void resetObs();
    // 使用三维坐标设置障碍物
    void setObsCoor(double coord_x, double coord_y, double coord_z);
    // 通过map系下的pcl点云设置障碍物, 会根据灰度图像进行过滤
    void setObsPclGray(pcl::PointCloud<pcl::PointXYZI>& cloud);
    // 通过map系下的pcl点云设置障碍物, 不会根据灰度图像进行过滤
    void setObsPcl(pcl::PointCloud<pcl::PointXYZI>& cloud);
    // 跨越地形模式下重置点云障碍物
    void setObsCross();
    // 清除车身被识别的障碍物点云
    void removeSelf(const Eigen::Vector3d &current_point);


    // 得到pcl类型的障碍物点
    void getOccupyPcl(pcl::PointCloud<pcl::PointXYZI>& cloud);
    // 得到vector类型的障碍物点
    void getOccupyVector(std::vector<Eigen::Vector3d>& cloud);

    double traversalCost(const Eigen::Vector3d &point);
    double surfaceHeight(const Eigen::Vector3d &point);
    // 计算路径代价
    double CalcHeu(GridNodePtr node1, GridNodePtr node2);
    // 计算临域
    void GetNeighbors(GridNodePtr cur, std::vector<GridNodePtr>& neighbors);

    // 获取路径
    bool GetPath(std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d &start_point, const Eigen::Vector3d &end_point);

    // 获取退打目标点
    Eigen::Vector3d getBackGoal(const Eigen::Vector3d &current_pos, const Eigen::Vector3d &obs_pos);

    // 检测点是否在地图内，包括z轴
    inline bool coorIsInMap(const Eigen::Vector3d& pt) {
        Eigen::Vector3d error = pt - center_;
        if (error.x() >= gl_low_.x() && error.x() < gl_upp_.x() && error.y() >= gl_low_.y() && error.y() < gl_upp_.y() && error.z() >= gl_low_.z() &&
            error.z() < gl_upp_.z()) {
            return true;
        }
        return false;
    }

    // 检测点是否在地图的二维投影内
    inline bool coorIsInMap2d(const Eigen::Vector3d& pt) {
        Eigen::Vector3d error = pt - center_;
        if (error.x() >= gl_low_.x() && error.x() < gl_upp_.x() && error.y() >= gl_low_.y() && error.y() < gl_upp_.y()) {
            return true;
        }
        return false;
    }

    // 地图数组下标转坐标
    inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
        Eigen::Vector3d pt;
        pt(0) = index(0) * resolution_ + resolution_ / 2 + gl_low_(0);
        pt(1) = index(1) * resolution_ + resolution_ / 2 + gl_low_(1);
        pt(2) = index(2) * resolution_ + resolution_ / 2 + gl_low_(2);
        pt += center_;
        return pt;
    }

    // 地图坐标转数组下标
    inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
        Eigen::Vector3d c_pt = pt - center_;
        Eigen::Vector3i index;
        index(0) = std::min(std::max(int((c_pt(0) - gl_low_(0)) / resolution_), 0), GL_SIZE_(0) - 1);
        index(1) = std::min(std::max(int((c_pt(1) - gl_low_(1)) / resolution_), 0), GL_SIZE_(1) - 1);
        index(2) = std::min(std::max(int((c_pt(2) - gl_low_(2)) / resolution_), 0), GL_SIZE_(2) - 1);
        return index;
    }

    // 通过三维地图数组下标判断是否为障碍物
    inline bool isOccupied(int idx_x, int idx_y, int idx_z) {
        return (idx_x >= 0 && idx_x < GL_SIZE_(0) && idx_y >= 0 && idx_y < GL_SIZE_(1) && idx_z >= 0 && idx_z < GL_SIZE_(2) &&
                (obs_grid_map[idx_x * GL_SIZE_(1) * GL_SIZE_(2) + idx_y * GL_SIZE_(2) + idx_z] == 1));
    }
    // 通过三维地图数组下标判断是否为障碍物
    inline bool isOccupied(const Eigen::Vector3i index) { return isOccupied(index(0), index(1), index(2)); }
    inline bool isFree(int idx_x, int idx_y, int idx_z) {
        return (idx_x >= 0 && idx_x < GL_SIZE_(0) && idx_y >= 0 && idx_y < GL_SIZE_(1) && idx_z >= 0 && idx_z < GL_SIZE_(2) &&
                (obs_grid_map[idx_x * GL_SIZE_(1) * GL_SIZE_(2) + idx_y * GL_SIZE_(2) + idx_z] < 1));
    }
    inline bool isFree(const Eigen::Vector3i index) { return isFree(index(0), index(1), index(2)); }
};
