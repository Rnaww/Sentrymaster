#include "global_planner/global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

bool GlobalPathPlanner::loadBinaryMap(const std::string &path){
    cv::Mat img_read = cv::imread(path, 0);
    if (img_read.empty()) {
        error_log("map is empty");
        return false;
    }
    int x = GL_SIZE_(0);
    int y = GL_SIZE_(1);
    uint8_t *grid_ = new uint8_t[x * y];
    memset(grid_, 0, x * y * sizeof(uint8_t));
    for (int i = 0; i < x; i++) {
        for (int j = 0; j < y; j++) {
            if (img_read.at<uchar>(i, j) == 255) {
                grid_[i * GL_SIZE_(0) + j] = 1;
            }
        }
    }
    memcpy(binary_grid_map, grid_,GL_SIZE_(0)*GL_SIZE_(1)*sizeof(uint8_t));
    delete[] grid_;
    return true;
}

bool GlobalPathPlanner::loadGrayMap(const std::string &path){
    cv::Mat img_read = cv::imread(path, 0);
    if (img_read.empty()) {
        error_log("map is empty");
        return false;
    }
    int x = GL_SIZE_(0);
    int y = GL_SIZE_(1);
    uint8_t *grid_ = new uint8_t[x * y];
    memset(grid_, 0, x * y * sizeof(uint8_t));
    for (int i = 0; i < x; i++) {
        for (int j = 0; j < y; j++) {
            grid_[i * GL_SIZE_(0) + j] = img_read.at<uchar>(i, j);
        }
    }
    std::memcpy(gray_grid_map,grid_,GL_SIZE_(0)*GL_SIZE_(1)*sizeof(uint8_t));
    cv::Mat heights, low, high, clearance, free_map;
    img_read.convertTo(heights, CV_64F, 0.8/255);
    cv::erode(heights, low, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11,11)));
    cv::dilate(heights, high, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11,11)));
    cv::Mat binary(GL_SIZE_(0), GL_SIZE_(1), CV_8U, binary_grid_map);
    cv::compare(binary, 0, free_map, cv::CMP_EQ);
    cv::distanceTransform(free_map, clearance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    terrain_cost = cv::Mat(x, y, CV_64F);
    for (int i = 0; i < x; i++){
        for (int j = 0; j < y; j++){
            auto point = gridIndex2coord(Eigen::Vector3i(i,j,0));
            double limit = (point.x() > 14.2 && point.x() < 16.2 && point.y() > 2.2 && point.y() < 6.8) ? 0.65 : 4.5;
            double h = heights.at<double>(i,j);
            if (high.at<double>(i,j)-low.at<double>(i,j) > 0.015) limit = 0.45;
            // 通行图已留边 0.60m，近障惩罚再加 0.10m 正则项，避免重复强罚。
            terrain_cost.at<double>(i,j) = 1/limit + 30*(high.at<double>(i,j)-low.at<double>(i,j)) +
                std::max(0.0,h-0.1) + 0.3/(clearance.at<float>(i,j)*resolution_+0.7);
        }
    }
    delete[] grid_;
    return true;
}

bool GlobalPathPlanner::loadCrossMap(const std::string &path){
    cv::Mat img_read = cv::imread(path, 0);
    if (img_read.empty()) {
        error_log("map is empty");
        return false;
    }
    int x = GL_SIZE_(0);
    int y = GL_SIZE_(1);
    uint8_t *grid_ = new uint8_t[x * y];
    memset(grid_, 0, x * y * sizeof(uint8_t));
    for (int i = 0; i < x; i++) {
        for (int j = 0; j < y; j++) {
            if (img_read.at<uchar>(i, j) == 255) {
                grid_[i * GL_SIZE_(0) + j] = 1;
            }
        }
    }
    memcpy(cross_grid_map, grid_,GL_SIZE_(0)*GL_SIZE_(1)*sizeof(uint8_t));
    delete[] grid_;
    return true;
}

void GlobalPathPlanner::buildESDF(){
    const std::size_t total_size = static_cast<std::size_t>(GL_SIZE_(0)) *
                                   static_cast<std::size_t>(GL_SIZE_(1)) *
                                   static_cast<std::size_t>(GL_SIZE_(2));
    if (static_esdf == nullptr || esdf == nullptr || total_size == 0) {
        return;
    }

    const int size_x = GL_SIZE_(0);
    const int size_y = GL_SIZE_(1);
    const int size_z = GL_SIZE_(2);
    const double fallback_distance = 1.0e6;
    const double search_radius = std::max(dilate_radius, resolution_ * 2.0);
    const int radius_cells = std::max(0, static_cast<int>(std::ceil(search_radius / resolution_)));

    std::fill(static_esdf, static_esdf + total_size, fallback_distance);
    std::fill(esdf, esdf + total_size, fallback_distance);

    for (int x = 0; x < size_x; ++x) {
        for (int y = 0; y < size_y; ++y) {
            const std::size_t idx2d = static_cast<std::size_t>(x) * static_cast<std::size_t>(size_y) +
                                      static_cast<std::size_t>(y);

            double min_dist_sq = std::numeric_limits<double>::infinity();
            if (binary_grid_map[idx2d] == 1) {
                min_dist_sq = 0.0;
            } else {
                const int row_begin = std::max(0, x - radius_cells);
                const int row_end = std::min(size_x - 1, x + radius_cells);
                const int col_begin = std::max(0, y - radius_cells);
                const int col_end = std::min(size_y - 1, y + radius_cells);
                for (int ii = row_begin; ii <= row_end; ++ii) {
                    for (int jj = col_begin; jj <= col_end; ++jj) {
                        const std::size_t nidx2d = static_cast<std::size_t>(ii) * static_cast<std::size_t>(size_y) +
                                                   static_cast<std::size_t>(jj);
                        if (binary_grid_map[nidx2d] == 0) {
                            continue;
                        }
                        const double dx = static_cast<double>(ii - x) * resolution_;
                        const double dy = static_cast<double>(jj - y) * resolution_;
                        const double dist_sq = dx * dx + dy * dy;
                        if (dist_sq < min_dist_sq) {
                            min_dist_sq = dist_sq;
                        }
                    }
                }
            }

            if (!std::isfinite(min_dist_sq)) {
                continue;
            }

            const double distance = std::sqrt(min_dist_sq);
            for (int z = 0; z < size_z; ++z) {
                const std::size_t idx3d = static_cast<std::size_t>(x) * static_cast<std::size_t>(size_y) *
                                          static_cast<std::size_t>(size_z) +
                                          static_cast<std::size_t>(y) * static_cast<std::size_t>(size_z) +
                                          static_cast<std::size_t>(z);
                static_esdf[idx3d] = distance;
                esdf[idx3d] = distance;
            }
        }
    }
}

void GlobalPathPlanner::updateESDF(Eigen::Vector3d &start_pos, std::vector<Eigen::Vector3d> &dy_pc){
    const std::size_t total_size = static_cast<std::size_t>(GL_SIZE_(0)) *
                                   static_cast<std::size_t>(GL_SIZE_(1)) *
                                   static_cast<std::size_t>(GL_SIZE_(2));
    if (static_esdf == nullptr || esdf == nullptr || total_size == 0) {
        return;
    }

    std::memcpy(esdf, static_esdf, total_size * sizeof(double));
    (void)dy_pc;

    const double update_half_size = std::max(0.0, esdf_update_box_half_size);
    const double search_radius = std::max(0.0, esdf_search_radius);
    if (update_half_size <= 0.0) {
        return;
    }

    const int size_x = GL_SIZE_(0);
    const int size_y = GL_SIZE_(1);
    const int size_z = GL_SIZE_(2);
    const Eigen::Vector3d rel_center = start_pos - center_;
    const double update_padding = update_half_size + search_radius;
    const double compute_low_x = std::max(rel_center.x() - update_padding, gl_low_.x());
    const double compute_upper_x = std::min(rel_center.x() + update_padding, gl_upp_.x());
    const double compute_low_y = std::max(rel_center.y() - update_padding, gl_low_.y());
    const double compute_upper_y = std::min(rel_center.y() + update_padding, gl_upp_.y());

    const int row_min = std::clamp(static_cast<int>(std::ceil((compute_low_x - gl_low_.x()) / resolution_ - 0.5)), 0, size_x - 1);
    const int row_max = std::clamp(static_cast<int>(std::ceil((compute_upper_x - gl_low_.x()) / resolution_ - 0.5)) - 1, 0, size_x - 1);
    const int col_min = std::clamp(static_cast<int>(std::ceil((compute_low_y - gl_low_.y()) / resolution_ - 0.5)), 0, size_y - 1);
    const int col_max = std::clamp(static_cast<int>(std::ceil((compute_upper_y - gl_low_.y()) / resolution_ - 0.5)) - 1, 0, size_y - 1);
    if (row_min > row_max || col_min > col_max) {
        return;
    }

    const int radius_cells = std::max(0, static_cast<int>(std::ceil(search_radius / resolution_)));
    const std::size_t row_stride = static_cast<std::size_t>(size_y) * static_cast<std::size_t>(size_z);

    for (int x = row_min; x <= row_max; ++x) {
        const std::size_t x_offset = static_cast<std::size_t>(x) * row_stride;
        for (int y = col_min; y <= col_max; ++y) {
            const std::size_t idx3d = x_offset + static_cast<std::size_t>(y) * static_cast<std::size_t>(size_z);
            bool occupied = false;
            for (int z = 0; z < size_z; ++z) {
                if (obs_grid_map[idx3d + static_cast<std::size_t>(z)] == 1) {
                    occupied = true;
                    break;
                }
            }

            if (occupied) {
                for (int z = 0; z < size_z; ++z) {
                    esdf[idx3d + static_cast<std::size_t>(z)] = 0.0;
                }
                continue;
            }

            double min_dist_sq = std::numeric_limits<double>::infinity();
            const int row_begin = std::max(row_min, x - radius_cells);
            const int row_end = std::min(row_max, x + radius_cells);
            const int col_begin = std::max(col_min, y - radius_cells);
            const int col_end = std::min(col_max, y + radius_cells);
            for (int ii = row_begin; ii <= row_end; ++ii) {
                const std::size_t n_x_offset = static_cast<std::size_t>(ii) * row_stride;
                for (int jj = col_begin; jj <= col_end; ++jj) {
                    const std::size_t nidx3d = n_x_offset + static_cast<std::size_t>(jj) * static_cast<std::size_t>(size_z);
                    bool neighbor_occupied = false;
                    for (int z = 0; z < size_z; ++z) {
                        if (obs_grid_map[nidx3d + static_cast<std::size_t>(z)] == 1) {
                            neighbor_occupied = true;
                            break;
                        }
                    }
                    if (!neighbor_occupied) {
                        continue;
                    }

                    const double dx = static_cast<double>(ii - x) * resolution_;
                    const double dy = static_cast<double>(jj - y) * resolution_;
                    const double dist_sq = dx * dx + dy * dy;
                    if (dist_sq < min_dist_sq) {
                        min_dist_sq = dist_sq;
                    }
                }
            }

            if (!std::isfinite(min_dist_sq)) {
                continue;
            }

            const double distance = std::sqrt(min_dist_sq);
            for (int z = 0; z < size_z; ++z) {
                esdf[idx3d + static_cast<std::size_t>(z)] = std::min(esdf[idx3d + static_cast<std::size_t>(z)], distance);
            }
        }
    }
}

void GlobalPathPlanner::resetObs(){
    memset(obs_grid_map, 0, GL_SIZE_(0)*GL_SIZE_(1)*GL_SIZE_(2)*sizeof(uint8_t));
    memcpy(obs_grid_map, binary_grid_map,GL_SIZE_(0)*GL_SIZE_(1)* GL_SIZE_(2)*sizeof(uint8_t));
}

void GlobalPathPlanner::setObsCoor(double coord_x, double coord_y, double coord_z){
    coord_x -= center_.x();
    coord_y -= center_.y();
    coord_z -= center_.z();

    int x = static_cast<int>((coord_x - gl_low_(0)) / resolution_);
    int y = static_cast<int>((coord_y - gl_low_(1)) / resolution_);
    int z = static_cast<int>((coord_z - gl_low_(2)) / resolution_);
    obs_grid_map[x * GL_SIZE_(1) * GL_SIZE_(2) + y * GL_SIZE_(2) + z] = 1;
}

void GlobalPathPlanner::setObsPclGray(pcl::PointCloud<pcl::PointXYZI>& cloud){
    resetObs();
    for (int i = 0; i < cloud.points.size(); i++) {
        Eigen::Vector3d new_pt;
        new_pt << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
        if (coorIsInMap2d(new_pt) == false) continue;

        // 如果膨胀半径小于分辨率, 直接添加该点
        if (dilate_radius < resolution_) {
            setObsCoor(cloud.points[i].x, cloud.points[i].y, 0);
            continue;
        }

        // 高度检测, 如果在灰度图地面以上的高度区间内, 则视为障碍物
        Eigen::Vector3i index = coord2gridIndex(new_pt);
        int gray_height = gray_grid_map[index.x() * GL_SIZE_(1) * GL_SIZE_(2) + index.y() * GL_SIZE_(2) + index.z()];
        double ground_height = gray_low + (gray_height / 255.0) * (gray_high - gray_low); 
        if (new_pt.z() < ground_height + obs_low || new_pt.z() > ground_height + obs_high){
            continue;   
        }

        // 点云膨胀
        for (int j = -dilate_radius / resolution_; j < dilate_radius / resolution_; ++j) {
            for (int k = -dilate_radius / resolution_; k < dilate_radius / resolution_; ++k) {
                for (int l = 0; gl_low_[2] + resolution_ * l < gl_upp_[2]; ++l) {
                    setObsCoor(cloud.points[i].x + resolution_ * j, cloud.points[i].y + resolution_ * k, gl_low_[2] + resolution_ * l);
                }
            }
        }
    }
}

void GlobalPathPlanner::setObsCross(){
    memset(obs_grid_map, 0, GL_SIZE_(0)*GL_SIZE_(1)*GL_SIZE_(2)*sizeof(uint8_t));
    memcpy(obs_grid_map, cross_grid_map,GL_SIZE_(0)*GL_SIZE_(1)* GL_SIZE_(2)*sizeof(uint8_t));
}

void GlobalPathPlanner::setObsPcl(pcl::PointCloud<pcl::PointXYZI>& cloud){
    resetObs();
    for (int i = 0; i < cloud.points.size(); i++) {
        Eigen::Vector3d new_pt;
        new_pt << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
        if (coorIsInMap2d(new_pt) == false) continue;

        // 如果膨胀半径小于分辨率, 直接添加该点
        if (dilate_radius < resolution_) {
            setObsCoor(cloud.points[i].x, cloud.points[i].y, 0);
            continue;
        }

        // 如果在一定高度区间内，视为障碍物
        if (new_pt.z() < obs_low || new_pt.z() > obs_high){
            continue;
        }

        // 点云膨胀
        for (int j = -dilate_radius / resolution_; j < dilate_radius / resolution_; ++j) {
            for (int k = -dilate_radius / resolution_; k < dilate_radius / resolution_; ++k) {
                for (int l = 0; gl_low_[2] + resolution_ * l < gl_upp_[2]; ++l) {
                    setObsCoor(cloud.points[i].x + resolution_ * j, cloud.points[i].y + resolution_ * k, gl_low_[2] + resolution_ * l);
                }
            }
        }
    }
}

void GlobalPathPlanner::removeSelf(const Eigen::Vector3d &current_point){
    Eigen::Vector3i index = coord2gridIndex(current_point);
    for (int i = -robot_pixel_size; i < robot_pixel_size + 1; i++){
        for (int j = -robot_pixel_size; j < robot_pixel_size + 1; j++){
            int x_index = index.x() + i;
            int y_index = index.y() + j;
            obs_grid_map[x_index * GL_SIZE_(1) * GL_SIZE_(2) + y_index * GL_SIZE_(2) + index.z()] = 0;
        }
    }
}

void GlobalPathPlanner::getOccupyVector(std::vector<Eigen::Vector3d>& cloud){
    cloud.clear();
    for (int i = 0; i < GL_SIZE_.x(); i++) {
        for (int j = 0; j < GL_SIZE_.y(); j++) {
            for (int k = 0; k < GL_SIZE_.z(); k++) {
                if (isOccupied(i, j, k)) {
                    Eigen::Vector3d pt;
                    pt[0] = i * resolution_ + resolution_ / 2 + gl_low_.x() + center_.x();
                    pt[1] = j * resolution_ + resolution_ / 2 + gl_low_.y() + center_.y();
                    pt[2] = k * resolution_ + resolution_ / 2 + gl_low_.z() + center_.z();
                    cloud.push_back(pt);
                }
            }
        }
    }
}

void GlobalPathPlanner::getOccupyPcl(pcl::PointCloud<pcl::PointXYZI>& cloud){
    cloud.points.clear();
    for (int i = 0; i < GL_SIZE_.x(); i++) {
        for (int j = 0; j < GL_SIZE_.y(); j++) {
            for (int k = 0; k < GL_SIZE_.z(); k++) {
                if (isOccupied(i, j, k)) {
                    pcl::PointXYZI pt;
                    pt.x = i * resolution_ + resolution_ / 2 + gl_low_.x() + center_.x();
                    pt.y = j * resolution_ + resolution_ / 2 + gl_low_.y() + center_.y();
                    pt.z = k * resolution_ + resolution_ / 2 + gl_low_.z() + center_.z();
                    cloud.points.push_back(pt);
                }
            }
        }
    }
}

void GlobalPathPlanner::GetNeighbors(GridNodePtr current, std::vector<GridNodePtr>& neighbors){
    neighbors.clear();
    int current_z = current->index.z();

    // 相邻8点搜索
    Eigen::Vector3i index_l, index_r, index_u, index_d, index_lu, index_ru, index_ld, index_rd;
    index_l << current->index.x()-1, current->index.y(), 0;
    index_r << current->index.x()+1, current->index.y(), 0;
    index_u << current->index.x(), current->index.y()-1, 0;
    index_d << current->index.x(), current->index.y()+1, 0;
    index_lu << current->index.x()-1, current->index.y()-1, 0;
    index_ru << current->index.x()+1, current->index.y()-1, 0;
    index_ld << current->index.x()-1, current->index.y()+1, 0;
    index_rd << current->index.x()+1, current->index.y()+1, 0;
    if (isFree(index_l)){
        neighbors.push_back(GridNodeMap_[index_l.x()][index_l.y()][current_z]);
    }
    if (isFree(index_r)){
        neighbors.push_back(GridNodeMap_[index_r.x()][index_r.y()][current_z]);
    }
    if (isFree(index_u)){
        neighbors.push_back(GridNodeMap_[index_u.x()][index_u.y()][current_z]);
    }
    if (isFree(index_d)){
        neighbors.push_back(GridNodeMap_[index_d.x()][index_d.y()][current_z]);
    }

    if (isFree(index_lu) && isFree(index_l) && isFree(index_u)){
        neighbors.push_back(GridNodeMap_[index_lu.x()][index_lu.y()][current_z]);
    }
    if (isFree(index_ru) && isFree(index_r) && isFree(index_u)){
        neighbors.push_back(GridNodeMap_[index_ru.x()][index_ru.y()][current_z]);
    }
    if (isFree(index_ld) && isFree(index_l) && isFree(index_d)){
        neighbors.push_back(GridNodeMap_[index_ld.x()][index_ld.y()][current_z]);
    }
    if (isFree(index_rd) && isFree(index_r) && isFree(index_d)){
        neighbors.push_back(GridNodeMap_[index_rd.x()][index_rd.y()][current_z]);
    }
}

double GlobalPathPlanner::CalcHeu(GridNodePtr node1, GridNodePtr node2){
    // 对角距离为根号2,直线距离为1
    const double dx = std::abs(static_cast<double>(node1->index.x() - node2->index.x()));
    const double dy = std::abs(static_cast<double>(node1->index.y() - node2->index.y()));
    const double diagonal = std::min(dx, dy);
    const double straight = std::max(dx, dy) - diagonal;
    return (std::sqrt(2.0) * diagonal + straight) * resolution_ / 4.5;
}

bool GlobalPathPlanner::GetPath(std::vector<Eigen::Vector3d>& path, const Eigen::Vector3d &start_point, const Eigen::Vector3d &end_point){
    path.clear();

    // 判断规划点是否在地图内部，如果不是则规划失败
    if (!coorIsInMap2d(start_point) || !coorIsInMap2d(end_point)){
        error_log("start or end is out of map!");
        return false;
    }

    // 障碍物跳出采样半径，防止初始点在障碍物内部
    int obs_skip_sample_r = 3;
    int obs_skip_sample_edge = obs_skip_sample_r * 2 + 1;
    int obs_skip_sample_size = obs_skip_sample_edge * obs_skip_sample_edge;
    // 判断初始位置是否在障碍物内部，如果是，则向着最近的非障碍物点寻找路径
    Eigen::Vector3i start_point_index = coord2gridIndex(start_point);
    Eigen::Vector3i opt_start_point_index = start_point_index;
    if (isOccupied(start_point_index)){
        // info_log("start point in obstacle");
        // 最小距离跳出点（曼哈顿距离）
        Eigen::Vector3i min_dis_index = start_point_index;
        int min_dis = INFINITY;

        for (int i = 1; i <= obs_skip_sample_size; i++){
            int dx = i / obs_skip_sample_edge - obs_skip_sample_r;
            int dy = i % obs_skip_sample_edge - obs_skip_sample_r;
            Eigen::Vector3i tmp_index(start_point_index.x() + dx, start_point_index.y() + dy, start_point_index.z());
            int dis = abs(dx) + abs(dy);
            if (!isOccupied(tmp_index) && dis < min_dis){
                min_dis_index = tmp_index;
                min_dis = dis;
            }
        }
        opt_start_point_index = min_dis_index;
    }

    // 终点路径
    Eigen::Vector3i end_point_index = coord2gridIndex(end_point);
    Eigen::Vector3i opt_end_point_index = end_point_index;

    // 路径规划
    iter_flag++;
    GridNodePtr startPtr = GridNodeMap_[opt_start_point_index(0)][opt_start_point_index(1)][opt_start_point_index(2)];
    GridNodePtr endPtr = GridNodeMap_[opt_end_point_index(0)][opt_end_point_index(1)][opt_end_point_index(2)];
    startPtr->id = 1;
    startPtr->round = iter_flag;
    startPtr->cameFrom = NULL;
    startPtr->gScore = 0;
    startPtr->fScore = CalcHeu(startPtr, endPtr);

    // 使用优先队列, 会自动比较, 将最优的路径放入top
    std::priority_queue<OpenGridNode, std::vector<OpenGridNode>, Compare> empty_lists;
    open_lists_.swap(empty_lists);
    open_lists_.push({startPtr, startPtr->fScore});

    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;
    GridNodePtr terminatePtr_ = NULL;
    std::vector<GridNodePtr> neighbors;

    bool plan_success = false;
    while (!open_lists_.empty()) {
        currentPtr = open_lists_.top().ptr;
        open_lists_.pop();
        if (currentPtr->id == -1) continue;
        currentPtr->id = -1;  // add current node in close_list
        
        if (currentPtr->index == opt_end_point_index) {
            terminatePtr_ = currentPtr;
            plan_success = true;
            break;
        }
        
        GetNeighbors(currentPtr, neighbors);  // find neighbors and their costs
        
        for (int i = 0; i < neighbors.size(); i++) {
            neighborPtr = neighbors[i];
            // 实际代价
            auto a = currentPtr->index, b = neighborPtr->index;
            double step = (a-b).cast<double>().norm()*resolution_;
            double height_change = std::abs(static_cast<double>(gray_grid_map[a.x()*GL_SIZE_(1)+a.y()])-gray_grid_map[b.x()*GL_SIZE_(1)+b.y()])*0.8/255;
            double gh = currentPtr->gScore + step*(terrain_cost.at<double>(a.x(),a.y())+terrain_cost.at<double>(b.x(),b.y()))*0.5 + 15*height_change;
            // 实际代价 + 预估代价(启发函数计算)
            double heu = CalcHeu(neighborPtr, endPtr);
            double fh = gh + heu;
            
            // 更新轮次, 这样不需要清除上次的cost
            if (neighborPtr->round != iter_flag) {
                neighborPtr->round = iter_flag;
                neighborPtr->id = 0;
            }
            
            // 如果没有发现则加入空集
            if (neighborPtr->id == 0) {
                neighborPtr->id = 1;  // add in open list
                neighborPtr->gScore = gh;
                neighborPtr->fScore = fh;
                neighborPtr->cameFrom = currentPtr;
                open_lists_.push({neighborPtr, neighborPtr->fScore});
                
                if (!terminatePtr_){
                    terminatePtr_ = neighborPtr;
                }else if (terminatePtr_->fScore - terminatePtr_->gScore > heu){
                    terminatePtr_ = neighborPtr;
                }
            } else if (neighborPtr->id == 1) {
                if (neighborPtr->gScore > gh) {  // update open list's cost
                    neighborPtr->gScore = gh;
                    neighborPtr->fScore = fh;
                    neighborPtr->cameFrom = currentPtr;
                    open_lists_.push({neighborPtr, neighborPtr->fScore});
                }
            } else {
                continue;
            }
        }
    }

    // 如果没有完全规划 或者 不能截取部分路径则算为规划失败
    if (!plan_success && !terminatePtr_){
        error_log("no path");
        return false;
    }

    // 获得原始路径
    std::vector<Eigen::Vector3d> raw_path;
    raw_path.clear();
    std::vector<GridNodePtr> gridPath;
    GridNodePtr tmp = terminatePtr_;
    while (tmp->cameFrom != NULL) {
        gridPath.push_back(tmp);
        tmp = tmp->cameFrom;
    }
    for (auto ptr : gridPath) {
        // 路径中有一点位于障碍物内，删除前面的路径
        if(isOccupied(ptr->index)) {
            raw_path.clear();
            continue;
        }
        Eigen::Vector3d coord = gridIndex2coord(ptr->index);
        raw_path.push_back(coord);
    }
    std::reverse(raw_path.begin(), raw_path.end());

    // 同角度路径简化
    if (raw_path.size() <= 1) {
        error_log("raw path point size < 2");
        return false;
    }else{
        // 如果规划起点距离实际起点太远
        if ((raw_path[0] - gridIndex2coord(opt_start_point_index)).norm() > 0.5){
            error_log("start point offset too large");
            return false;
        }

        // 如果没有完整规划但是中间截取，舍去最末尾小于阈值长度的路径点
        if (!plan_success && terminatePtr_){
            double path_length = 0.0;
            Eigen::Vector3d last_path_point = raw_path.back();
            int delet_index = raw_path.size() - 1;
            for (int i = delet_index; i >= 0; i--){
                Eigen::Vector3d path_point = raw_path[i];
                path_length += (path_point - last_path_point).norm();
                last_path_point = path_point;
                if (path_length < interupt_path_min_length){
                    delet_index = i;
                }else{
                    break;
                }
            }
            raw_path.erase(raw_path.begin() + delet_index, raw_path.end());
        }

        if (raw_path.size() <= 1){
            error_log("interupt path too short");
            return false;
        }

        path.push_back(raw_path[0]);
        if (raw_path.size() > 2){
            Eigen::Vector3d vec_last = raw_path[1] - raw_path[0];
            for (int i = 2; i < raw_path.size(); i++) {
                Eigen::Vector3d vec = raw_path[i] - raw_path[i - 1];
                if (vec.dot(vec_last) == vec.norm() * vec_last.norm()) {  
                    // The included angle of two vectors is 0
                    continue;
                }
                path.push_back(raw_path[i - 1]);
                vec_last = vec;
            }
            path.push_back(raw_path[raw_path.size() - 1]);
        }else{
            path.push_back(raw_path[1]);
        }
    }

    return true;
}

Eigen::Vector3d GlobalPathPlanner::getBackGoal(const Eigen::Vector3d &current_pos, const Eigen::Vector3d &obs_pos){
    if (!coorIsInMap2d(current_pos) || back_goal_search_grid_num <= 0) {
        return current_pos;
    }

    Eigen::Vector3d back_direction = current_pos - obs_pos;
    back_direction.z() = 0.0;
    if (back_direction.norm() < 1e-6) {
        return current_pos;
    }
    back_direction.normalize();

    const Eigen::Vector3i current_index = coord2gridIndex(current_pos);
    const int size_x = GL_SIZE_.x();
    const int size_y = GL_SIZE_.y();
    const double min_distance_sq = static_cast<double>(back_goal_search_grid_num * back_goal_search_grid_num);
    const auto visited_offset = [size_y](const Eigen::Vector3i &index) {
        return index.x() * size_y + index.y();
    };

    struct SearchGrid {
        double distance_score;
        double direction_score;
        Eigen::Vector3i index;

        bool operator<(const SearchGrid &other) const {
            if (distance_score == other.distance_score) {
                return direction_score < other.direction_score;
            }
            return distance_score < other.distance_score;
        }
    };

    std::vector<uint8_t> visited(size_x * size_y, 0);
    std::priority_queue<SearchGrid> open_list;
    visited[visited_offset(current_index)] = 1;
    open_list.push({0.0, 0.0, current_index});

    const int dxs[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dys[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    while (!open_list.empty()) {
        const SearchGrid current_grid = open_list.top();
        open_list.pop();

        const int dx_from_start = current_grid.index.x() - current_index.x();
        const int dy_from_start = current_grid.index.y() - current_index.y();
        const Eigen::Vector3d offset(
            static_cast<double>(dx_from_start) * resolution_,
            static_cast<double>(dy_from_start) * resolution_,
            0.0);
        const double grid_distance_sq =
            static_cast<double>(dx_from_start * dx_from_start + dy_from_start * dy_from_start);
        if (grid_distance_sq >= min_distance_sq && offset.dot(back_direction) > 0.0) {
            Eigen::Vector3d back_goal = gridIndex2coord(current_grid.index);
            back_goal.z() = 0.0;
            return back_goal;
        }

        for (int i = 0; i < 8; ++i) {
            Eigen::Vector3i next_index(
                current_grid.index.x() + dxs[i],
                current_grid.index.y() + dys[i],
                current_grid.index.z());

            if (!isFree(next_index)) {
                continue;
            }
            if (visited[visited_offset(next_index)]) {
                continue;
            }
            if (dxs[i] != 0 && dys[i] != 0) {
                Eigen::Vector3i side_index_x(next_index.x(), current_grid.index.y(), next_index.z());
                Eigen::Vector3i side_index_y(current_grid.index.x(), next_index.y(), next_index.z());
                if (!isFree(side_index_x) || !isFree(side_index_y)) {
                    continue;
                }
            }

            const int next_dx_from_start = next_index.x() - current_index.x();
            const int next_dy_from_start = next_index.y() - current_index.y();
            const Eigen::Vector3d next_offset(
                static_cast<double>(next_dx_from_start) * resolution_,
                static_cast<double>(next_dy_from_start) * resolution_,
                0.0);
            if (next_offset.dot(back_direction) <= -resolution_) {
                continue;
            }

            const double next_distance_score = static_cast<double>(
                next_dx_from_start * next_dx_from_start + next_dy_from_start * next_dy_from_start);
            const double next_direction_score = next_offset.dot(back_direction);
            visited[visited_offset(next_index)] = 1;
            open_list.push({next_distance_score, next_direction_score, next_index});
        }
    }

    return current_pos;
}

double GlobalPathPlanner::traversalCost(const Eigen::Vector3d &point){
    auto index = coord2gridIndex(point);
    return terrain_cost.at<double>(index.x(), index.y());
}

// 与 A* 高度变化项使用同一张灰度高度图。
double GlobalPathPlanner::surfaceHeight(const Eigen::Vector3d &point){
    auto index = coord2gridIndex(point);
    return gray_grid_map[index.x()*GL_SIZE_(1)+index.y()]*0.8/255;
}
