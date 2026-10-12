#pragma once
#include "common.h"

// 地图节点
struct GridNode;
typedef GridNode* GridNodePtr;
struct GridNode {
    GridNode() { GridNode(Eigen::Vector3i(0, 0, 0)); }
    GridNode(Eigen::Vector3i _index) {
        id = 0;
        round = 0;
        index = _index;
        dir = Eigen::Vector3i::Zero();
        gScore = 0;
        fScore = 0;
        cameFrom = NULL;
    }
    ~GridNode() {}

    int8_t id;  // 1--> open set, -1 --> closed set
    int round;
    Eigen::Vector3i index;
    Eigen::Vector3i dir;
    double gScore;
    double fScore;
    GridNodePtr cameFrom;
};

// 节点评分比较函数
struct Compare {
    bool operator()(const GridNodePtr a, const GridNodePtr b) { return a->fScore > b->fScore; }
};