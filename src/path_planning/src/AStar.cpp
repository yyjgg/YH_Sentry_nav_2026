#include "AStar.h"
#include <queue>
#include <algorithm>
#include <iostream>
#include <cmath> 
#include <algorithm>

AStar::AStar(const GridMap& map) 
    : grid_map_(map), nodes_explored_(0), max_open_list_size_(0) {
    // 初始化涂色数组
    close_list = new bool[grid_map_.getSizeX()*grid_map_.getSizeY()];
    memset(close_list, false, grid_map_.getSizeX()*grid_map_.getSizeY() * sizeof(uint8_t));
}

void AStar::initializeSearch() {
    // 重置涂色数组
    memset(close_list, false, grid_map_.getSizeX()*grid_map_.getSizeY() * sizeof(uint8_t));
    nodes_explored_ = 0;
    max_open_list_size_ = 0;
}

bool AStar::isValidNode(const MapNode& node) const {
    int x = static_cast<int>(node.x);
    int y = static_cast<int>(node.y);
    return grid_map_.isWithinBounds(x, y) && 
           !grid_map_.isObstacle(x, y) &&
           !grid_map_.isInflatedObstacle(x, y);
}

std::vector<MapNode> AStar::reconstructPath(const MapNode& endNode) const {
    std::vector<MapNode> path;
    const MapNode* current = &endNode;
    
    while (current != nullptr) {
        path.push_back(*current);
        current = current->parent.get(); // 假设parent是std::shared_ptr
    }
    // std::shared_ptr<MapNode> current = std::make_shared<MapNode>(endNode);
    
    // while (current != nullptr) {
    //     path.push_back(*current);
    //     current = current->parent;
    // }

    // 反转得到起点到终点
    std::reverse(path.begin(), path.end());
    return path;
}
// A*路径搜索
std::vector<MapNode> AStar::Search(const MapNode& start, const MapNode& goal) {
    if (!grid_map_.isValid()) {
        std::cout << "Error: Invalid map configuration!" << std::endl;
        return {};
    }
    
    initializeSearch();
    
    // 使用自定义比较器优先队列作为OpenList
    std::priority_queue<MapNode, std::vector<MapNode>, NodeCompare> open_list;
    
    // 设置起点代价并加入OpenList（使用整数代价）
    MapNode startNode = start;
    startNode.g_cost = 0;
    startNode.h_cost = startNode.CalcDeltaHValue(goal);
    open_list.push(startNode);
    
    while (!open_list.empty()) {
        // 更新最大OpenList大小统计
        max_open_list_size_ = std::max(max_open_list_size_, static_cast<int>(open_list.size()));
        
        // 取出代价最小的节点
        MapNode current = open_list.top();// 获取优先队列顶部元素（f_cost最小的节点）
        open_list.pop(); // 从队列中移除该节点
        
        // 关键检查：跳过已在CloseList中的节点
        if (close_list[static_cast<int>(current.x)*grid_map_.getSizeY()+static_cast<int>(current.y)]) {
            continue;
        }
        
        // 标记为已探索
        close_list[static_cast<int>(current.x)*grid_map_.getSizeY()+static_cast<int>(current.y)] = true;
        nodes_explored_++;
        
        // 检查是否到达目标
        if (std::sqrt(pow(current.x-goal.x,2)+pow(current.y-goal.y,2))<2) {
            std::cout << "Path found! Nodes explored: " << nodes_explored_ 
                      << ", Max open list size: " << max_open_list_size_ 
                      << ", Total cost: " << current.g_cost << std::endl;
            return reconstructPath(current);
        }
        
        // 生成并处理子节点
        auto children = current.GenerateChildren(grid_map_);
        for (MapNode child : children) {
            if (!isValidNode(child) || close_list[static_cast<int>(child.x)*grid_map_.getSizeY()+static_cast<int>(child.y)]) {
                continue;
            }
            // 计算整数代价（欧氏距离）
            child.g_cost = current.g_cost + child.CalcDeltaGValue(current);
            child.h_cost = child.CalcDeltaHValue(goal);
            open_list.push(child);
        }
    }
    
    std::cout << "No path found! Nodes explored: " << nodes_explored_ << std::endl;
    return {}; // 返回空路径
}

// 计算两点间距离
double AStar::calcDistance(const MapNode& a, const MapNode& b) const {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

// Bresenham直线算法检查视线是否畅通
bool AStar::isLineOfSightClear(const MapNode& start, const MapNode& end) const {
    int x0 = static_cast<int>(start.x);
    int y0 = static_cast<int>(start.y);
    int x1 = static_cast<int>(end.x);
    int y1 = static_cast<int>(end.y);
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    while (true) {
        // 检查当前栅格是否为障碍物或膨胀区域
        if (grid_map_.isObstacle(x, y)|| grid_map_.isInflatedObstacle(x, y)) {
            return false;
        }
        
        if (x == x1 && y == y1) {
            break;
        }
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    
    return true;
}

// 关键点提取算法
std::vector<MapNode> AStar::extractKeyPoints(const std::vector<MapNode>& path, double maxConnectionDist) const {
    if (path.size() <= 2) {
        return path; // 路径太短，无需简化
    }
    
    std::vector<MapNode> keyPoints;
    keyPoints.push_back(path[0]); // 总是包含起点
    
    size_t lastKeyIndex = 0;
    
    // 从第三个点开始检查（索引2）
    for (size_t i = 2; i < path.size(); i++) {
        double distance = calcDistance(path[lastKeyIndex], path[i]);
        
        // 如果距离太远或视线不通，将前一个点设为关键点
        if (distance > maxConnectionDist || !isLineOfSightClear(path[lastKeyIndex], path[i])) {
            keyPoints.push_back(path[i-1]);
            lastKeyIndex = i-1;
        }
    }
    
    // 总是包含终点
    keyPoints.push_back(path.back());
    return keyPoints;
}

