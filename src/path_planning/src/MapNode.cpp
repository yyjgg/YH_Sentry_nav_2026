#include "MapNode.h"
#include "GridMap.h"
#include <cmath>
#include <algorithm>

// 移动方向定义：上、右、下、左
const double MapNode::DIRECTIONS[8][2] = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1},
                                        {-1, 1}, {-1, 0}, {-1, -1}};

MapNode::MapNode(double x, double y, std::shared_ptr<MapNode> parent, double g, double h)
    : x(x), y(y), parent(parent), g_cost(g), h_cost(h) {}

double MapNode::f_cost() const {
    return g_cost + h_cost;
}

double MapNode::CalcDeltaGValue(const MapNode& parentMapNode) const {
    return std::sqrt(pow(x-parentMapNode.x,2) + pow(y-parentMapNode.y,2));
}

double MapNode::CalcDeltaHValue(const MapNode& goalMapNode) const {
    // 使用欧氏距离作为启发式函数
    // return std::sqrt(pow(x-goalMapNode.x,2) + pow(y-goalMapNode.y,2));
    // 使用对角线距离作为启发式函数
    return std::abs(x-goalMapNode.x) + std::abs(y-goalMapNode.y) - (1.414-2)*std::min(std::abs(x-goalMapNode.x),std::abs(y-goalMapNode.y));
}

std::vector<MapNode> MapNode::GenerateChildren(const GridMap& map) const {
    std::vector<MapNode> children;
    children.reserve(8); // 预分配空间提高效率
    
    for (const auto& dir : DIRECTIONS) {
        double newX = x + dir[0];
        double newY = y + dir[1];
        
        // 检查是否在地图范围内
        if (newX >= 0 && newX < map.getSizeX() && 
            newY >= 0 && newY < map.getSizeY()) {
            children.emplace_back(newX, newY, std::make_shared<MapNode>(*this));
        }
    }
    return children;
}

bool MapNode::operator<(const MapNode& other) const {
    if (f_cost() == other.f_cost()) {
        return h_cost > other.h_cost; // f相同时，h小的优先
    }
    return f_cost() > other.f_cost(); // 优先队列需要大顶堆行为
}

bool MapNode::operator==(const MapNode& other) const {
    return x == other.x && y == other.y;
}