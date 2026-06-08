#ifndef NODE_H
#define NODE_H

#include <vector>
#include <memory>

// 前向声明
class GridMap;

/**
 * @brief 表示网格地图中的单个节点
 * 
 * 使用欧氏距离和整数代价计算，优化A*算法性能
 */
class MapNode {
public:
    double x, y;
    std::shared_ptr<MapNode> parent;
    double g_cost; 
    double h_cost;  

    MapNode(double x = 0, double y = 0, 
         std::shared_ptr<MapNode> parent = nullptr, 
         double g = 0, double h = 0);

    // 计算总代价 f(n) = g(n) + h(n)
    double f_cost() const;

    // 使用欧氏距离计算从父节点移动到当前节点的G值增量
    double CalcDeltaGValue(const MapNode& parentMapNode) const;
    
    // 使用欧氏距离计算当前节点到终点的H值
    double CalcDeltaHValue(const MapNode& goalMapNode) const;
    
    // 生成子节点
    std::vector<MapNode> GenerateChildren(const GridMap& map) const;
    
    // 重载比较运算符
    bool operator<(const MapNode& other) const;
    bool operator==(const MapNode& other) const;
    
    // 获取坐标
    std::pair<double, double> getCoordinates() const { return {x, y}; }

private:
    static const double DIRECTIONS[8][2]; // 移动方向定义
};

#endif // NODE_H