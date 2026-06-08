#ifndef ASTAR_H
#define ASTAR_H

#include "MapNode.h"
#include "GridMap.h"
#include <queue>
#include <vector>
#include <functional>
using namespace std;
/**
 * @brief A*路径搜索算法实现类
 * 
 * 使用曼哈顿距离和整数代价计算，优化性能
 */
class AStar {
public:
    explicit AStar(const GridMap& map);
    
    /**
     * @brief 执行A*搜索算法
     * @param start 起始节点
     * @param goal 目标节点
     * @return 找到的路径节点序列，空序列表示无路径
     */
    std::vector<MapNode> Search(const MapNode& start, const MapNode& goal);
    
    // 获取搜索统计信息
    int getNodesExplored() const { return nodes_explored_; }
    int getMaxOpenListSize() const { return max_open_list_size_; }

    std::vector<MapNode> extractKeyPoints(const std::vector<MapNode>& path, double maxConnectionDist = 5.0) const;
    
private:
    const GridMap& grid_map_;
    
    // 优先队列比较函数
    struct NodeCompare {
        bool operator()(const MapNode& a, const MapNode& b) const {
            return a < b;
        }
    };
    
    // 使用涂色数组替代CloseList
    bool *close_list;
    
    // 搜索统计
    // 已探索节点数
    int nodes_explored_;
    // 最大开放列表大小
    int max_open_list_size_;
    
    // 初始化搜索状态
    void initializeSearch();
    
    // 检查节点是否有效（在地图内且非障碍物）
    bool isValidNode(const MapNode& node) const;
    
    // 回溯构建路径
    std::vector<MapNode> reconstructPath(const MapNode& endNode) const;

    // 视线检查方法
    bool isLineOfSightClear(const MapNode& start, const MapNode& end) const;
    double calcDistance(const MapNode& a, const MapNode& b) const;

};

#endif // ASTAR_H