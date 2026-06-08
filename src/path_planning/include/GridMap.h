#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include <iostream>
#include <string.h> 
#include <stdint.h>
#include <deque>
#include <unordered_set>
#include "rclcpp/rclcpp.hpp"

struct grid{
    bool is_obstacle;
    double time;
    grid() : is_obstacle(false), time(0.0) {}
};
typedef grid*** grid_ptr;

/**
 * @brief 网格地图类，管理地图数据和障碍物信息
 */
class GridMap {
public:
    GridMap(int size_x, int size_y, int inflation_radius = 4);
    
    // 地图基本信息
    int getSizeX() const { return size_x_; }
    int getSizeY() const { return size_y_; }
    
    // 用于在地图中添加或移除障碍物
    void setObstacle(int x, int y, bool is_obstacle = true);
    // 检查指定坐标位置是否为障碍物
    bool isObstacle(int x, int y) const;
    // 验证坐标是否在地图的合法范围内
    bool isWithinBounds(int x, int y) const;
    // 用于在地图中添加障碍物膨胀区域
    void setInflatedObstacle(int x, int y, int inflation_distance);
    // 用于在地图中移除障碍物膨胀区域
    void clearInflatedObstacle(int x, int y, int inflation_distance);
    // 检查指定坐标位置是否为障碍物膨胀区域
    bool isInflatedObstacle(int x, int y) const;
    
    // 新增函数：清除特定区域的膨胀区域，但不影响原始障碍物
    void clearInflatedAreaOnly(int cx, int cy, int radius);
    
    // 新增函数：保存指定区域的状态
    void saveAreaState(int cx, int cy, int radius, std::vector<std::tuple<int, int, int>>& saved_states);
    
    // 新增函数：恢复指定区域的状态
    void restoreAreaState(const std::vector<std::tuple<int, int, int>>& saved_states);
    
    // 地图验证
    bool isValid() const { return size_x_ > 0 && size_y_ > 0; }
    // 维护最近1s的地图
    std::deque<std::vector<int>> history;
    
    // 用于访问私有数据的getter函数（仅用于A*算法）
    int* getGridData() const { return l_data; }
    int getGridCellValue(int x, int y) const { return l_data[x * size_y_ + y]; }
    void setGridCellValue(int x, int y, int value) { l_data[x * size_y_ + y] = value; }

private:
    int size_x_, size_y_;
    int inflation_radius_;
    int *l_data; // l_data存放局部地图的障碍物信息（l_data[i]为1说明i这个栅格是障碍物栅格）
    grid_ptr** partial_map;
};

#endif // GRIDMAP_H