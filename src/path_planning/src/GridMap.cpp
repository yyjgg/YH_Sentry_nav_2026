#include "GridMap.h"
#include <algorithm>
#include <cmath>

GridMap::GridMap(int size_x, int size_y, int inflation_radius) 
    : size_x_(size_x), size_y_(size_y), inflation_radius_(std::max(1, inflation_radius)) {
        // int数组存储障碍物类型：0=无障碍, 1=原始障碍物, 2=膨胀区域
        l_data = new int[size_x*size_y];
        memset(l_data, 0, size_x*size_y * sizeof(int));
}

void GridMap::setObstacle(int x, int y, bool is_obstacle) {
    if (isWithinBounds(x, y)) {
        if (is_obstacle) {
            // 如果已经是障碍物则无需处理
            if (l_data[x*size_y_+y] == 1) return;
            
            // 设置为原始障碍物
            l_data[x*size_y_+y] = 1;
            // 添加膨胀区域
            setInflatedObstacle(x, y, inflation_radius_);
        } else {
            // 如果不是障碍物则无需处理
            if (l_data[x*size_y_+y] == 0) return;
            
            // 清除该点产生的膨胀区域
            clearInflatedObstacle(x, y, inflation_radius_);
            // 清除原始障碍物
            l_data[x*size_y_+y] = 0;
        }
    }
}

bool GridMap::isObstacle(int x, int y) const {
    if (isWithinBounds(x, y) && l_data[x*size_y_+y] == 1) {
        return true;
    }  
    return false; 
}

bool GridMap::isInflatedObstacle(int x, int y) const {
    if (isWithinBounds(x, y) && l_data[x*size_y_+y] == 2) {
        return true;
    }  
    return false; 
}

// 清除特定区域的膨胀区域，但不影响原始障碍物
void GridMap::clearInflatedAreaOnly(int cx, int cy, int radius) {
    if (!isWithinBounds(cx, cy)) return;
    
    for (int dx = -radius; dx <= radius; dx++) {
        for (int dy = -radius; dy <= radius; dy++) {
            int x = cx + dx;
            int y = cy + dy;
            
            if (isWithinBounds(x, y)) {
                // 只清除膨胀区域，保留原始障碍物
                if (l_data[x * size_y_ + y] == 2) {
                    l_data[x * size_y_ + y] = 0; // 恢复为自由空间
                }
            }
        }
    }
}

void GridMap::setInflatedObstacle(int x, int y, int inflation_distance) {
    // 膨胀区域设置函数，inflation_distance表示距离原始障碍物的距离(1-3格)
    if (!isWithinBounds(x, y)) return;
    
    // 限制膨胀距离至少为1格
    if (inflation_distance < 1) return;
    
    // 在原始障碍物周围创建膨胀区域
    for (int dx = -inflation_distance; dx <= inflation_distance; dx++) {
        for (int dy = -inflation_distance; dy <= inflation_distance; dy++) {
            int Inflat_x = x + dx;
            int Inflat_y = y + dy;
            
            // 跳过原始障碍物位置本身
            if (dx == 0 && dy == 0) continue;
            
            // 检查边界
            if (!isWithinBounds(Inflat_x, Inflat_y)) continue;
            
            // 计算当前点到原始障碍物的切比雪夫距离
            int distance = std::max(std::abs(dx), std::abs(dy));
            
            // 只处理在指定膨胀范围内的点
            if (distance <= inflation_distance) {
                // 仅当不是原始障碍物时才设置为膨胀区域
                if (l_data[Inflat_x * size_y_ + Inflat_y] != 1) {
                    l_data[Inflat_x * size_y_ + Inflat_y] = 2; // 设置为膨胀区域  
                }                   
            }
        }
    }
}

void GridMap::clearInflatedObstacle(int x, int y, int inflation_distance) {
    // 膨胀区域清除函数
    if (!isWithinBounds(x, y)) return;
    
    // 限制膨胀距离至少为1格
    if (inflation_distance < 1) return;
    
    // 在原始障碍物周围清除膨胀区域
    for (int dx = -inflation_distance; dx <= inflation_distance; dx++) {
        for (int dy = -inflation_distance; dy <= inflation_distance; dy++) {
            int clear_x = x + dx;
            int clear_y = y + dy;
            
            // 跳过原始障碍物位置本身
            if (dx == 0 && dy == 0) continue;
            
            // 检查边界
            if (!isWithinBounds(clear_x, clear_y)) continue;
            
            // 计算当前点到原始障碍物的切比雪夫距离
            int distance = std::max(std::abs(dx), std::abs(dy));
            
            if (distance <= inflation_distance) {
                // 仅清除膨胀区域（不包括原始障碍物）
                if (l_data[clear_x * size_y_ + clear_y] == 2) {
                    // 需要检查其他障碍物是否也会使此点成为膨胀区域
                    bool still_inflated_by_other_obstacle = false;
                    
                    // 搜索附近的其他障碍物
                    for (int ox = std::max(0, clear_x - inflation_distance); 
                         ox < std::min(size_x_, clear_x + inflation_distance + 1); ox++) {
                        for (int oy = std::max(0, clear_y - inflation_distance); 
                             oy < std::min(size_y_, clear_y + inflation_distance + 1); oy++) {
                            
                            // 跳过当前正在删除的障碍物
                            if (ox == x && oy == y) continue;
                            
                            // 检查是否有其他障碍物存在
                            if (l_data[ox * size_y_ + oy] == 1) {
                                // 计算此障碍物到当前点的距离
                                int dist_x = std::abs(ox - clear_x);
                                int dist_y = std::abs(oy - clear_y);
                                int dist = std::max(dist_x, dist_y); // 切比雪夫距离
                                
                                if (dist <= inflation_distance) {
                                    still_inflated_by_other_obstacle = true;
                                    break;
                                }
                            }
                        }
                        if (still_inflated_by_other_obstacle) break;
                    }
                    
                    // 只有当没有其他障碍物影响此点时，才将其恢复为自由空间
                    if (!still_inflated_by_other_obstacle) {
                        l_data[clear_x * size_y_ + clear_y] = 0;
                    }
                }
            }
        }
    }
}
// 保存指定区域的状态
void GridMap::saveAreaState(int cx, int cy, int radius, std::vector<std::tuple<int, int, int>>& saved_states) {
    if (!isWithinBounds(cx, cy)) return;
    
    saved_states.clear(); // 清空之前的状态
    
    for (int dx = -radius; dx <= radius; dx++) {
        for (int dy = -radius; dy <= radius; dy++) {
            int x = cx + dx;
            int y = cy + dy;
            
            if (isWithinBounds(x, y)) {
                // 检查该坐标是否已经保存过了，避免重复保存
                bool already_exists = false;
                for (const auto& state : saved_states) {
                    if (std::get<0>(state) == x && std::get<1>(state) == y) {
                        already_exists = true;
                        break;
                    }
                }
                
                if (!already_exists) {
                    // 保存坐标和当前状态
                    saved_states.push_back(std::make_tuple(x, y, l_data[x * size_y_ + y]));
                }
            }
        }
    }
}

// 恢复指定区域的状态
void GridMap::restoreAreaState(const std::vector<std::tuple<int, int, int>>& saved_states) {
    for (const auto& state : saved_states) {
        int x = std::get<0>(state);
        int y = std::get<1>(state);
        int value = std::get<2>(state);
        
        if (isWithinBounds(x, y)) {
            l_data[x * size_y_ + y] = value;
        }
    }
}

bool GridMap::isWithinBounds(int x, int y) const {
    return x >= 0 && x < size_x_ && y >= 0 && y < size_y_;
}

