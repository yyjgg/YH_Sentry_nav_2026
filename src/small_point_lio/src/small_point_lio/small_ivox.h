#pragma once

#include "../ankerl/unordered_dense.h"  // 高性能哈希表库
#include <small_point_lio/pch.h>

namespace small_point_lio {

    // 体素位置索引的哈希函数声明
    inline uint64_t hash_position_index(const Eigen::Matrix<int16_t, 3, 1> &v);

    // 点与距离的包装类，用于最近邻搜索排序
    class PointWithDistance {
    public:
        Eigen::Vector3f point;     // 点坐标
        float distance = 0;         // 到查询点的距离

        PointWithDistance(Eigen::Vector3f point, float distance);

        // 函数对象，用于比较两点距离（用于排序算法）
        bool operator()(const PointWithDistance &p1, const PointWithDistance &p2) const;

        // 小于运算符重载，用于排序
        bool operator<(const PointWithDistance &rhs) const;
    };

    // 增量体素地图核心类
    class SmallIVox {
    private:
        float inv_resolution;       // 分辨率倒数，用于快速计算体素索引
        size_t capacity;            // 地图最大容量（点数）
        std::list<Eigen::Vector3f> grids_cache_;   // 点云数据缓存链表（LRU缓存）
        std::vector<PointWithDistance> candidates; // 最近邻候选点临时存储

    public:
        explicit SmallIVox(float resolution, size_t capacity);  // 构造函数

        bool add_point(const Eigen::Vector3f &point_to_add);    // 添加点到地图

        // 查找距离给定点最近的几个点
        void get_closest_point(const Eigen::Vector3f &pt, std::vector<Eigen::Vector3f> &closest_pt, size_t max_num = 5);

        // 哈希表：体素索引 -> 点在链表中的迭代器（实现LRU缓存）
        ankerl::unordered_dense::map<uint64_t, std::list<Eigen::Vector3f>::iterator> grids_map;
        
        // 计算点所在的体素索引
        [[nodiscard]] Eigen::Matrix<uint16_t, 3, 1> get_position_index(const Eigen::Vector3f &pt) const;
    };

    // 体素位置索引哈希函数实现（将3D坐标编码为64位哈希值）
    inline __attribute__((always_inline)) uint64_t hash_position_index(const Eigen::Matrix<uint16_t, 3, 1> &v) {
        return (static_cast<uint64_t>(v[0]) << 32) |  // x坐标占高32位
               (static_cast<uint64_t>(v[1]) << 16) |  // y坐标占中16位  
               static_cast<uint64_t>(v[2]);           // z坐标占低16位
    }

    // PointWithDistance构造函数
    inline __attribute__((always_inline)) PointWithDistance::PointWithDistance(Eigen::Vector3f point, float distance)
        : point(std::move(point)),    // 移动语义提高效率
          distance(distance) {}

    // 函数对象实现：按距离比较
    inline __attribute__((always_inline)) bool PointWithDistance::operator()(const PointWithDistance &p1, const PointWithDistance &p2) const {
        return p1.distance < p2.distance;  // 距离小的优先
    }

    // 小于运算符重载
    inline __attribute__((always_inline)) bool PointWithDistance::operator<(const PointWithDistance &rhs) const {
        return distance < rhs.distance;  // 用于排序
    }

    // SmallIVox构造函数
    inline __attribute__((always_inline)) SmallIVox::SmallIVox(float resolution, size_t capacity)
        : inv_resolution(1 / resolution),   // 预计算倒数，避免除法开销
          capacity(capacity) {}             // 设置最大容量

    // 核心功能：查找最近邻点
    inline __attribute__((always_inline)) void SmallIVox::get_closest_point(const Eigen::Vector3f &pt, 
                                                                           std::vector<Eigen::Vector3f> &closest_pt, 
                                                                           size_t max_num) {
        closest_pt.clear();
        Eigen::Matrix<uint16_t, 3, 1> key = get_position_index(pt);  // 计算查询点所在体素
        uint64_t hash_key = hash_position_index(key);              // 计算体素哈希值
        
        ankerl::unordered_dense::map<uint64_t, std::list<Eigen::Vector3f>::iterator>::iterator iter;

        // 搜索3x3x3邻域内的27个体素（实际实现搜索7个关键邻域）
        {
            iter = grids_map.find(hash_key);  // 中心体素
            if (iter != grids_map.end()) {
                closest_pt.push_back(*iter->second);
            }
        }
        {
            // x方向+1邻域：修改x坐标低位
            iter = grids_map.find((hash_key & 0xFFFFFFFFFFFF0000) | ((hash_key + 1L) & 0x000000000000FFFF));
            if (iter != grids_map.end()) {
                closest_pt.push_back(*iter->second);
            }
        }
        {
            // x方向-1邻域
            iter = grids_map.find((hash_key & 0xFFFFFFFFFFFF0000) | ((hash_key - 1L) & 0x000000000000FFFF));
            if (iter != grids_map.end()) {
                closest_pt.push_back(*iter->second);
            }
        }
        {
            // y方向+1邻域：修改y坐标中位
            iter = grids_map.find((hash_key & 0xFFFFFFFF0000FFFF) | ((hash_key + (1L << 16)) & 0x00000000FFFF0000));
            if (iter != grids_map.end()) {
                closest_pt.push_back(*iter->second);
            }
        }
        {
            // y方向-1邻域
            iter = grids_map.find((hash_key & 0xFFFFFFFF0000FFFF) | ((hash_key - (1L << 16)) & 0x00000000FFFF0000));
            if (iter != grids_map.end()) {
                closest_pt.push_back(*iter->second);
            }
        }
        {
            // z方向+1邻域：修改z坐标高位
            iter = grids_map.find((hash_key & 0xFFFF0000FFFFFFFF) | ((hash_key + (1L << 32)) & 0x0000FFFF00000000));
            if (iter != grids_map.end()) {
                closest_pt.push_back(*iter->second);
            }
        }
        {
            // z方向-1邻域
            iter = grids_map.find((hash_key & 0xFFFF0000FFFFFFFF) | ((hash_key - (1L << 32)) & 0x0000FFFF00000000));
            if (iter != grids_map.end()) {
                closest_pt.push_back(*iter->second);
            }
        }

        // 如果找到的点太多，进行排序和截断
        if (closest_pt.size() > max_num) [[likely]] { 
            candidates.clear();
            // 计算所有候选点到查询点的平方距离
            for (auto &point: closest_pt) {
                candidates.emplace_back(point, (point - pt).squaredNorm());
            }
            // 部分排序：找到前max_num个最近点（比完全排序更快）
            std::nth_element(candidates.begin(), 
                           candidates.begin() + static_cast<std::ptrdiff_t>(max_num) - 1, 
                           candidates.end());
            // 保留最近的max_num个点
            closest_pt.clear();
            for (auto &it: candidates) {
                closest_pt.push_back(it.point);
            }
        }
    }

    // 添加点到地图（实现LRU缓存策略）
    inline __attribute__((always_inline)) bool SmallIVox::add_point(const Eigen::Vector3f &point) {
        Eigen::Matrix<uint16_t, 3, 1> key = get_position_index(point);
        auto hash_key = hash_position_index(key);
        auto iter = grids_map.find(hash_key);
        
        if (iter != grids_map.end()) {
            // 点已存在：移动到链表头部（LRU最近使用）
            grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
            grids_map[hash_key] = grids_cache_.begin(); 
            return false;  // 返回false表示不是新点
        } else {
            // 新点：添加到链表头部
            grids_cache_.push_front(point);
            grids_map.emplace(hash_key, grids_cache_.begin());
            
            // 如果超过容量，移除最久未使用的点（链表尾部）
            if (grids_map.size() >= capacity) {
                grids_map.erase(hash_position_index(get_position_index(grids_cache_.back())));
                grids_cache_.pop_back();
            }
            return true;  // 返回true表示是新点
        }
    }

    // 计算点对应的体素索引
    [[nodiscard]] inline __attribute__((always_inline)) Eigen::Matrix<uint16_t, 3, 1> 
    SmallIVox::get_position_index(const Eigen::Vector3f &pt) const {
        return (pt * inv_resolution).array().floor().cast<uint16_t>();  // 坐标除以分辨率后取整
    }

}// namespace small_point_lio