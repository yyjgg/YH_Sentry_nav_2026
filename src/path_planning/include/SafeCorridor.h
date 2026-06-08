#ifndef SAFECORRIDOR_H
#define SAFECORRIDOR_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "GridMap.h"

/**
 * @brief 安全走廊类，用于在二维栅格地图中构建机器人安全行驶区域
 */
class SafeTravelCorridor2D {
public:
    /**
     * @brief 二维凸多面体结构体，表示安全走廊的一个段
     */
    struct Polyhedron2D {
        Eigen::MatrixXd A;           // 约束矩阵，每行代表一个面的法向量 A ∈ R^(m×2)
        Eigen::VectorXd b;           // 约束向量，每行代表对应面的偏移量 b ∈ R^m
        std::vector<Eigen::Vector2d> vertices; // 多面体的顶点集合
        
        Polyhedron2D() = default;
        
        /**
         * @brief 检查点是否在多面体内
         * @param point 待检查的点坐标
         * @param tolerance 数值容差，用于处理浮点数精度问题
         * @return true如果点在多面体内，false否则
         */
        bool contains(const Eigen::Vector2d& point, double tolerance = 1e-6) const;
    };

    /**
     * @brief 构造函数
     * @param grid_map 栅格地图的共享指针
     * @param safety_margin 安全边界距离，用于保持与障碍物的安全距离
     */
    SafeTravelCorridor2D(std::shared_ptr<GridMap> grid_map, double safety_margin = 0.0);
    
    /**
     * @brief 构建安全行驶走廊的主接口
     * @param path 机器人行驶路径，由一系列二维点组成
     * @param num_directions 扩展方向数量，决定多面体的精度
     * @param max_expansion 最大扩展距离，限制多面体的大小
     * @return 安全行驶走廊，由一系列凸多面体组成
     * @throws std::invalid_argument 当路径为空时抛出
     * @throws std::runtime_error 当栅格地图无效时抛出
     */
    std::vector<Polyhedron2D> buildCorridor(const std::vector<Eigen::Vector2d>& path, 
                                            int num_directions = 16, 
                                            double max_expansion = 20.0);

private:
    std::shared_ptr<GridMap> grid_map_;  // 栅格地图指针，用于障碍物检测
    double safety_margin_;               // 安全边界距离，确保与障碍物保持安全距离
    
    /**
     * @brief 种子扩展算法，为路径点构建凸多面体
     * @param seed_point 种子点坐标，路径上的一个点
        * @param directions 预生成的扩展方向集合
     * @param max_expansion 最大扩展距离
     * @return 构建的凸多面体
     */
    Polyhedron2D seedExpansion(const Eigen::Vector2d& seed_point, 
                            const std::vector<Eigen::Vector2d>& directions,
                              double max_expansion);
    
    /**
     * @brief 生成均匀分布的扩展方向
     * @param num_directions 需要的方向数量
     * @return 单位方向向量集合
     */
    std::vector<Eigen::Vector2d> generateExpansionDirections(int num_directions);
    
    /**
     * @brief 射线投射算法，计算沿某方向的最大安全距离
     * @param origin 射线起点坐标
     * @param direction 射线方向（单位向量）
     * @param max_distance 最大检测距离
     * @return 安全扩展距离
     */
    double rayCasting(const Eigen::Vector2d& origin, 
                     const Eigen::Vector2d& direction, 
                     double max_distance);
    
    /**
     * @brief 构建凸包（使用礼品包装算法）
     * @param points 输入点集
     * @return 凸包多面体
     */
    Polyhedron2D buildConvexHull(const std::vector<Eigen::Vector2d>& points);
    
    /**
     * @brief 计算三点叉积（用于凸包构建）
     * @param a 点a坐标
     * @param b 点b坐标  
     * @param c 点c坐标
     * @return 叉积值，正数表示c在ab左侧，负数表示右侧
     */
    static double crossProduct(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c);
    
    /**
     * @brief 从凸包顶点提取多面体表示
     * @param hull_points 凸包顶点集合
     * @return 多面体结构
     */
    Polyhedron2D extractPolyhedronFromHull(const std::vector<Eigen::Vector2d>& hull_points);
    
    /**
     * @brief 创建保守的多边形（当点太少无法构建正常凸包时）
     * @param points 输入点集
     * @return 保守的多边形
     */
    Polyhedron2D createConservativePolygon(const std::vector<Eigen::Vector2d>& points);
    
    /**
     * @brief 确保走廊段之间的连接性
     * @param corridor 初始走廊
     * @return 优化后的连接走廊
     */
    std::vector<Polyhedron2D> ensureConnectivity(const std::vector<Polyhedron2D>& corridor);
    
    /**
     * @brief 检查两个多面体是否有交集
     * @param poly1 第一个多面体
     * @param poly2 第二个多面体
     * @return true如果有交集，false否则
     */
    bool checkIntersection(const Polyhedron2D& poly1, const Polyhedron2D& poly2);
    
    /**
     * @brief 检查两条线段是否相交
     * @param p1 第一条线段的第一个端点
     * @param p2 第一条线段的第二个端点
     * @param q1 第二条线段的第一个端点
     * @param q2 第二条线段的第二个端点
     * @return true如果线段相交，false否则
     */
    bool lineSegmentsIntersect(const Eigen::Vector2d& p1, 
                              const Eigen::Vector2d& p2, 
                              const Eigen::Vector2d& q1, 
                              const Eigen::Vector2d& q2);
    
    /**
     * @brief 重新构建段以确保与前一多面体重叠
     * @param previous 前一个多面体
     * @param current 当前多面体
     * @return 重建后的多面体
     */
    Polyhedron2D rebuildSegmentWithOverlap(const Polyhedron2D& previous, const Polyhedron2D& current);
};

#endif // SAFECORRIDOR_H