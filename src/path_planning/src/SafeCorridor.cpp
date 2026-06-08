#include "SafeCorridor.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>

namespace {

struct AABB2D {
    Eigen::Vector2d min_pt;
    Eigen::Vector2d max_pt;
};

inline AABB2D computeAABB(const std::vector<Eigen::Vector2d>& vertices) {
    AABB2D box;
    if (vertices.empty()) {
        box.min_pt.setZero();
        box.max_pt.setZero();
        return box;
    }

    box.min_pt = vertices.front();
    box.max_pt = vertices.front();
    for (const auto& v : vertices) {
        box.min_pt = box.min_pt.cwiseMin(v);
        box.max_pt = box.max_pt.cwiseMax(v);
    }
    return box;
}

inline bool aabbOverlap(const AABB2D& a, const AABB2D& b, double eps = 1e-6) {
    if (a.max_pt.x() < b.min_pt.x() - eps || b.max_pt.x() < a.min_pt.x() - eps) {
        return false;
    }
    if (a.max_pt.y() < b.min_pt.y() - eps || b.max_pt.y() < a.min_pt.y() - eps) {
        return false;
    }
    return true;
}

} // namespace

// Polyhedron2D 成员函数实现
bool SafeTravelCorridor2D::Polyhedron2D::contains(const Eigen::Vector2d& point, double tolerance) const {
    if (A.rows() == 0) return false;  // 如果没有约束面，返回false

    // 逐行检查约束，避免每次调用时创建临时向量
    for (int i = 0; i < A.rows(); ++i) {
        if (A.row(i).dot(point) - b(i) > tolerance) {
            return false;
        }
    }
    return true;
}

// SafeTravelCorridor2D 成员函数实现
SafeTravelCorridor2D::SafeTravelCorridor2D(std::shared_ptr<GridMap> grid_map, double safety_margin)
    : grid_map_(std::move(grid_map)), safety_margin_(safety_margin) {
    // 参数验证
    if (safety_margin_ <= 0) {
        safety_margin_ = 0.0;  // 默认安全距离
    };
}

std::vector<SafeTravelCorridor2D::Polyhedron2D> SafeTravelCorridor2D::buildCorridor(
    const std::vector<Eigen::Vector2d>& path, 
    int num_directions, 
    double max_expansion) {
    
    // 参数验证
    if (path.empty()) {
        throw std::invalid_argument("路径不能为空");
    }
    
    if (!grid_map_ || !grid_map_->isValid()) {
        throw std::runtime_error("栅格地图无效");
    }
    std::vector<Polyhedron2D> corridor;  // 安全走廊容器
    corridor.reserve(path.size());

    const std::vector<Eigen::Vector2d> directions = generateExpansionDirections(num_directions);
    
    // 对路径上的每个点构建凸多面体
    for (const auto& seed_point : path) {
        corridor.emplace_back(seedExpansion(seed_point, directions, max_expansion));
    }
    
    // 确保走廊的连接性并返回
    return ensureConnectivity(corridor);
}

SafeTravelCorridor2D::Polyhedron2D SafeTravelCorridor2D::seedExpansion(
    const Eigen::Vector2d& seed_point, 
    const std::vector<Eigen::Vector2d>& directions,
    double max_expansion) {

    const int num_directions = static_cast<int>(directions.size());
    std::vector<Eigen::Vector2d> expanded_points;  // 扩展点集合
    expanded_points.reserve(directions.size());
    
    // 沿每个方向进行扩展
    std::vector<double> distances;
    distances.reserve(directions.size());
    double min_distance = std::numeric_limits<double>::max();
    
    // 记录所有方向的距离并找到最小值
    for (const auto& direction : directions) {
        double distance = rayCasting(seed_point, direction, max_expansion);
        distances.push_back(distance);
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    
    std::vector<int> obstacle_dirs;
    obstacle_dirs.reserve(directions.size());
    for (size_t i = 0; i < distances.size(); ++i) {
        if (distances[i] == min_distance) {
            obstacle_dirs.push_back(static_cast<int>(i));
        }
    }

    for (int i = 0; i < num_directions; ++i) {
        // 计算当前方向与每个障碍物方向的夹角，判断是否在任意一个障碍物方向的180度范围内
        bool in_obstacle_range = false;
        for (size_t j = 0; j < obstacle_dirs.size(); ++j) {
            // 计算当前方向与障碍物方向的夹角索引差
            size_t angle_diff = std::abs(i - obstacle_dirs[j]);
            // 取圆周内的最小夹角索引差
            angle_diff = std::min(angle_diff, num_directions - angle_diff);
            // 180度为360度的一半，顺时针逆时针方向再各一半占90度，所以除以四
            if (angle_diff > 0 && angle_diff < directions.size()/4) {
                in_obstacle_range = true;
                break;
            }
        }
        if (in_obstacle_range ) {
            Eigen::Vector2d expanded_point = seed_point + min_distance * directions[i];
            expanded_points.push_back(expanded_point);
        } 
        else {
            // 其他方向使用实际距离
            Eigen::Vector2d expanded_point = seed_point + distances[i] * directions[i];
            expanded_points.push_back(expanded_point);
        }
    }

    // 构建凸包并返回
    return buildConvexHull(expanded_points);
}

std::vector<Eigen::Vector2d> SafeTravelCorridor2D::generateExpansionDirections(int num_directions) {
    std::vector<Eigen::Vector2d> directions;  // 方向向量容器
    directions.reserve(std::max(0, num_directions));
    
    // 生成均匀的角度分布
    for (int i = 0; i < num_directions; ++i) {
        double angle = 2.0 * M_PI * i / num_directions;  // 等间隔角度
        directions.emplace_back(std::cos(angle), std::sin(angle));
    }
    
    return directions;
}

double SafeTravelCorridor2D::rayCasting(
    const Eigen::Vector2d& origin, 
    const Eigen::Vector2d& direction, 
    double max_distance) {
    
    double min_distance = max_distance;  // 初始化为最大距离
    double step_size = 0.2;              // 射线采样步长
    int max_steps = static_cast<int>(max_distance / step_size);  // 最大采样步数
    
    // 沿着射线方向采样检测障碍物
    for (int step = 1; step <= max_steps; ++step) {
        double current_distance = step * step_size;  // 当前采样距离
        Eigen::Vector2d test_point = origin + current_distance * direction;  // 测试点坐标
        
        // 将连续坐标转换为栅格坐标
        int grid_x = static_cast<int>(std::round(test_point.x()));
        int grid_y = static_cast<int>(std::round(test_point.y()));
        
        // 检查是否超出地图边界
        if (!grid_map_->isWithinBounds(grid_x, grid_y)) {
            min_distance = current_distance;
            break;
        }
        
        // 检查是否为障碍物
        if (grid_map_->isInflatedObstacle(grid_x, grid_y) || grid_map_->isObstacle(grid_x, grid_y)) {
            min_distance = current_distance ;  // 减去安全距离
            break;
        }
    }
    
    return std::max(0.1, min_distance);  // 确保最小距离为正
}

SafeTravelCorridor2D::Polyhedron2D SafeTravelCorridor2D::buildConvexHull(
    const std::vector<Eigen::Vector2d>& points) {
    
    // 如果点太少，返回保守多边形
    if (points.size() < 3) {
        return createConservativePolygon(points);
    }
    
    // 复制点集以避免修改原数据
    std::vector<Eigen::Vector2d> working_points = points;
    
    size_t leftmost_index = 0;  // 最左边点的索引
    
    // 找到最左边的点（x最小，如果x相同则y最小）
    for (size_t i = 1; i < working_points.size(); ++i) {
        if (working_points[i].x() < working_points[leftmost_index].x() ||
            (working_points[i].x() == working_points[leftmost_index].x() && 
             working_points[i].y() < working_points[leftmost_index].y())) {
            leftmost_index = i;
        }
    }
    
    std::vector<Eigen::Vector2d> hull_points;  // 凸包顶点集合
    size_t current_index = leftmost_index;     // 当前处理的顶点索引
    
    // 礼品包装算法构建凸包
    do {
        hull_points.push_back(working_points[current_index]);
        size_t next_index = (current_index + 1) % working_points.size();  // 初始下一个顶点
        
        // 寻找最右侧的点
        for (size_t i = 0; i < working_points.size(); ++i) {
            if (i == current_index || i == next_index) continue;
            
            // 如果点i在current->next向量的右侧，更新next_index
            double cross = crossProduct(working_points[current_index], 
                                      working_points[next_index], 
                                      working_points[i]);
            if (cross < 0) {
                next_index = i;
            }
            
        }
        
        current_index = next_index;
        
        // 防止无限循环
        if (hull_points.size() > working_points.size()) {
            break;
        }
        
    } while (current_index != leftmost_index);

    // 从凸包顶点提取多面体
    return extractPolyhedronFromHull(hull_points);
}

double SafeTravelCorridor2D::crossProduct(
    const Eigen::Vector2d& a, 
    const Eigen::Vector2d& b, 
    const Eigen::Vector2d& c) {
    
    // 计算向量ab和ac的叉积
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

SafeTravelCorridor2D::Polyhedron2D SafeTravelCorridor2D::extractPolyhedronFromHull(
    const std::vector<Eigen::Vector2d>& hull_points) {
    
    Polyhedron2D polyhedron;
    polyhedron.vertices = hull_points;  // 设置顶点
    
    size_t num_edges = hull_points.size();  // 边数等于顶点数
    polyhedron.A.resize(num_edges, 2);     // 调整约束矩阵大小
    polyhedron.b.resize(num_edges);        // 调整约束向量大小
    
    // 为每条边计算约束
    for (size_t i = 0; i < num_edges; ++i) {
        size_t next_i = (i + 1) % num_edges;  // 下一个顶点索引
        
        // 计算边的方向向量
        Eigen::Vector2d edge_dir = hull_points[next_i] - hull_points[i];
        
        // 计算法向量（指向多边形内部）
        Eigen::Vector2d normal(-edge_dir.y(), edge_dir.x());  // 旋转90度得到法向量
        normal.normalize();  // 单位化
        
        // 设置约束：normal·x <= normal·p
        polyhedron.A.row(i) = normal;
        polyhedron.b(i) = normal.dot(hull_points[i]);
    }
    
    return polyhedron;
}

SafeTravelCorridor2D::Polyhedron2D SafeTravelCorridor2D::createConservativePolygon(
    const std::vector<Eigen::Vector2d>& points) {
    
    Polyhedron2D polyhedron;
    
    // 如果没有点，创建以原点为中心的正方形
    if (points.empty()) {
        double size = 1.0;  // 正方形大小
        std::vector<Eigen::Vector2d> conservative_points = {
            Eigen::Vector2d(-size, -size),
            Eigen::Vector2d(size, -size), 
            Eigen::Vector2d(size, size),
            Eigen::Vector2d(-size, size)
        };
        return extractPolyhedronFromHull(conservative_points);
    }
    
    // 以第一个点为中心创建正方形
    Eigen::Vector2d center = points[0];
    double size = 2.0;  // 正方形大小
    std::vector<Eigen::Vector2d> conservative_points = {
        center + Eigen::Vector2d(-size, -size),
        center + Eigen::Vector2d(size, -size),
        center + Eigen::Vector2d(size, size),
        center + Eigen::Vector2d(-size, size)
    };
    
    return extractPolyhedronFromHull(conservative_points);
}

std::vector<SafeTravelCorridor2D::Polyhedron2D> SafeTravelCorridor2D::ensureConnectivity(
    const std::vector<Polyhedron2D>& corridor) {
    
    // 如果走廊段数小于等于1，直接返回
    if (corridor.size() <= 1) return corridor;
    
    std::vector<Polyhedron2D> optimized_corridor;  // 优化后的走廊
    optimized_corridor.reserve(corridor.size());
    optimized_corridor.push_back(corridor[0]);     // 添加第一个段
    
    // 遍历后续段，确保连接性
    for (size_t i = 1; i < corridor.size(); ++i) {
        Polyhedron2D current_segment = corridor[i];        // 当前段
        Polyhedron2D previous_segment = optimized_corridor.back();  // 前一段
        
        // 检查连接性，如果没有交集则重建当前段
        if (!checkIntersection(previous_segment, current_segment)) {
            current_segment = rebuildSegmentWithOverlap(previous_segment, current_segment);
        }
        
        optimized_corridor.push_back(current_segment);
    }
    
    return optimized_corridor;
}

bool SafeTravelCorridor2D::checkIntersection(const Polyhedron2D& poly1, const Polyhedron2D& poly2) {
    // 如果任一多边形为空，返回false
    if (poly1.vertices.empty() || poly2.vertices.empty()) {
        return false;
    }

    // 包围盒快速排斥，显著减少后续 contains/边相交计算
    const AABB2D box1 = computeAABB(poly1.vertices);
    const AABB2D box2 = computeAABB(poly2.vertices);
    if (!aabbOverlap(box1, box2)) {
        return false;
    }
    
    // 检查多边形顶点是否在另一多边形内部
    for (const auto& vertex : poly1.vertices) {
        if (poly2.contains(vertex)) {
            return true;
        }
    }
    
    for (const auto& vertex : poly2.vertices) {
        if (poly1.contains(vertex)) {
            return true;
        }
    }
    
    // 检查边与边的相交情况
    for (size_t i = 0; i < poly1.vertices.size(); ++i) {
        Eigen::Vector2d p1 = poly1.vertices[i];
        Eigen::Vector2d p2 = poly1.vertices[(i + 1) % poly1.vertices.size()];
        
        for (size_t j = 0; j < poly2.vertices.size(); ++j) {
            Eigen::Vector2d q1 = poly2.vertices[j];
            Eigen::Vector2d q2 = poly2.vertices[(j + 1) % poly2.vertices.size()];
            
            // 检查线段p1p2和q1q2是否相交
            if (lineSegmentsIntersect(p1, p2, q1, q2)) {
                return true;
            }
        }
    }
    
    return false;
}

// 新增辅助函数：检查两条线段是否相交
bool SafeTravelCorridor2D::lineSegmentsIntersect(const Eigen::Vector2d& p1, 
                                                 const Eigen::Vector2d& p2, 
                                                 const Eigen::Vector2d& q1, 
                                                 const Eigen::Vector2d& q2) {
    const double min_px = std::min(p1.x(), p2.x());
    const double max_px = std::max(p1.x(), p2.x());
    const double min_py = std::min(p1.y(), p2.y());
    const double max_py = std::max(p1.y(), p2.y());

    const double min_qx = std::min(q1.x(), q2.x());
    const double max_qx = std::max(q1.x(), q2.x());
    const double min_qy = std::min(q1.y(), q2.y());
    const double max_qy = std::max(q1.y(), q2.y());

    if (max_px < min_qx || max_qx < min_px || max_py < min_qy || max_qy < min_py) {
        return false;
    }

    // 使用向量叉积判断线段是否相交
    double r_s = ((p2.x() - p1.x()) * (q2.y() - q1.y()) - (p2.y() - p1.y()) * (q2.x() - q1.x()));
    
    if (std::abs(r_s) < 1e-6) {  // 平行或共线
        return false;
    }
    
    double t = ((q1.x() - p1.x()) * (q2.y() - q1.y()) - (q1.y() - p1.y()) * (q2.x() - q1.x())) / r_s;
    double u = ((q1.x() - p1.x()) * (p2.y() - p1.y()) - (q1.y() - p1.y()) * (p2.x() - p1.x())) / r_s;
    
    return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
}

SafeTravelCorridor2D::Polyhedron2D SafeTravelCorridor2D::rebuildSegmentWithOverlap(
    const Polyhedron2D& previous, const Polyhedron2D& current) {
    
    if (current.vertices.empty()) {
        return current;
    }
    
    // 计算两个多边形的重心
    Eigen::Vector2d prev_centroid(0, 0);
    for (const auto& v : previous.vertices) {
        prev_centroid += v;
    }
    prev_centroid /= static_cast<double>(previous.vertices.size());
    
    Eigen::Vector2d curr_centroid(0, 0);
    for (const auto& v : current.vertices) {
        curr_centroid += v;
    }
    curr_centroid /= static_cast<double>(current.vertices.size());
    
    // 计算两个重心之间的距离
    double distance = (prev_centroid - curr_centroid).norm();
    
    // 如果距离很小，尝试扩大当前多边形
    if (distance < safety_margin_) {
        std::vector<Eigen::Vector2d> expanded_vertices = current.vertices;
        
        // 扩大当前多边形
        for (auto& vertex : expanded_vertices) {
            Eigen::Vector2d direction = vertex - curr_centroid;  // 从重心指向顶点的方向
            direction.normalize();
            vertex += direction * safety_margin_;  // 扩展
        }
        
        return buildConvexHull(expanded_vertices);
    } else {
        // 否则，移动当前多边形使其与前一个多边形重叠
        std::vector<Eigen::Vector2d> moved_vertices = current.vertices;
        
        // 计算需要移动的距离（使两个多边形有重叠）
        Eigen::Vector2d direction = prev_centroid - curr_centroid;
        direction.normalize();
        
        // 将当前多边形沿着连接两个重心的方向移动一定距离
        for (auto& vertex : moved_vertices) {
            vertex += direction * (distance * 0.5) + direction * safety_margin_;
        }
        
        return buildConvexHull(moved_vertices);
    }
}