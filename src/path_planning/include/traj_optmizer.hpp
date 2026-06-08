// trajopt
#ifndef TRAJ_OPTMIZER_HPP
#define TRAJ_OPTMIZER_HPP
#include "lbfgs.hpp"
#include "minco.hpp"
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <cfloat>
#include "SafeCorridor.h"

class TrajOpt{
public:
    struct Sample {
        Eigen::Vector2d pos; // 在TrajOpt内部使用的坐标单位（与输入path一致，通常为栅格坐标）
        Eigen::Vector2d vel; // 对应速度（同样的单位/秒）
        double t;            // 相对于轨迹起点的时间(秒)
    };
    typedef Eigen::Matrix2Xd PolyhedronV;  // 顶点表示的多面体，2×N的矩阵，N为顶点数量，每列：[x, y]^T
    typedef std::vector<PolyhedronV> PolyhedraV; // 顶点表示的安全走廊
    typedef Eigen::MatrixX3d PolyhedronH;  // H表示的多面体，M×3的矩阵，M为平面数量，每行：[a, b, c]表示约束ax + by ≤ c
    typedef std::vector<PolyhedronH> PolyhedraH; // 半平面表示的安全走廊

private:
    minco::MINCO_S3NU<2> minco;  // 使用MINCO_S3NU<2>，s=3对应minimum jerk
    
    double rho;                           // 时间权重系数，在优化问题中作为时间项的权重：cost += weightT * times.sum()
    Eigen::Matrix<double, 2, 3> headPVA;  // 轨迹起点状态约束 (位置、速度、加速度)
    Eigen::Matrix<double, 2, 3> tailPVA;  // 轨迹终点状态约束 (位置、速度、加速度)

    PolyhedraV vPolytopes;                // 顶点表示的安全走廊
    std::vector<SafeTravelCorridor2D::Polyhedron2D> safecorridors; // 原始安全走廊结构体
    Eigen::Matrix2Xd shortPath;           // 存储从起点到终点的初始路径点,2×N 矩阵，每列为一个2D坐标点 (x,y)

    Eigen::VectorXi pieceIdx;             // 向量，存储每个多面体区域内的轨迹分段数，pieceIdx(i)表示第 i个安全走廊内分配的多项式轨迹段数量
    Eigen::VectorXi vPolyIdx;             // 索引映射，关联轨迹段与对应的多面体
    Eigen::VectorXi hPolyIdx;             // H表示的索引映射

    int polyN;                           // 安全走廊数量
    int pieceN;                          // 整个轨迹的总分段数

    int spatialDim;                       // 空间优化变量的维度 = 控制点总数
    int temporalDim;                      // 时间优化变量的维度 = 时间分段数

    double smoothEps;
    int integralRes;                      // 积分分辨率（每段轨迹的积分采样点数），降低以提高优化稳定性
    Eigen::VectorXd magnitudeBd;          // 轨迹段内控制点速度的范数上限
    Eigen::VectorXd penaltyWt;            // 惩罚项权重向量 [位置惩罚权重, 速度惩罚权重]
    double allocSpeed = 5;                // 初始时间分配参考速度 (栅格/s) 

    lbfgs::lbfgs_parameter_t lbfgs_params;// 优化器参数

    Eigen::Matrix2Xd points;              // 轨迹中间控制点（优化变量,没有头尾），2×(pieceN-1) 矩阵，每列为一个中间控制点的2D坐标
    Eigen::VectorXd times;                // pieceN维向量，每段轨迹的时间分配
    Eigen::Matrix2Xd gradByPoints;        // 目标函数对控制点的梯度 ∂W/∂q
    Eigen::VectorXd gradByTimes;          // 目标函数对时间的梯度 ∂W/∂T  
    Eigen::MatrixX2d partialGradByCoeffs; // [∂J/∂c0x, ∂J/∂c0y; ... ∂J/∂c5x, ∂J/∂c5y]
    Eigen::VectorXd partialGradByTimes;   // ∂J/∂T0, ∂J/∂T1, ..., ∂J/∂T_(pieceN-1)

    
public:
    // planning
    std::vector<Sample> Planning(std::vector<Eigen::Vector2d> path, std::vector<SafeTravelCorridor2D::Polyhedron2D>& corridors){
        if (path.size() < 2 || corridors.empty()) {
            return {};
        }

        double pathLength = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            pathLength += (path[i] - path[i - 1]).norm();
        }
        const double targetPieces = std::clamp(static_cast<double>(corridors.size()) * 1.5, 8.0, 80.0);
        const double adaptiveLengthPerPiece = std::max(pathLength / targetPieces, 1.0);

 
        setup(1.0, path[0], path.back(), corridors, adaptiveLengthPerPiece, 0.5, 2, 1.0, 100.0, 100.0);
        minco.setConditions(headPVA, tailPVA, pieceN);
        Trajectory<5> traj;
        optimize(traj);
        std::vector<Sample> samples;

        // 获取轨迹总时长
        double total_duration = traj.getTotalDuration();

        // 控制采样规模，避免长轨迹在后处理阶段产生过高开销
        const double sample_interval = 0.05;
        const int num_samples_by_interval = std::max(2, static_cast<int>(total_duration / sample_interval) + 1);
        const int min_samples = std::max(2, pieceN * 3 + 1);
        const int max_samples = 400;
        const int num_samples = std::clamp(num_samples_by_interval, min_samples, max_samples);
        samples.reserve(num_samples);

        // 均匀采样轨迹点并同时获取速度
        for (int i = 0; i < num_samples; ++i) {
            double t = (i * total_duration) / (num_samples - 1);
            Eigen::Vector2d position = traj.getPos(t);
            Eigen::Vector2d velocity = traj.getVel(t);
            
            // 限制速度不超过最大速度
            if (velocity.norm() > 0.1) {
                velocity = velocity.normalized() * 0.1;
            }
            
            Sample s;
            s.pos = position;
            s.vel = velocity;
            s.t = t;
            samples.push_back(s);
        }

        return samples;

    }

    // magnitudeBounds = [v_max]^T
    // penaltyWeights = [pos_weight, vel_weight]^T
    inline bool setup(const double &timeWeight,
                      const Eigen::Vector2d &initialPoint,
                      const Eigen::Vector2d &terminalPoint,
                      const std::vector<SafeTravelCorridor2D::Polyhedron2D> &safeCorridor,
                      const double &lengthPerPiece,
                      const double &smoothingFactor,
                      const int &integralResolution,
                      const double &maxVelocity,
                      const double &positionWeight,
                      const double &velocityWeight)
    {
        rho = timeWeight;
        headPVA.setZero();
        tailPVA.setZero();
        headPVA.col(0) = initialPoint;
        tailPVA.col(0) = terminalPoint;

        safecorridors = safeCorridor;
        polyN = safeCorridor.size();
        
        if (polyN == 0) {
            std::cerr << "Error: No safe corridors provided!" << std::endl;
            return false;
        }
        
        // 转换H表示到V表示
        if (!processCorridor(safeCorridor, vPolytopes)) {
            std::cerr << "Error: Could not convert safe corridors to vertex representation!" << std::endl;
            return false;
        }
        
        smoothEps = smoothingFactor;
        integralRes = integralResolution;
        
        magnitudeBd.resize(1);
        magnitudeBd(0) = maxVelocity;
        penaltyWt.resize(2);
        penaltyWt(0) = positionWeight;
        penaltyWt(1) = velocityWeight;
        // Keep initial time allocation consistent with velocity limits.
        allocSpeed = std::max(3.0 * maxVelocity, 1.0e-3);

        // 创建初始路径
        getShortestPath(initialPoint, terminalPoint, vPolytopes, smoothEps, shortPath);
        
        // 计算每段的轨迹分段数
        Eigen::Matrix2Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
        pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
        pieceIdx.array() += 1;
        pieceN = pieceIdx.sum();

        temporalDim = pieceN;
        spatialDim = 0;
        vPolyIdx.resize(pieceN - 1);
        hPolyIdx.resize(pieceN);
        
        // 建立索引映射
        for (int i = 0, j = 0, k; i < polyN; i++)
        {
            k = pieceIdx(i);
            for (int l = 0; l < k; l++, j++)
            {
                if (l < k - 1)
                {
                    vPolyIdx(j) = 2 * i;
                    spatialDim += vPolytopes[2 * i].cols();
                }
                else if (i < polyN - 1)
                {
                    vPolyIdx(j) = 2 * i + 1;
                    spatialDim += vPolytopes[2 * i + 1].cols();
                }
                hPolyIdx(j) = i;
            }
        }

        // 分配临时变量
        points.resize(2, pieceN - 1);
        times.resize(pieceN);
        gradByPoints.resize(2, pieceN - 1);
        gradByTimes.resize(pieceN);
        partialGradByCoeffs.resize(6 * pieceN, 2);
        partialGradByTimes.resize(pieceN);

        return true;
    }

private:
    static inline double cross2d(const Eigen::Vector2d &a,
                                 const Eigen::Vector2d &b,
                                 const Eigen::Vector2d &c)
    {
        const Eigen::Vector2d ab = b - a;
        const Eigen::Vector2d ac = c - a;
        return ab.x() * ac.y() - ab.y() * ac.x();
    }

    static inline double polygonSignedArea(const std::vector<Eigen::Vector2d> &poly)
    {
        if (poly.size() < 3)
        {
            return 0.0;
        }

        double area2 = 0.0;
        for (size_t i = 0; i < poly.size(); ++i)
        {
            const Eigen::Vector2d &p = poly[i];
            const Eigen::Vector2d &q = poly[(i + 1) % poly.size()];
            area2 += p.x() * q.y() - q.x() * p.y();
        }
        return 0.5 * area2;
    }

    static inline bool lineIntersection(const Eigen::Vector2d &p1,
                                        const Eigen::Vector2d &p2,
                                        const Eigen::Vector2d &q1,
                                        const Eigen::Vector2d &q2,
                                        Eigen::Vector2d &inter)
    {
        const Eigen::Vector2d r = p2 - p1;
        const Eigen::Vector2d s = q2 - q1;
        const double denom = r.x() * s.y() - r.y() * s.x();
        if (std::abs(denom) < 1.0e-10)
        {
            return false;
        }

        const Eigen::Vector2d qp = q1 - p1;
        const double t = (qp.x() * s.y() - qp.y() * s.x()) / denom;
        inter = p1 + t * r;
        return true;
    }

    static inline bool insideClipEdge(const Eigen::Vector2d &p,
                                      const Eigen::Vector2d &a,
                                      const Eigen::Vector2d &b,
                                      const bool clipIsCCW,
                                      const double eps = 1.0e-9)
    {
        const double c = cross2d(a, b, p);
        return clipIsCCW ? (c >= -eps) : (c <= eps);
    }

    static inline void deduplicateVertices(std::vector<Eigen::Vector2d> &vertices,
                                           const double quantizeFactor = 1000.0)
    {
        std::map<std::pair<int, int>, bool> seen;
        std::vector<Eigen::Vector2d> uniqueVertices;
        uniqueVertices.reserve(vertices.size());

        for (const auto &vertex : vertices)
        {
            auto key = std::make_pair(
                static_cast<int>(std::round(vertex.x() * quantizeFactor)),
                static_cast<int>(std::round(vertex.y() * quantizeFactor)));
            if (seen.find(key) == seen.end())
            {
                seen[key] = true;
                uniqueVertices.push_back(vertex);
            }
        }

        vertices.swap(uniqueVertices);

        // 去掉首尾重复点，保持多边形简洁
        if (vertices.size() > 1 && (vertices.front() - vertices.back()).norm() < 1.0e-6)
        {
            vertices.pop_back();
        }
    }

    static inline void shrinkPolygonVertices(std::vector<Eigen::Vector2d> &vertices,
                                             const double minEdgeLen = 1.0e-3,
                                             const double collinearTol = 1.0e-3)
    {
        if (vertices.size() < 3)
        {
            return;
        }

        // 先过滤微小边，减少近重复点
        std::vector<Eigen::Vector2d> filtered;
        filtered.reserve(vertices.size());
        filtered.push_back(vertices.front());
        for (size_t i = 1; i < vertices.size(); ++i)
        {
            if ((vertices[i] - filtered.back()).norm() >= minEdgeLen)
            {
                filtered.push_back(vertices[i]);
            }
        }
        if (filtered.size() > 1 && (filtered.front() - filtered.back()).norm() < minEdgeLen)
        {
            filtered.pop_back();
        }

        if (filtered.size() < 3)
        {
            vertices.swap(filtered);
            return;
        }

        // 再过滤共线点，减少不必要顶点
        bool changed = true;
        while (changed && filtered.size() >= 3)
        {
            changed = false;
            std::vector<Eigen::Vector2d> compact;
            compact.reserve(filtered.size());

            for (size_t i = 0; i < filtered.size(); ++i)
            {
                const Eigen::Vector2d &prev = filtered[(i + filtered.size() - 1) % filtered.size()];
                const Eigen::Vector2d &curr = filtered[i];
                const Eigen::Vector2d &next = filtered[(i + 1) % filtered.size()];

                const Eigen::Vector2d v1 = curr - prev;
                const Eigen::Vector2d v2 = next - curr;
                const double n1 = v1.norm();
                const double n2 = v2.norm();
                if (n1 < minEdgeLen || n2 < minEdgeLen)
                {
                    changed = true;
                    continue;
                }

                const double cross = std::abs(v1.x() * v2.y() - v1.y() * v2.x());
                const double dot = v1.dot(v2);
                const bool nearlyCollinear = cross <= (collinearTol * n1 * n2);
                if (nearlyCollinear && dot >= 0.0)
                {
                    changed = true;
                    continue;
                }

                compact.push_back(curr);
            }

            if (compact.empty())
            {
                break;
            }
            filtered.swap(compact);
        }

        vertices.swap(filtered);
    }

    static inline void convexPolygonIntersection(const std::vector<Eigen::Vector2d> &subjectPoly,
                                                 const std::vector<Eigen::Vector2d> &clipPoly,
                                                 std::vector<Eigen::Vector2d> &out)
    {
        out.clear();
        if (subjectPoly.empty() || clipPoly.size() < 3)
        {
            return;
        }

        bool clipIsCCW = polygonSignedArea(clipPoly) >= 0.0;
        std::vector<Eigen::Vector2d> output = subjectPoly;

        for (size_t i = 0; i < clipPoly.size(); ++i)
        {
            const Eigen::Vector2d &A = clipPoly[i];
            const Eigen::Vector2d &B = clipPoly[(i + 1) % clipPoly.size()];
            if (output.empty())
            {
                break;
            }

            std::vector<Eigen::Vector2d> input = output;
            output.clear();
            Eigen::Vector2d S = input.back();

            for (const auto &E : input)
            {
                const bool E_inside = insideClipEdge(E, A, B, clipIsCCW);
                const bool S_inside = insideClipEdge(S, A, B, clipIsCCW);

                if (E_inside)
                {
                    if (!S_inside)
                    {
                        Eigen::Vector2d inter;
                        if (lineIntersection(S, E, A, B, inter))
                        {
                            output.push_back(inter);
                        }
                    }
                    output.push_back(E);
                }
                else if (S_inside)
                {
                    Eigen::Vector2d inter;
                    if (lineIntersection(S, E, A, B, inter))
                    {
                        output.push_back(inter);
                    }
                }

                S = E;
            }
        }

        out = output;
        deduplicateVertices(out);
        shrinkPolygonVertices(out);
    }

    static inline void forwardT(const Eigen::VectorXd &tau,
                                Eigen::VectorXd &T)
    {
        const int sizeTau = tau.size();
        T.resize(sizeTau);
        for (int i = 0; i < sizeTau; i++)
        {
            T(i) = tau(i) > 0.0
                       ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                       : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
        }
        return;
    }

    template <typename EIGENVEC>
    static inline void backwardT(const Eigen::VectorXd &T,
                                 EIGENVEC &tau)
    {
        const int sizeT = T.size();
        tau.resize(sizeT);
        for (int i = 0; i < sizeT; i++)
        {
            tau(i) = T(i) > 1.0
                         ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                         : (1.0 - sqrt(2.0 / T(i) - 1.0));
        }

        return;
    }

    template <typename EIGENVEC>
    static inline void backwardGradT(const Eigen::VectorXd &tau,
                                     const Eigen::VectorXd &gradT,
                                     EIGENVEC &gradTau)
    {
        const int sizeTau = tau.size();
        gradTau.resize(sizeTau);
        double denSqrt;
        for (int i = 0; i < sizeTau; i++)
        {
            if (tau(i) > 0)
            {
                gradTau(i) = gradT(i) * (tau(i) + 1.0);
            }
            else
            {
                denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
            }
        }

        return;
    }

    static inline void forwardP(const Eigen::VectorXd &xi,
                                const Eigen::VectorXi &vIdx,
                                const PolyhedraV &vPolys,
                                Eigen::Matrix2Xd &P)
    {
        const int sizeP = vIdx.size();
        P.resize(2, sizeP);
        Eigen::VectorXd q;
        for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
        {
            l = vIdx(i);
            k = vPolys[l].cols();
            q = xi.segment(j, k).normalized().head(k - 1);
            P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                       vPolys[l].col(0);
        }
        return;
    }

    static inline double costTinyNLS(void *ptr,
                                     const Eigen::VectorXd &xi,
                                     Eigen::VectorXd &gradXi)
    {
        const int n = xi.size();
        const Eigen::Matrix2Xd &ovPoly = *(Eigen::Matrix2Xd *)ptr;

        const double sqrNormXi = xi.squaredNorm();
        const double invNormXi = 1.0 / sqrt(sqrNormXi);
        const Eigen::VectorXd unitXi = xi * invNormXi;
        const Eigen::VectorXd r = unitXi.head(n - 1);
        const Eigen::Vector2d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                      ovPoly.col(1) - ovPoly.col(0);

        double cost = delta.squaredNorm();
        gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                             r.array() * 2.0;
        gradXi(n - 1) = 0.0;
        gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

        const double sqrNormViolation = sqrNormXi - 1.0;
        if (sqrNormViolation > 0.0)
        {
            double c = sqrNormViolation * sqrNormViolation;
            const double dc = 3.0 * c;
            c *= sqrNormViolation;
            cost += c;
            gradXi += dc * 2.0 * xi;
        }

        return cost;
    }

    template <typename EIGENVEC>
    static inline void backwardP(const Eigen::Matrix2Xd &P,
                                 const Eigen::VectorXi &vIdx,
                                 const PolyhedraV &vPolys,
                                 EIGENVEC &xi)
    {
        const int sizeP = P.cols();

        double minSqrD;
        lbfgs::lbfgs_parameter_t tiny_nls_params;
        tiny_nls_params.past = 0;
        tiny_nls_params.delta = 1.0e-5;
        tiny_nls_params.g_epsilon = FLT_EPSILON;
        tiny_nls_params.max_iterations = 128;

        Eigen::Matrix2Xd ovPoly;
        for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
        {
            l = vIdx(i);
            k = vPolys[l].cols();

            ovPoly.resize(2, k + 1);
            ovPoly.col(0) = P.col(i);
            ovPoly.rightCols(k) = vPolys[l];
            Eigen::VectorXd x(k);
            x.setConstant(sqrt(1.0 / k));
            lbfgs::lbfgs_optimize(x,
                                  minSqrD,
                                  &TrajOpt::costTinyNLS,
                                  nullptr,
                                  nullptr,
                                  &ovPoly,
                                  tiny_nls_params);

            xi.segment(j, k) = x;
        }

        return;
    }

    template <typename EIGENVEC>
    static inline void backwardGradP(const Eigen::VectorXd &xi,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     const Eigen::Matrix2Xd &gradP,
                                     EIGENVEC &gradXi)
    {
        const int sizeP = vIdx.size();
        gradXi.resize(xi.size());

        double normInv;
        Eigen::VectorXd q, gradQ, unitQ;
        for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
        {
            l = vIdx(i);
            k = vPolys[l].cols();
            q = xi.segment(j, k);
            normInv = 1.0 / q.norm();
            unitQ = q * normInv;
            gradQ.resize(k);
            gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                unitQ.head(k - 1).array() * 2.0;
            gradQ(k - 1) = 0.0;
            gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
        }

        return;
    }

    template <typename EIGENVEC>
    static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                           const Eigen::VectorXi &vIdx,
                                           const PolyhedraV &vPolys,
                                           double &cost,
                                           EIGENVEC &gradXi)
    {
        const int sizeP = vIdx.size();
        gradXi.resize(xi.size());

        double sqrNormQ, sqrNormViolation, c, dc;
        Eigen::VectorXd q;
        for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
        {
            k = vPolys[vIdx(i)].cols();

            q = xi.segment(j, k);
            sqrNormQ = q.squaredNorm();
            sqrNormViolation = sqrNormQ - 1.0;
            if (sqrNormViolation > 0.0)
            {
                c = sqrNormViolation * sqrNormViolation;
                dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradXi.segment(j, k) += dc * 2.0 * q;
            }
        }

        return;
    }

    static inline bool smoothedL1(const double &x,
                                  const double &mu,
                                  double &f,
                                  double &df)
    {
        if (x < 0.0)
        {
            return false;
        }
        else if (x > mu)
        {
            f = x - 0.5 * mu;
            df = 1.0;
            return true;
        }
        else
        {
            const double xdmu = x / mu;
            const double sqrxdmu = xdmu * xdmu;
            const double mumxd2 = mu - 0.5 * x;
            f = mumxd2 * sqrxdmu * xdmu;
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
            return true;
        }
    }

    static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                               const Eigen::MatrixX2d &coeffs,
                                               const Eigen::VectorXi &hIdx,
                                               const std::vector<SafeTravelCorridor2D::Polyhedron2D> &corridors,
                                               const double &smoothFactor,
                                               const int &integralResolution,
                                               const Eigen::VectorXd &magnitudeBounds,
                                               const Eigen::VectorXd &penaltyWeights,
                                               double &cost,
                                               Eigen::VectorXd &gradT,
                                               Eigen::MatrixX2d &gradC)
    {
        const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);  // 速度平方的最大值

        const double weightPos = penaltyWeights(0);
        const double weightVel = penaltyWeights(1);

        Eigen::Vector2d pos, vel;

        double s1, s2, s3, s4, s5;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
        int K;
        double violaPos, violaVel;
        double violaPosPenaD, violaVelPenaD;
        double violaPosPena, violaVelPena;
        double node, pena;

            const int pieceNum = T.size();
        const double integralFrac = 1.0 / integralResolution;
        const int samples = integralResolution + 1;

        // 复用局部变量减少频繁分配
        Eigen::Matrix<double, 6, 1> beta0_loc, beta1_loc, beta2_loc, beta3_loc, beta4_loc;
        Eigen::Matrix<double, 6, 2> c_block;
        Eigen::Matrix<double, 2, 6> cT;
        Eigen::Vector2d gradPos_loc, gradVel_loc;

        for (int i = 0; i < pieceNum; i++)
        {
            c_block = coeffs.block<6, 2>(i * 6, 0);
            cT = c_block.transpose(); // 2x6, reuse per sample

            double Ti = T(i);
            double T1 = Ti;
            double T2 = Ti * Ti;
            double T3 = T2 * Ti;
            double T4 = T3 * Ti;
            double T5 = T4 * Ti;

            for (int j = 0; j < samples; ++j)
            {
                // 直接计算 alpha 的幂，避免动态矩阵分配
                double alphaVal = j * integralFrac;
                double a1 = alphaVal;
                double a2 = a1 * alphaVal;
                double a3 = a2 * alphaVal;
                double a4 = a3 * alphaVal;
                double a5 = a4 * alphaVal;
                
                // s1..s5 = alpha^k * T^k
                s1 = a1 * T1;
                s2 = a2 * T2;
                s3 = a3 * T3;
                s4 = a4 * T4;
                s5 = a5 * T5;

                beta0_loc(0) = 1.0; beta0_loc(1) = s1; beta0_loc(2) = s2; beta0_loc(3) = s3; beta0_loc(4) = s4; beta0_loc(5) = s5;
                beta1_loc(0) = 0.0; beta1_loc(1) = 1.0; beta1_loc(2) = 2.0 * s1; beta1_loc(3) = 3.0 * s2; beta1_loc(4) = 4.0 * s3; beta1_loc(5) = 5.0 * s4;
                beta2_loc(0) = 0.0; beta2_loc(1) = 0.0; beta2_loc(2) = 2.0; beta2_loc(3) = 6.0 * s1; beta2_loc(4) = 12.0 * s2; beta2_loc(5) = 20.0 * s3;
                beta3_loc(0) = 0.0; beta3_loc(1) = 0.0; beta3_loc(2) = 0.0; beta3_loc(3) = 6.0; beta3_loc(4) = 24.0 * s1; beta3_loc(5) = 60.0 * s2;
                beta4_loc(0) = 0.0; beta4_loc(1) = 0.0; beta4_loc(2) = 0.0; beta4_loc(3) = 0.0; beta4_loc(4) = 24.0; beta4_loc(5) = 120.0 * s1;

                pos = cT * beta0_loc; // 2x1
                vel = cT * beta1_loc;

                // 检查速度是否违反约束
                violaVel = vel.squaredNorm() - velSqrMax;

                gradPos_loc.setZero();
                gradVel_loc.setZero();
                pena = 0.0;

                int L = hIdx(i);
                if(L < static_cast<int>(corridors.size())){
                    K = corridors[L].A.rows();
                    for (int k = 0; k < K; k++)
                    {
                        Eigen::Vector2d outerNormal = corridors[L].A.row(k);
                        violaPos = outerNormal.dot(pos) + corridors[L].b(k);
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            gradPos_loc += weightPos * violaPosPenaD * outerNormal;
                            pena += weightPos * violaPosPena;
                        }
                    }
                }

                if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                {
                    gradVel_loc += weightVel * violaVelPenaD * 2.0 * vel;
                    pena += weightVel * violaVelPena;
                }

                node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                gradC.block<6, 2>(i * 6, 0) += (beta0_loc * gradPos_loc.transpose() +
                                                beta1_loc * gradVel_loc.transpose()) *
                                                node * (Ti * integralFrac);
                gradT(i) += (gradPos_loc.dot(vel) +
                             gradVel_loc.dot(cT * beta2_loc)) *
                             alphaVal * node * (Ti * integralFrac) +
                         node * integralFrac * pena;
                cost += node * (Ti * integralFrac) * pena;
            }
        }

        return;
    }

    static inline double costFunctional(void *ptr,
                                        const Eigen::VectorXd &x,
                                        Eigen::VectorXd &g)
    {
        TrajOpt &obj = *(TrajOpt *)ptr;
        const int dimTau = obj.temporalDim;
        const int dimXi = obj.spatialDim;
        const double weightT = obj.rho;
        Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
        Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
        Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
        Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

        obj.forwardT(tau, obj.times);
        obj.forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);

        double cost = 0.0;
        obj.minco.setParameters(obj.points, obj.times);
        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

        attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                obj.hPolyIdx, obj.safecorridors,
                                obj.smoothEps, obj.integralRes,
                                obj.magnitudeBd, obj.penaltyWt,
                                cost, obj.partialGradByTimes, obj.partialGradByCoeffs);

        obj.minco.propagateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                obj.gradByPoints, obj.gradByTimes);

        cost += weightT * obj.times.sum();
        obj.gradByTimes.array() += weightT;

        obj.backwardGradT(tau, obj.gradByTimes, gradTau);
        obj.backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);
        obj.normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

        // 仅在调试构建中清理异常梯度，避免发布构建的额外遍历开销
#ifndef NDEBUG
        gradTau = gradTau.unaryExpr([](double val) {
            return std::isnan(val) || std::isinf(val) ? 0.0 : val;
        });

        gradXi = gradXi.unaryExpr([](double val) {
            return std::isnan(val) || std::isinf(val) ? 0.0 : val;
        });
#endif

        return cost;
    }

    static inline double costDistance(void *ptr,
                                      const Eigen::VectorXd &xi,
                                      Eigen::VectorXd &gradXi)
    {
        void **dataPtrs = (void **)ptr;
        const double &dEps = *((const double *)(dataPtrs[0]));
        const Eigen::Vector2d &ini = *((const Eigen::Vector2d *)(dataPtrs[1]));
        const Eigen::Vector2d &fin = *((const Eigen::Vector2d *)(dataPtrs[2]));
        const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));

        double cost = 0.0;
        const int overlaps = vPolys.size() / 2;

        Eigen::Matrix2Xd gradP = Eigen::Matrix2Xd::Zero(2, overlaps);
        Eigen::Vector2d a, b, d;
        Eigen::VectorXd r;
        double smoothedDistance;
        for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
        {
            a = i == 0 ? ini : b;
            if (i < overlaps)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);
                b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                    vPolys[2 * i + 1].col(0);
            }
            else
            {
                b = fin;
            }

            d = b - a;
            smoothedDistance = sqrt(d.squaredNorm() + dEps);
            cost += smoothedDistance;

            if (i < overlaps)
            {
                gradP.col(i) += d / smoothedDistance;
            }
            if (i > 0)
            {
                gradP.col(i - 1) -= d / smoothedDistance;
            }
        }

        Eigen::VectorXd unitQ;
        double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
        for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
        {
            k = vPolys[2 * i + 1].cols();
            Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
            Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
            sqrNormQ = q.squaredNorm();
            invNormQ = 1.0 / sqrt(sqrNormQ);
            unitQ = q * invNormQ;
            gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                unitQ.head(k - 1).array() * 2.0;
            gradQ(k - 1) = 0.0;
            gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

            sqrNormViolation = sqrNormQ - 1.0;
            if (sqrNormViolation > 0.0)
            {
                c = sqrNormViolation * sqrNormViolation;
                dc = 3.0 * c;
                c *= sqrNormViolation;
                cost += c;
                gradQ += dc * 2.0 * q;
            }
        }

        return cost;
    }

    static inline void getShortestPath(const Eigen::Vector2d &ini,
                                       const Eigen::Vector2d &fin,
                                       const PolyhedraV &vPolys,
                                       const double &smoothD,
                                       Eigen::Matrix2Xd &path)
    {
        const int overlaps = vPolys.size() / 2;
        if (overlaps == 0) {
            path.resize(2, 2);
            path.col(0) = ini;
            path.col(1) = fin;
            return;
        }
        
        Eigen::VectorXi vSizes(overlaps);
        for (int i = 0; i < overlaps; i++)
        {
            vSizes(i) = vPolys[2 * i + 1].cols();
        }
        Eigen::VectorXd xi(vSizes.sum());
        for (int i = 0, j = 0; i < overlaps; i++)
        {
            xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
            j += vSizes(i);
        }

        double minDistance;
        void *dataPtrs[4];
        dataPtrs[0] = (void *)(&smoothD);
        dataPtrs[1] = (void *)(&ini);
        dataPtrs[2] = (void *)(&fin);
        dataPtrs[3] = (void *)(&vPolys);
        lbfgs::lbfgs_parameter_t shortest_path_params;
        shortest_path_params.past = 3;
        shortest_path_params.delta = 1.0e-3;
        shortest_path_params.g_epsilon = 1.0e-5;

        lbfgs::lbfgs_optimize(xi,
                              minDistance,
                              &TrajOpt::costDistance,
                              nullptr,
                              nullptr,
                              dataPtrs,
                              shortest_path_params);

        path.resize(2, overlaps + 2);
        path.leftCols<1>() = ini;
        path.rightCols<1>() = fin;
        Eigen::VectorXd r;
        for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
        {
            k = vPolys[2 * i + 1].cols();
            Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
            r = q.normalized().head(k - 1);
            path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                              vPolys[2 * i + 1].col(0);
        }

        return;
    }

    static inline bool processCorridor(const std::vector<SafeTravelCorridor2D::Polyhedron2D> &hPs,
                                       PolyhedraV &vPs)
    {
        const int sizeCorridor = hPs.size();

        if (sizeCorridor == 0) {
            return false;
        }

        vPs.clear();
        vPs.reserve(2 * sizeCorridor - 1);  // N个走廊产生N个自身区域和N-1个交集区域

        // 处理每个走廊的独立区域
        for (int i = 0; i < sizeCorridor; i++)
        {
            // 使用当前走廊的顶点
            PolyhedronV curIV(2, hPs[i].vertices.size());
            for (size_t j = 0; j < hPs[i].vertices.size(); j++) {
                curIV.col(j) = hPs[i].vertices[j];
            }
            
            // 转换为偏移表示形式：第一个点作为基点，其余点作为相对于基点的偏移
            PolyhedronV curIOB(2, hPs[i].vertices.size());
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(hPs[i].vertices.size() - 1) = 
                curIV.rightCols(hPs[i].vertices.size() - 1).colwise() - curIV.col(0);
            
            vPs.push_back(curIOB);
            
            // 如果不是最后一个走廊，添加与下一个走廊的交集区域
            if (i < sizeCorridor - 1) {
                std::vector<Eigen::Vector2d> intersection_vertices;
                convexPolygonIntersection(hPs[i].vertices, hPs[i + 1].vertices, intersection_vertices);

                // 退化情况：裁剪失败时合并并去重
                if (intersection_vertices.empty()) {
                    intersection_vertices.insert(intersection_vertices.end(),
                                                 hPs[i].vertices.begin(),
                                                 hPs[i].vertices.end());
                    intersection_vertices.insert(intersection_vertices.end(),
                                                 hPs[i + 1].vertices.begin(),
                                                 hPs[i + 1].vertices.end());
                    deduplicateVertices(intersection_vertices);
                    shrinkPolygonVertices(intersection_vertices);
                }

                if (intersection_vertices.size() < 3) {
                    continue;
                }
                
                // 创建交集区域的偏移表示
                PolyhedronV interIV(2, intersection_vertices.size());
                for (size_t j = 0; j < intersection_vertices.size(); j++) {
                    interIV.col(j) = intersection_vertices[j];
                }
                
                PolyhedronV interOB(2, intersection_vertices.size());
                interOB.col(0) = interIV.col(0);
                if (intersection_vertices.size() > 1) {
                    interOB.rightCols(intersection_vertices.size() - 1) = 
                        interIV.rightCols(intersection_vertices.size() - 1).colwise() - interIV.col(0);
                }
                
                vPs.push_back(interOB);
            }
        }

        return true;
    }

    // 获取轨迹的主函数
    inline double optimize(Trajectory<5> &traj)
    {
        Eigen::VectorXd x(temporalDim + spatialDim);
        Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
        Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

        setInitial(shortPath, allocSpeed, pieceIdx, points, times);
        backwardT(times, tau);
        backwardP(points, vPolyIdx, vPolytopes, xi);

        double minCostFunctional;
        lbfgs_params.mem_size = 256;      // 减少L-BFGS的记忆大小，避免内存相关问题
        lbfgs_params.past = 3;            // 收敛检测窗口大小，检查过去past次迭代中目标函数值的相对变化
        lbfgs_params.min_step = 1.0e-13;  // 线搜索的最小步长，设置合理值避免过小
        lbfgs_params.g_epsilon = 1e-4;    // 设置适当的梯度收敛容差，避免0值
        lbfgs_params.delta = 1e-3;        // 相对成本容差（从外部传入），|f_old - f_new| / max(1, |f_new|) < delta
        lbfgs_params.max_iterations = 1000;

        int ret = lbfgs::lbfgs_optimize(x,
                                        minCostFunctional,
                                        &TrajOpt::costFunctional,
                                        nullptr,
                                        nullptr,
                                        this,
                                        lbfgs_params);

        if (ret >= 0)
        {
            forwardT(tau, times);
            forwardP(xi, vPolyIdx, vPolytopes, points);
            minco.setParameters(points, times);
            minco.getTrajectory(traj);
        }
        else
        {
            traj.clear();
            minCostFunctional = INFINITY;
            std::cout << "Optimization Failed: "
                      << lbfgs::lbfgs_strerror(ret)
                      << std::endl;
        }

        return minCostFunctional;
    }

    static inline void setInitial(const Eigen::Matrix2Xd &path,
                                  const double &speed,
                                  const Eigen::VectorXi &intervalNs,
                                  Eigen::Matrix2Xd &innerPoints,
                                  Eigen::VectorXd &timeAlloc)
    {
        const int sizeM = intervalNs.size();   // 区间数量
        if (sizeM <= 0 || path.cols() < 2) {
            // 如果没有区间或路径点不足，直接返回
            return;
        }
        
        const int sizeN = intervalNs.sum();    // 总点数
        innerPoints.resize(2, sizeN - 1);      // 初始化内部点矩阵
        timeAlloc.resize(sizeN);               // 初始化时间分配向量

        Eigen::Vector2d a, b, c;               // 临时变量：起点、终点、方向向量
        for (int i = 0, j = 0, k = 0, l; i < sizeM; i++) // 遍历每个区间
        {
            l = intervalNs(i);                  // 当前区间的点数
            
            // 确保不会越界访问path的列
            if (i >= path.cols() - 1) {
                break;  // 如果索引超出范围则退出循环
            }
            
            a = path.col(i);                    // 当前区间的起始点
            b = path.col(i + 1);                // 当前区间的结束点
            c = (b - a) / l;                    // 当前区间的单位方向向量
            timeAlloc.segment(j, l).setConstant(c.norm() / speed); // 计算并设置当前区间的时间分配
            j += l;
            for (int m = 0; m < l; m++)         // 生成当前区间的内部插值点
            {
                if (i > 0 || m > 0)             // 跳过第一个点，避免重复
                {
                    innerPoints.col(k++) = a + c * m;// 线性插值生成内部点
                }
            }
        }
    }
    
};

#endif