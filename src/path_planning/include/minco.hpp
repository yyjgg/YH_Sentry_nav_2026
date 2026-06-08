/*
    MIT 许可证

    版权所有 (c) 2021 王志培 (wangzhepei@live.com)

    特此免费授予任何获得本软件及相关文档文件（“软件”）副本的人不受限制地处理
    本软件的权限，包括但不限于使用、复制、修改、合并、发布、分发、再许可和/或销售
    本软件副本的权利，并允许接受本软件的人这样做，但须满足以下条件：

    上述版权声明和本许可声明应包含在所有的
    副本或主要部分中。

    本软件按“原样”提供，不附带任何明示或暗示的担保，包括但不限于对适销性、
    特定用途适用性和非侵权性的担保。在任何情况下，作者或版权持有人均不对
    因本软件产生或与之相关的任何索赔、损害赔偿或其他责任承担责任，
    无论是在合同诉讼、侵权行为或其他诉讼中，由本软件引起或与之相关。
*/

#ifndef MINCO_HPP
#define MINCO_HPP

#include "trajectory.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <vector>

#include <iostream>



namespace minco
{
    // Ax = b 与 M(T)c = b(q) 的映射关系，
    // A ------------- M(T)    (由时间分配T决定的带状系数矩阵)
    // x ------------- c       (多项式轨迹的系数向量)  
    // b ------------- b(q)    (由节点位置q决定的边界条件向量)

    // 带状系统类用于高效求解
    // 带状线性系统 Ax=b。
    // A 是一个 N*N 的带状矩阵，下带宽为 lowerBw
    // 上带宽为 upperBw。
    // 带状 LU 分解具有 O(N) 时间复杂度。
    class BandedSystem
    {
    public:
        // 需要 A 的尺寸 N，以及下/上带宽 p/q
        inline void create(const int &n, const int &p, const int &q)
        {
            // 防止在销毁前重新创建
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            return;
        }

        inline void destroy()
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
                ptrData = nullptr;
            }
            return;
        }

    private:
        int N;            // 路径段数
        int lowerBw;      // 下带
        int upperBw;      // 上帯
        // 此处必须初始化为 nullptr
        double *ptrData = nullptr;

    public:
        // 将矩阵重置为零
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // 带状矩阵的存储方式如《矩阵计算》中所建议
        // 对应带状矩阵压缩存储公式：A(i,j) = ptrData[(i-j+upperBw)*N+j]
        inline const double &operator()(const int &i, const int &j) const
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        // 此函数就地执行带状 LU 分解
        // 注意：为了效率，矩阵 "A" 上未应用主元选择！！！
        inline void factorizeLU()
        {
            int iM, jM;
            double cVl;
            // 对应高斯消元法的带状优化版本
            for (int k = 0; k <= N - 2; k++)
            {
                // 仅处理带宽范围内的元素，复杂度O(N)
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    cVl = operator()(k, j);
                    if (cVl != 0.0)
                    {
                        for (int i = k + 1; i <= iM; i++)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
            return;
        }

        // 此函数求解 Ax=b，然后将解 x 存储在 b 中
        // 输入 b 要求为 N*m 维，即
        // 需要求解 m 个向量。
        template <typename EIGENMAT>
        inline void solve(EIGENMAT &b) const
        {
            int iM;
            // 求解 M(T)c = b(q) 系统
            // 前向替代：求解 Ly = b(q)
            for (int j = 0; j <= N - 1; j++)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            // 后向替代：求解 Uc = y  
            for (int j = N - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // 此函数求解 A^Tx=b，然后将解 x 存储在 b 中，用于梯度传播。传入为位置约束b（q），传出为c，为高效使用指针
        // -------------- 求解 M(T)ᵀλ = ∂J/∂c，用于梯度传播 ------------------------------
        // 输入 b 要求为 N*m 维，即需要求解 m 个向量。
        template <typename EIGENMAT>
        inline void solveAdj(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            return;
        }
    };

    // 用于 s=2 且时间非均匀的 MINCO
    // 对应论文第3.5节：多阶段无约束最小化控制量轨迹
    template<int dim>
    class MINCO_S3NU
    {
    public:
        MINCO_S3NU() = default;
        ~MINCO_S3NU() { A.destroy(); }

    private:
        int N; // 段数 - 对应论文中的轨迹段数M
        Eigen::Matrix<double, dim, 3> headPVA; // 起点位置、速度和加速度 - 对应边界条件¯z_o
        Eigen::Matrix<double, dim, 3> tailPVA; // 终点位置、速度和加速度 - 对应边界条件¯z_f
        BandedSystem A; // 带状系统 - 对应线性方程组系数矩阵M
        Eigen::MatrixX2d b; // 系数矩阵 - 对应多项式系数c
        Eigen::VectorXd T1; // 时间向量 T - 对应Δt_i = t_i - t_{i-1}
        Eigen::VectorXd T2; // 时间平方向量 T^2 - 用于多项式计算
        Eigen::VectorXd T3; // 时间立方向量 T^3 - 用于多项式计算
        Eigen::VectorXd T4; // 时间四次方向量 T^4 - 用于多项式计算
        Eigen::VectorXd T5; // 时间五次方向量 T^5 - 用于多项式计算

    public:
        // headState: [[x y],[vx,vy]] - 初始状态向量
        // 对应论文公式(3-18c)的边界条件
        inline void setConditions(const Eigen::Matrix<double, dim, 3> &headState,
                                  const Eigen::Matrix<double, dim, 3> &tailState,
                                  const int &pieceNum)
        {
            N = pieceNum;
            headPVA = headState;  // ¯z_o = [p₀, v₀, a₀]ᵀ
            tailPVA = tailState;  // ¯z_f = [p_f, v_f, a_f]ᵀ
            A.create(6 * N, 6, 6);
            b.resize(6 * N, 2);
            T1.resize(N);
            T2.resize(N);
            T3.resize(N);
            T4.resize(N);
            T5.resize(N);// 时间相关向量
            return;
        }

        inline void setParameters(const Eigen::Matrix2Xd &inPs,  // 路径点
                                  const Eigen::VectorXd &ts)     // 时间点
        {
            // 总共有 N 段轨迹，对应 N-1 个中间航点
            // 第 i 段轨迹：p_i(t) = c₀ⁱ + c₁ⁱt + c₂ⁱt² + c₃ⁱt³, 其中 t ∈ [0, T_i]
            // ts 是局部时间：每段轨迹都有自己的时间坐标系，从0到T_i
            T1 = ts; // 时间向量T = [T₁, T₂, ..., T_M]ᵀ
            T2 = T1.cwiseProduct(T1); // T² = [T₁², T₂², ..., T_M²]ᵀ  
            T3 = T2.cwiseProduct(T1); // T³ = [T₁³, T₂³, ..., T_M³]ᵀ
            T4 = T2.cwiseProduct(T2); // T⁴ = [T₁⁴, T₂⁴, ..., T_M⁴]ᵀ
            T5 = T4.cwiseProduct(T1); // T⁵ = [T₁⁵, T₂⁵, ..., T_M⁵]ᵀ

            A.reset(); // 重置带状矩阵
            b.setZero(); // 清零右端向量

            // 设置起点边界条件 - 对应论文公式(3-25)
            A(0, 0) = 1.0; // 位置边界：p₁(t) = c₀¹ + c₁¹t + c₂¹t² + c₃¹t³，p₁(0) = c₀¹，A(0, 0)对应c₀¹的系数
            A(1, 1) = 1.0; // 速度边界：v₁(t) = c₁¹ + 2c₂¹t + 3c₃¹t²，v₁(0) = c₁¹，A(1, 1)对应c₁¹的系数
            A(2, 2) = 2.0; // 加速度边界：a₁(t) = 2c₂¹ + 6c₃¹t，a₁(0) = 2c₂¹，A(2, 2)对应c₂¹的系数
            b.row(0) = headPVA.col(0).transpose(); // 起点位置 p₀
            b.row(1) = headPVA.col(1).transpose(); // 起点速度 v₀
            b.row(2) = headPVA.col(2).transpose(); // 起点加速度 a₀
            
            // 设置连续性条件和路径点约束 - 对应定理2的最优性条件
            for (int i = 0; i < N - 1; i++)
            {
                // 加加速度连续性约束：j_i(T_i) = j_{i+1}(0)
                // 第 i 段轨迹的加加速度函数（jerk）：j_i(t) = 6c₃ⁱ + 24c₄ⁱt + 60c₅ⁱt²
                // 在段末时刻 t = T_i 的加加速度：j_i(T_i) = 6c₃ⁱ + 24c₄ⁱT_i + 60c₅ⁱT_i²
                // 第 i+1 段轨迹在段首的加加速度：j_{i+1}(0) = 6c₃ⁱ⁺¹
                // 加加速度连续性要求：j_i(T_i) = j_{i+1}(0)
                // ⇒ 6c₃ⁱ + 24c₄ⁱT_i + 60c₅ⁱT_i² - 6c₃ⁱ⁺¹ = 0
                A(6 * i + 3, 6 * i + 3) = 6.0;              // 6c₃ⁱ
                A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);     // 24c₄ⁱT_i
                A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);     // 60c₅ⁱT_i²
                A(6 * i + 3, 6 * i + 9) = -6.0;             // -6c₃ⁱ⁺¹

                // 加加加速度连续性约束：s_i(T_i) = s_{i+1}(0)
                // 第 i 段轨迹的加加加速度函数（snap）：s_i(t) = 24c₄ⁱ + 120c₅ⁱt
                // 在段末时刻 t = T_i 的加加加速度：s_i(T_i) = 24c₄ⁱ + 120c₅ⁱT_i
                // 第 i+1 段轨迹在段首的加加加速度：s_{i+1}(0) = 24c₄ⁱ⁺¹
                // ⇒ 24c₄ⁱ + 120c₅ⁱT_i - 24c₄ⁱ⁺¹ = 0
                A(6 * i + 4, 6 * i + 4) = 24.0;             // 24c₄ⁱ
                A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);    // 120c₅ⁱT_i
                A(6 * i + 4, 6 * i + 10) = -24.0;           // -24c₄ⁱ⁺¹

                // 末端位置约束：p_i(T_i) = q_i
                // 第 i 段轨迹的位置函数：p_i(t) = c₀ⁱ + c₁ⁱt + c₂ⁱt² + c₃ⁱt³ + c₄ⁱt⁴ + c₅ⁱt⁵
                // 在段末时刻 t = T_i 的位置：p_i(T_i) = c₀ⁱ + c₁ⁱT_i + c₂ⁱT_i² + c₃ⁱT_i³ + c₄ⁱT_i⁴ + c₅ⁱT_i⁵
                // ⇒ c₀ⁱ + c₁ⁱT_i + c₂ⁱT_i² + c₃ⁱT_i³ + c₄ⁱT_i⁴ + c₅ⁱT_i⁵  = q_i
                A(6 * i + 5, 6 * i) = 1.0;                  // c₀ⁱ
                A(6 * i + 5, 6 * i + 1) = T1(i);            // c₁ⁱT_i
                A(6 * i + 5, 6 * i + 2) = T2(i);            // c₂ⁱT_i²
                A(6 * i + 5, 6 * i + 3) = T3(i);            // c₃ⁱT_i³
                A(6 * i + 5, 6 * i + 4) = T4(i);            // c₄ⁱT_i⁴
                A(6 * i + 5, 6 * i + 5) = T5(i);            // c₅ⁱT_i⁵

                // 位置连续性约束：p_i(T_i) = p_{i+1}(0)
                // 第 i 段轨迹的位置函数：p_i(t) = c₀ⁱ + c₁ⁱt + c₂ⁱt² + c₃ⁱt³ + c₄ⁱt⁴ + c₅ⁱt⁵
                // 在段末时刻 t = T_i 的位置：p_i(T_i) = c₀ⁱ + c₁ⁱT_i + c₂ⁱT_i² + c₃ⁱT_i³ + c₄ⁱT_i⁴ + c₅ⁱT_i⁵
                // 第 i+1 段轨迹在段首的位置：p_{i+1}(0) = c₀ⁱ⁺¹
                // ⇒ c₀ⁱ + c₁ⁱT_i + c₂ⁱT_i² + c₃ⁱT_i³ + c₄ⁱT_i⁴ + c₅ⁱT_i⁵ - c₀ⁱ⁺¹ = 0
                
                A(6 * i + 6, 6 * i) = 1.0;                  // c₀ⁱ
                A(6 * i + 6, 6 * i + 1) = T1(i);            // c₁ⁱT_i
                A(6 * i + 6, 6 * i + 2) = T2(i);            // c₂ⁱT_i²
                A(6 * i + 6, 6 * i + 3) = T3(i);            // c₃ⁱT_i³
                A(6 * i + 6, 6 * i + 4) = T4(i);            // c₄ⁱT_i⁴
                A(6 * i + 6, 6 * i + 5) = T5(i);            // c₅ⁱT_i⁵
                A(6 * i + 6, 6 * i + 6) = -1.0;             // -c₀ⁱ⁺¹

                // 速度连续性约束：v_i(T_i) = v_{i+1}(0)
                // 第 i 段轨迹的速度函数：v_i(t) = c₁ⁱ + 2c₂ⁱt + 3c₃ⁱt² + 4c₄ⁱt³ + 5c₅ⁱt⁴
                // 在段末时刻 t = T_i 的速度：v_i(T_i) = c₁ⁱ + 2c₂ⁱT_i + 3c₃ⁱT_i² + 4c₄ⁱT_i³ + 5c₅ⁱT_i⁴
                // 第 i+1 段轨迹在段首的速度：v_{i+1}(0) = c₁ⁱ⁺¹
                // ⇒ c₁ⁱ + 2c₂ⁱT_i + 3c₃ⁱT_i² + 4c₄ⁱT_i³ + 5c₅ⁱT_i⁴ - c₁ⁱ⁺¹ = 0
                A(6 * i + 7, 6 * i + 1) = 1.0;              // c₁ⁱ
                A(6 * i + 7, 6 * i + 2) = 2 * T1(i);        // 2c₂ⁱT_i
                A(6 * i + 7, 6 * i + 3) = 3 * T2(i);        // 3c₃ⁱT_i²
                A(6 * i + 7, 6 * i + 4) = 4 * T3(i);        // 4c₄ⁱT_i³
                A(6 * i + 7, 6 * i + 5) = 5 * T4(i);        // 5c₅ⁱT_i⁴
                A(6 * i + 7, 6 * i + 7) = -1.0;             // -c₁ⁱ⁺¹

                // 加速度连续性约束：a_i(T_i) = a_{i+1}(0)
                // 第 i 段轨迹的加速度函数：a_i(t) = 2c₂ⁱ + 6c₃ⁱt + 12c₄ⁱt² + 20c₅ⁱt³
                // 在段末时刻 t = T_i 的加速度：a_i(T_i) = 2c₂ⁱ + 6c₃ⁱT_i + 12c₄ⁱT_i² + 20c₅ⁱT_i³
                // 第 i+1 段轨迹在段首的加速度：a_{i+1}(0) = 2c₂ⁱ⁺¹
                // ⇒ 2c₂ⁱ + 6c₃ⁱT_i + 12c₄ⁱT_i² + 20c₅ⁱT_i³ - 2c₂ⁱ⁺¹ = 0
                A(6 * i + 8, 6 * i + 2) = 2.0;              // 2c₂ⁱ
                A(6 * i + 8, 6 * i + 3) = 6 * T1(i);        // 6c₃ⁱT_i
                A(6 * i + 8, 6 * i + 4) = 12 * T2(i);       // 12c₄ⁱT_i²
                A(6 * i + 8, 6 * i + 5) = 20 * T3(i);       // 20c₅ⁱT_i³
                A(6 * i + 8, 6 * i + 8) = -2.0;             // -2c₂ⁱ⁺¹

                // 将路径点位置 q_i 设置到右端向量 b 中
                b.row(6 * i + 5) = inPs.col(i).transpose(); // 路径点位置 q_i
            }
            

            // 设置终点条件 - 对应论文公式(3-25)
            // 位置边界：p_N(T_N) = c₀ᴺ + c₁ᴺT_N + c₂ᴺT_N² + c₃ᴺT_N³p_N(T_N) = c₀ᴺ + c₁ᴺT_N + c₂ᴺT_N² + c₃ᴺT_N³ + c₄ᴺT_N⁴ + c₅ᴺT_N⁵
            A(6 * N - 3, 6 * N - 6) = 1.0;                  // c₀ᴺ
            A(6 * N - 3, 6 * N - 5) = T1(N - 1);            // c₁ᴺT_N
            A(6 * N - 3, 6 * N - 4) = T2(N - 1);            // c₂ᴺT_N²
            A(6 * N - 3, 6 * N - 3) = T3(N - 1);            // c₃ᴺT_N³
            A(6 * N - 3, 6 * N - 2) = T4(N - 1);            // c₄ᴺT_N⁴
            A(6 * N - 3, 6 * N - 1) = T5(N - 1);            // c₅ᴺT_N⁵
            // 速度边界：v_N(T_N) = c₁ᴺ + 2c₂ᴺT_N + 3c₃ᴺT_N² + 4c₄ᴺT_N³ + 5c₅ᴺT_N⁴
            A(6 * N - 2, 6 * N - 5) = 1.0;                  // c₁ᴺ
            A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);        // 2c₂ᴺT_N
            A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);        // 3c₃ᴺT_N²
            A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);        // 4c₄ᴺT_N³
            A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);        // 5c₅ᴺT_N⁴
            // 加速度边界：a_N(T_N) = 2c₂ᴺ + 6c₃ᴺT_N + 12c₄ᴺT_N² + 20c₅ᴺT_N³
            A(6 * N - 1, 6 * N - 4) = 2;                    // 2c₂ᴺ
            A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);        // 6c₃ᴺT_N
            A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);       // 12c₄ᴺT_N²
            A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);       // 20c₅ᴺT_N³

            b.row(6 * N - 3) = tailPVA.col(0).transpose(); // 终点位置 p_f
            b.row(6 * N - 2) = tailPVA.col(1).transpose(); // 终点速度 v_f
            b.row(6 * N - 1) = tailPVA.col(2).transpose(); // 终点加速度 a_f

            // 求解线性系统 M(T)c = b(q) - 对应论文第3.6节
            A.factorizeLU(); // LU分解，时间复杂度 O(N)
            A.solve(b);      // 求解得到多项式系数c

            return;
        }

        inline void getTrajectory(Trajectory<5> &traj) const
        {
            traj.clear();
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i),
                                  b.block<6, 2>(6 * i, 0)
                                      .transpose()
                                      .rowwise()
                                      .reverse());
            }
            return;
        }

        inline void getEnergy(double &energy) const
        {
            // 计算最小控制量代价：J = ∫‖ȧ(t)‖²dt - 对应论文公式(3-18a)
            // 对于第i段轨迹，加加速度函数为：j_i(t) = 6c₃ⁱ + 24c₄ⁱt + 60c₅ⁱt²
            // 能量函数为加加速度平方的积分：J_i = ∫₀^{T_i} ‖j_i(t)‖² dt = ∫₀^{T_i} (6c₃ⁱ + 24c₄ⁱt + 60c₅ⁱt²)² dt
            // = 36‖c₃ⁱ‖²T_i + 144(c₃ⁱ·c₄ⁱ)T_i² + 192‖c₄ⁱ‖²T_i³ + 240(c₄ⁱ·c₅ⁱ)T_i⁴ + 720(c₅ⁱ·c₃ⁱ)T_i⁵ + 720‖c₅ⁱ‖²T_i⁶
            energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                energy += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                          144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                          192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                          240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                          720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                          720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
            }
            return;
        }

        inline const Eigen::MatrixX2d &getCoeffs(void) const
        {
            return b;
        }

        // 能量函数对系数的偏导 - 对应论文第3.8节
        inline void getEnergyPartialGradByCoeffs(Eigen::MatrixX2d &gdC) const
        {
            // 能量函数关于系数的梯度：
            // ∂J/∂c₅ⁱ = 240·c₃·T³ + 720·c₄·T⁴ + 1440·c₅·T⁵ 
            // ∂J/∂c₄ⁱ = 144·c₃·T² + 384·c₄·T³ + 720·c₅·T⁴
            // ∂J/∂c₃ⁱ = 72·c₃·T + 144·c₄·T² + 240·c₅·T³
            gdC.resize(6 * N, 2);
            for (int i = 0; i < N; i++)
            {
                gdC.row(6 * i + 5) = 240.0 * b.row(6 * i + 3) * T3(i) +
                                     720.0 * b.row(6 * i + 4) * T4(i) +
                                     1440.0 * b.row(6 * i + 5) * T5(i);
                gdC.row(6 * i + 4) = 144.0 * b.row(6 * i + 3) * T2(i) +
                                     384.0 * b.row(6 * i + 4) * T3(i) +
                                     720.0 * b.row(6 * i + 5) * T4(i);
                gdC.row(6 * i + 3) = 72.0 * b.row(6 * i + 3) * T1(i) +
                                     144.0 * b.row(6 * i + 4) * T2(i) +
                                     240.0 * b.row(6 * i + 5) * T3(i);
                gdC.block<3, 2>(6 * i, 0).setZero();
            }
            return;
        }
           
        // 能量函数对时间的偏导
        inline void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
        {
            // 能量函数关于时间的梯度：∂J/∂T_i   
            // = 36·‖c₃ⁱ‖² + 288·(c₃ⁱ·c₄ⁱ)·T_i + 576·‖c₄ⁱ‖²·T_i² + 720·(c₅ⁱ·c₃ⁱ)·T_i² + 2880·(c₅ⁱ·c₄ⁱ)·T_i³ + 3600·‖c₅ⁱ‖²·T_i⁴
            gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                gdT(i) = 36.0 * b.row(6 * i + 3).squaredNorm() +
                         288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                         576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                         720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                         2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                         3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
            }
            return;
        }
      
        // 线性约束系统：M(T) · c = b(q) 代价函数：J(c, T)
        // 代价函数全微dJ = ⟨∂J/∂c, dc⟩ + ⟨∂J/∂T, dT⟩= (∂J/∂c)ᵀ·dc + (∂J/∂T)ᵀ·dT
        // 约束条件微分 对 M(T)·c = b(q) 进行微分：d(M·c) = db M·dc + (∂M/∂T)·c·dT = (∂b/∂q)·dq
        // 引入拉格朗日乘子 λ，构造增广函数：L(c, T, λ) = J(c, T) + λᵀ·[M(T)·c - b(q)]
        // 最优性条件：∂L/∂c = ∂J/∂c + Mᵀ·λ = 0 ⇒ Mᵀ·λ = -∂J/∂c
        // 通过伴随方程求解 λ：λ = (Mᵀ)⁻¹ · (-∂J/∂c)
        // 然后计算其他梯度：∂J/∂q = (∂b/∂q)ᵀ · λ ∂J/∂T = ∂J/∂T + λᵀ·(∂M/∂T)·c
        // 梯度传播：通过伴随方法计算梯度 - 对应论文第3.8节

        // M ∈ R^(m×n)     # 约束矩阵
        // m：约束条件的总个数包括所有连续性约束、边界条件约束和中间点约束对于s阶系统，m = 2Ms（M为轨迹段数）物理意义：整个轨迹需要满足的所有线性约束方程的数量
        // n：决策变量的总维度对于s阶多项式轨迹，n = 2Ms（每个维度有2s个系数）

        // c ∈ R^n         # 决策变量
        // n：轨迹系数的总个数对于三维轨迹，n = 3 × 2Ms
        
        // q ∈ R^p         # 路径点参数
        // p：路径点的总自由度对于三维轨迹，p = 3 × (M-1)（M-1个中间路径点）

        // T ∈ R           # 时间参数
        // λ ∈ R^m         # 拉格朗日乘子
        // m：与约束条件个数相同

        // ∂J/∂c ∈ R^n     # 代价函数对c的梯度
        // n：与决策变量维度相同
        // ∂J/∂T ∈ R       # 代价函数对T的梯度
        // λ 表示约束 violation 对代价函数的边际影响
        // 每个 λ_i 对应第 i 个约束的"影子价格"
        // ∂J/∂q: 路径点调整对代价的敏感度
        // ∂J/∂T: 时间分配调整对代价的敏感度
        // ∂J/∂c: 轨迹形状调整对代价的敏感度
        
        // 1. 计算直接梯度: ∂J/∂c, ∂J/∂T
        // 2. 求解伴随方程: Mᵀλ = -∂J/∂c
        // 3. 计算间接梯度: ∂J/∂q = (∂b/∂q)ᵀλ
        // 4. 合成总梯度: ∂J/∂T_total = ∂J/∂T + λᵀ(∂M/∂T)c
        inline void propagateGrad(const Eigen::MatrixX2d &partialGradByCoeffs,
                                const Eigen::VectorXd &partialGradByTimes,
                                Eigen::Matrix2Xd &gradByPoints,
                                Eigen::VectorXd &gradByTimes)
        {
            gradByPoints.resize(2, N - 1);
            gradByTimes.resize(N);
            
            // 对于线性系统 M(T)c=b(q)，其伴随系统定义为：M(T)Tλ=∂c/∂J​其中 λ是伴随变量，J是代价函数。 - 对应公式(3-85)
            Eigen::MatrixX2d adjGrad = partialGradByCoeffs;
            A.solveAdj(adjGrad); // 带状系统求解伴随变量

            // 路径点梯度计算pi​(Ti​)=qi - 对应公式(3-89)
            for (int i = 0; i < N - 1; i++)
            {
                gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
            }

            // 时间梯度计算 - 对应公式(3-93)
            // 公式：∂J/∂T_i = ∂J/∂T_i_direct - Tr(G_i^T ∂E_i/∂T_i c_i)
            Eigen::Matrix<double, 6, 2> B1;
            Eigen::Matrix<double, 3, 2> B2;
            for (int i = 0; i < N - 1; i++)
            {   
                // negative velocity
                B1.row(2) = -(b.row(i * 6 + 1) +
                              2.0 * T1(i) * b.row(i * 6 + 2) +
                              3.0 * T2(i) * b.row(i * 6 + 3) +
                              4.0 * T3(i) * b.row(i * 6 + 4) +
                              5.0 * T4(i) * b.row(i * 6 + 5));
                B1.row(3) = B1.row(2);

                // negative acceleration
                B1.row(4) = -(2.0 * b.row(i * 6 + 2) +
                              6.0 * T1(i) * b.row(i * 6 + 3) +
                              12.0 * T2(i) * b.row(i * 6 + 4) +
                              20.0 * T3(i) * b.row(i * 6 + 5));

                // negative jerk
                B1.row(5) = -(6.0 * b.row(i * 6 + 3) +
                              24.0 * T1(i) * b.row(i * 6 + 4) +
                              60.0 * T2(i) * b.row(i * 6 + 5));

                // negative snap
                B1.row(0) = -(24.0 * b.row(i * 6 + 4) +
                              120.0 * T1(i) * b.row(i * 6 + 5));

                // negative crackle
                B1.row(1) = -120.0 * b.row(i * 6 + 5);

                gradByTimes(i) = B1.cwiseProduct(adjGrad.block<6, 2>(6 * i + 3, 0)).sum();
            }

            // negative velocity
            B2.row(0) = -(b.row(6 * N - 5) +
                          2.0 * T1(N - 1) * b.row(6 * N - 4) +
                          3.0 * T2(N - 1) * b.row(6 * N - 3) +
                          4.0 * T3(N - 1) * b.row(6 * N - 2) +
                          5.0 * T4(N - 1) * b.row(6 * N - 1));

            // negative acceleration
            B2.row(1) = -(2.0 * b.row(6 * N - 4) +
                          6.0 * T1(N - 1) * b.row(6 * N - 3) +
                          12.0 * T2(N - 1) * b.row(6 * N - 2) +
                          20.0 * T3(N - 1) * b.row(6 * N - 1));

            // negative jerk
            B2.row(2) = -(6.0 * b.row(6 * N - 3) +
                          24.0 * T1(N - 1) * b.row(6 * N - 2) +
                          60.0 * T2(N - 1) * b.row(6 * N - 1));

            gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<3, 2>(6 * N - 3, 0)).sum();

            gradByTimes += partialGradByTimes;
        }

    };

}

#endif