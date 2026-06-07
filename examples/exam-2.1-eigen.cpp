/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *
 * Author: shoujian zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * Description:
 *  Eigen 库入门演示程序
 *  展示矩阵与向量的定义、基本运算和常用方法
 */

#include <iostream>
#include <Eigen/Dense>    // Eigen核心头文件

// 调试模式开关
#define DEBUG_MODE 1

// 函数声明
void printMatrixInfo(const Eigen::MatrixXd& mat, const std::string& name);
void printVectorInfo(const Eigen::VectorXd& vec, const std::string& name);

int main() {
    std::cout << "==========================================" << std::endl;
    std::cout << "         Eigen 库矩阵与向量操作演示" << std::endl;
    std::cout << "==========================================" << std::endl;

    // --------------------------
    // 1. 矩阵定义与初始化
    // --------------------------
    std::cout << "\n【1. 矩阵定义与初始化】" << std::endl;
    
    // 方法1：动态矩阵，指定大小
    Eigen::MatrixXd A(2, 2);
    A << 3, -1,
         2.5, 1.5;
    printMatrixInfo(A, "矩阵 A");

    // 方法2：静态矩阵（编译时确定大小）
    Eigen::Matrix3d B;
    B << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    printMatrixInfo(B, "矩阵 B (3x3)");

    // 方法3：特殊矩阵
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3, 3);  // 单位矩阵
    printMatrixInfo(C, "单位矩阵 C");

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2, 3);      // 零矩阵
    printMatrixInfo(D, "零矩阵 D");

    Eigen::MatrixXd E = Eigen::MatrixXd::Random(2, 2);    // 随机矩阵
    printMatrixInfo(E, "随机矩阵 E");

    // --------------------------
    // 2. 向量定义与初始化
    // --------------------------
    std::cout << "\n【2. 向量定义与初始化】" << std::endl;
    
    // 列向量
    Eigen::VectorXd v(3);
    v << 1, 2, 3;
    printVectorInfo(v, "列向量 v");

    // 行向量
    Eigen::RowVectorXd rv(3);
    rv << 4, 5, 6;
    std::cout << "行向量 rv: " << rv << std::endl;

    // 从矩阵提取列
    Eigen::VectorXd col1 = B.col(0);
    printVectorInfo(col1, "B 的第1列");

    // --------------------------
    // 3. 矩阵基本运算
    // --------------------------
    std::cout << "\n【3. 矩阵基本运算】" << std::endl;
    
    // 矩阵加法
    Eigen::MatrixXd A_add = A + A;
    printMatrixInfo(A_add, "A + A");

    // 矩阵减法
    Eigen::MatrixXd A_sub = A - A;
    printMatrixInfo(A_sub, "A - A");

    // 矩阵乘法
    Eigen::MatrixXd A_mul = A * A;
    printMatrixInfo(A_mul, "A * A");

    // 矩阵数乘
    Eigen::MatrixXd A_scalar = 2 * A;
    printMatrixInfo(A_scalar, "2 * A");

    // --------------------------
    // 4. 矩阵常用方法
    // --------------------------
    std::cout << "\n【4. 矩阵常用方法】" << std::endl;
    
    // 转置
    Eigen::MatrixXd A_trans = A.transpose();
    printMatrixInfo(A_trans, "A 的转置");

    // 行列式
    double detA = A.determinant();
    std::cout << "A 的行列式: " << detA << std::endl;

    // 求逆（仅方阵且可逆）
    if (detA != 0) {
        Eigen::MatrixXd A_inv = A.inverse();
        printMatrixInfo(A_inv, "A 的逆矩阵");
        
        // 验证: A * A_inv = I
        Eigen::MatrixXd A_Ainv = A * A_inv;
        printMatrixInfo(A_Ainv, "A * A_inv (应接近单位矩阵)");
    } else {
        std::cout << "警告: A 是奇异矩阵，无法求逆" << std::endl;
    }

    // 迹（对角线元素之和）
    double traceA = A.trace();
    std::cout << "A 的迹: " << traceA << std::endl;

    // --------------------------
    // 5. 向量运算
    // --------------------------
    std::cout << "\n【5. 向量运算】" << std::endl;
    
    // 向量加法
    Eigen::VectorXd v_add = v + v;
    printVectorInfo(v_add, "v + v");

    // 向量点积
    double dot_prod = v.dot(v);
    std::cout << "v 与 v 的点积: " << dot_prod << std::endl;

    // 向量范数
    double norm_v = v.norm();
    std::cout << "v 的范数: " << norm_v << std::endl;

    // 标准化向量
    Eigen::VectorXd v_normalized = v.normalized();
    printVectorInfo(v_normalized, "v 的单位向量");

    // --------------------------
    // 6. 求解线性方程组
    // --------------------------
    std::cout << "\n【6. 求解线性方程组 Ax = b】" << std::endl;
    
    Eigen::VectorXd b(2);
    b << 5, 7;
    printVectorInfo(b, "向量 b");

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
    printVectorInfo(x, "方程组的解 x");

    // 验证
    Eigen::VectorXd Ax = A * x;
    printVectorInfo(Ax, "验证: A*x (应等于 b)");

    // --------------------------
    // 7. 矩阵块操作
    // --------------------------
    std::cout << "\n【7. 矩阵块操作】" << std::endl;
    
    // 提取子矩阵
    Eigen::MatrixXd B_sub = B.block(0, 0, 2, 2);  // 从(0,0)开始，取2x2子矩阵
    printMatrixInfo(B_sub, "B 的左上角 2x2 子矩阵");

    // 修改子矩阵
    Eigen::MatrixXd F(2, 2);
    F << 10, 20,
         30, 40;
    B.block(0, 0, 2, 2) = F;
    printMatrixInfo(B, "修改后的矩阵 B");

    std::cout << "\n==========================================" << std::endl;
    std::cout << "              演示结束" << std::endl;
    std::cout << "==========================================" << std::endl;

    return 0;
}

// 打印矩阵信息
void printMatrixInfo(const Eigen::MatrixXd& mat, const std::string& name) {
    std::cout << name << " (" << mat.rows() << "x" << mat.cols() << "):" << std::endl;
    std::cout << mat << std::endl;
    
    #if DEBUG_MODE
    std::cout << "  - 元素类型: double" << std::endl;
    std::cout << "  - 存储顺序: " << (mat.IsRowMajor ? "行优先" : "列优先") << std::endl;
    #endif
}

// 打印向量信息
void printVectorInfo(const Eigen::VectorXd& vec, const std::string& name) {
    std::cout << name << " (维度: " << vec.size() << "):" << std::endl;
    std::cout << vec << std::endl;
}