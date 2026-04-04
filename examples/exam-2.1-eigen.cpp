//
// Created by shjzh on 2024/12/24.
//

/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *  As stipulated by the MulanPSL-2.0, you are granted the following freedoms:
 *      To copy, use, and modify the software;
 *      To use the software for commercial purposes;
 *      To redistribute the software.
 *
 * Author: shoujian zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * References:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
 * 2. Eckel, Bruce. Thinking in C++. 2nd ed., Prentice Hall, 2000.
 */


#include <iostream>
#include <Eigen/Dense>    // Eigen头文件，<Eigen/Dense>包含Eigen库里面所有的函数和类

Eigen::MatrixXd operator+(const Eigen::MatrixXd& m1, const Eigen::MatrixXd& m2)
{
    if (m1.rows()!=m2.rows()||m1.cols()!=m2.cols())
        {
            std::cerr << "Matrix must be equal" << std::endl;
        return Eigen::MatrixXd();
        }
    Eigen::MatrixXd m(m1.rows(), m1.cols());
    for (int i=0; i<m1.rows(); i++) {
        for (int j=0; j<m1.cols(); j++) {
            m(i,j) = m1(i,j) + m2(i,j);
        }
    }
    return m;
}
Eigen::MatrixXd operator-(const Eigen::MatrixXd& m1, const Eigen::MatrixXd& m2) {
    if (m1.rows()!=m2.rows()||m1.cols()!=m2.cols())
    {
        std::cerr << "Matrix must be equal" << std::endl;
        return Eigen::MatrixXd();
    }
    Eigen::MatrixXd m(m1.rows(), m1.cols());
    for (int i=0; i<m1.rows(); i++) {
        for (int j=0; j<m1.cols(); j++) {
            m(i,j) = m1(i,j) - m2(i,j);
        }
    }
    return m;
}

Eigen::MatrixXd operator*(const Eigen::MatrixXd& m1, const Eigen::MatrixXd& m2) {
    if (m1.cols()!=m2.rows()) {
        std::cerr << "The first matrix's cols must be equal to the second matrix's rows" << std::endl;
        return Eigen::MatrixXd();
    }
    Eigen::MatrixXd m(m1.rows(), m2.cols());
    m.setZero();
    for (int i=0; i<m1.rows(); i++) {
        for (int j=0; j<m2.cols(); j++) {
            for (int k=0; k<m1.cols(); k++) {
                m(i,j) += m1(i,k) * m2(k,j);
                //std::cout<<m(i,j)<<std::endl;
            }

        }
    }
    return m;

}
int main() {
    Eigen::MatrixXd m(2, 2);   // MatrixXd 表示的是动态数组，初始化的时候指定数组的行数和列数
    m(0, 0) = 3;               //m(i,j) 表示第i行第j列的值，这里对数组进行初始化
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    std::cout << m << std::endl;     // eigen重载了<<运算符，可以直接输出eigen矩阵的值

    Eigen::MatrixXd m_test_add(2,2);
    m_test_add=m+m;
    std::cout << "m_test_add=" <<std::endl<<m_test_add<< std::endl;

    Eigen::MatrixXd m_test_ch(2,2);
    m_test_ch=m*m;
    std::cout << "m_test_ch=" <<std::endl<<m_test_ch<< std::endl;

    Eigen::MatrixXd m_test_jian(2,2);
    m_test_jian=m-m;
    std::cout << "m_test_jian=" <<std::endl<<m_test_jian<< std::endl;

    Eigen::MatrixXd m_test_in(2,2);
    m_test_in=m.inverse();
    std::cout << "m_test_in=" <<std::endl<<m_test_in<< std::endl;
}
