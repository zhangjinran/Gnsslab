/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *  As stipulated by the MulanPSL-2.0, you are granted the following freedoms:
 *      To copy, use, and modify the software;
 *      To use the software for commercial purposes;
 *      To redistribute the software.
 *
 * Author: Shoujian Zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * References:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
 * 2. Eckel, Bruce. Thinking in C++. 2nd ed., Prentice Hall, 2000.
 */

#ifndef SolverLSQ_HPP
#define SolverLSQ_HPP

#include <Eigen/Eigen>
#include "GnssStruct.h"

using namespace Eigen;

/*
 * 这个类通过定义通用方程系统，可以被用单点定位，rtk，精密单点定位或者其他最小二乘任务
 */
class SolverLSQ {
public:

    SolverLSQ() {};
    Vector3d getxyz() const;
    virtual void solve(EquSys &equSys);
    virtual void solveGeneral(EquSys &equSys);
    int getIndex(const VariableSet &varSet, const Variable &thisVar);
    double getSolution(const Parameter &type,
                       VariableSet &currentUnkSet,
                       const VectorXd &stateVec);
    Eigen::VectorXd getResiduals() const;

    Eigen::VectorXd getState() const { return state; }

    void setxyz(const Vector3d &xyz) ;
    MatrixXd getcov_r();
    double getBias() const;
    double getSigma0() const;
    MatrixXd getw() const;

    MatrixXd getCovMatrix() const;
    Eigen::Vector3d getdxyz() const{
        return dxyz;
    };

    /// Destructor.
    virtual ~SolverLSQ() {};

private:
    double bias;
    MatrixXd W;
    VectorXd state;
    MatrixXd covMatrix;
    MatrixXd cov_r;
    Vector3d dxyz;
    Vector3d xyz;
    VariableSet currentUnkSet;
    VectorXd residuals;
    double sigma0;
}; // End of class 'SolverLSQ'


#endif   // SolverLSQ_HPP
