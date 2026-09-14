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

#include <iomanip>
#include "SolverLSQ.h"
#include <fstream>

#define debug 0
using namespace std;
Eigen::VectorXd SolverLSQ::getResiduals() const
{
    return residuals;
}

double SolverLSQ::getSigma0() const
{
    return sigma0;
}

void SolverLSQ::solve(EquSys &equSys) {

    if(debug) {
        cout << "\n" << string(60, '=') << endl;
        cout << "=== SolverLSQ::solve() - 最小二乘求解 ===" << endl;
        cout << string(60, '-') << endl;
    }

    currentUnkSet = equSys.varSet;
    int numUnk = currentUnkSet.size();
    int numObs = equSys.obsEquData.size();

    if(debug) {
        cout << "观测方程数 (numObs): " << numObs << endl;
        cout << "未知参数数 (numUnk): " << numUnk << endl;
        cout << "自由度 (redundancy): " << numObs - numUnk << endl;
        cout << string(60, '-') << endl;
    }

    VectorXd prefit = VectorXd::Zero(numObs);
    MatrixXd hMatrix = MatrixXd::Zero(numObs, numUnk);
    MatrixXd wMatrix = MatrixXd::Zero(numObs, numObs);

    int iobs(0);
    for (auto ed: equSys.obsEquData) {
        prefit(iobs) = ed.second.prefit;

        for (auto vc: ed.second.varCoeffData) {
            int indexUnk = getIndex(currentUnkSet, vc.first);
            hMatrix(iobs, indexUnk) = vc.second;
        }
        wMatrix(iobs, iobs) = ed.second.weight;
        iobs++;
    }

    MatrixXd hT = hMatrix.transpose();

    if (prefit.size()!= hMatrix.rows()) {
        InvalidSolver e("prefit size don't equal with rows of hMatrix");
        throw(e);
    }

    if (debug) {
        cout << "\n【先验残差向量 (prefit)】" << endl;
        cout << "维度: " << prefit.rows() << " x " << prefit.cols() << endl;
        cout << prefit.transpose() << endl;

        cout << "\n【设计矩阵 (hMatrix)】" << endl;
        cout << "维度: " << hMatrix.rows() << " x " << hMatrix.cols() << endl;
        cout << hMatrix << endl;

        cout << "\n【权重矩阵 (wMatrix)】" << endl;
        cout << "维度: " << wMatrix.rows() << " x " << wMatrix.cols() << endl;
        cout << "对角元素(权重值): ";
        for (int i = 0; i < min(10, (int)wMatrix.rows()); ++i) {
            cout << wMatrix(i, i) << " ";
        }
        if (wMatrix.rows() > 10) cout << "...";
        cout << endl;
    }

    try {
        covMatrix = hT * wMatrix * hMatrix;
        covMatrix = covMatrix.inverse();
    }
    catch (...) {
        InvalidSolver e("Unable to invert matrix covMatrix");
        throw (e);
    }


    state = covMatrix * hT * wMatrix * prefit;

    // 后验残差
    residuals = prefit - hMatrix * state;

    // 单位权中误差
    int redundancy = numObs - numUnk;

    if(redundancy > 0)
    {
        sigma0 = sqrt(
            (residuals.transpose()
            * wMatrix
            * residuals)(0,0)
            / redundancy
        );
    }
    else
    {
        sigma0 = 0.0;
    }
    MatrixXd I = MatrixXd::Identity(numObs, numObs);
    cov_r = sigma0 * sigma0 * (I - hMatrix * covMatrix * hMatrix.transpose() * wMatrix)*wMatrix.inverse();
    W=wMatrix;
    if(debug) {
        cout << "\n【求解结果】" << endl;
        cout << string(40, '-') << endl;
        
        cout << "\n1. 状态向量 (state)" << endl;
        cout << "维度: " << state.rows() << " x " << state.cols() << endl;
        cout << "解向量: " << state.transpose() << endl;
        
        cout << "\n2. 单位权中误差 (sigma0)" << endl;
        cout << fixed << setprecision(6) << "sigma0 = " << sigma0 << " m" << endl;
        
        cout << "\n3. 后验残差 (residuals)" << endl;
        cout << "维度: " << residuals.rows() << " x " << residuals.cols() << endl;
        cout << "残差向量: " << residuals.transpose() << endl;
        
        // 计算残差统计信息
        double maxResidual = residuals.cwiseAbs().maxCoeff();
        double minResidual = residuals.cwiseAbs().minCoeff();
        double meanResidual = residuals.mean();
        double rmsResidual = sqrt(residuals.squaredNorm() / residuals.size());
        
        cout << "\n4. 残差统计" << endl;
        cout << fixed << setprecision(6);
        cout << "   最大值: " << maxResidual << " m" << endl;
        cout << "   最小值: " << minResidual << " m" << endl;
        cout << "   平均值: " << meanResidual << " m" << endl;
        cout << "   RMS:    " << rmsResidual << " m" << endl;
        
        cout << "\n" << string(60, '=') << endl;
    }

    double dx = getSolution(Parameter::dX, currentUnkSet, state);
    double dy = getSolution(Parameter::dY, currentUnkSet, state);
    double dz = getSolution(Parameter::dZ, currentUnkSet, state);

    dxyz[0] = dx;
    dxyz[1] = dy;
    dxyz[2] = dz;

    try {
        bias=getSolution(Parameter::bias, currentUnkSet, state);
    }
    catch (...) {}

}

void SolverLSQ::solveGeneral(EquSys &equSys) {
    // 与 solve() 相同，但不提取 dX/dY/dZ（适用于速度等非常规参数）
    currentUnkSet = equSys.varSet;
    int numUnk = currentUnkSet.size();
    int numObs = equSys.obsEquData.size();

    VectorXd prefit = VectorXd::Zero(numObs);
    MatrixXd hMatrix = MatrixXd::Zero(numObs, numUnk);
    MatrixXd wMatrix = MatrixXd::Zero(numObs, numObs);

    int iobs(0);
    for (auto ed: equSys.obsEquData) {
        prefit(iobs) = ed.second.prefit;
        for (auto vc: ed.second.varCoeffData) {
            int indexUnk = getIndex(currentUnkSet, vc.first);
            hMatrix(iobs, indexUnk) = vc.second;
        }
        wMatrix(iobs, iobs) = ed.second.weight;
        iobs++;
    }

    MatrixXd hT = hMatrix.transpose();
    covMatrix = (hT * wMatrix * hMatrix).inverse();
    state = covMatrix * hT * wMatrix * prefit;

    VectorXd v = hMatrix * state - prefit;
    residuals = v;

    if (numObs > numUnk) {
        sigma0 = sqrt(
            (v.transpose() * wMatrix * v)(0,0) / (numObs - numUnk)
        );
    } else {
        sigma0 = 0.0;
    }

    MatrixXd I = MatrixXd::Identity(numObs, numObs);
    MatrixXd hInv = covMatrix * hT * wMatrix;
    cov_r = sigma0 * sigma0 * (I - hMatrix * hInv) * wMatrix.inverse();
    W = wMatrix;
}

int SolverLSQ::getIndex(const VariableSet &varSet, const Variable &thisVar) {
    int index(0);
    for (auto var: varSet) {
        if (var == thisVar) {
            break;
        }
        index++;
    }
    return index;
};

double SolverLSQ::getSolution(const Parameter &type,
                              VariableSet &currentUnkSet,
                              const VectorXd &stateVec)
noexcept(false) {
    // Declare an varIterator for 'stateMap' and go to the first element
    auto varIt = currentUnkSet.begin();
    int index(0);
    while ((*varIt).getParaType() != type) {
        // If the same type is not found, throw an exception
        if (varIt == currentUnkSet.end()) {
            InvalidRequest e("SolverLSQ::Type not found in state vector.");
            throw (e);
        }
        index++;
        varIt++;
    }

    // Else, return the corresponding value
    return stateVec(index);

}  // End of method 'SolverGeneral::getSolution()'   


Vector3d SolverLSQ::getxyz() const {
    return xyz;
}
void SolverLSQ::setxyz(const Vector3d &xyz) {
    this->xyz = xyz;
}


MatrixXd SolverLSQ::getCovMatrix() const {
    return covMatrix;
}
MatrixXd SolverLSQ::getcov_r() {
    return cov_r;
}
MatrixXd SolverLSQ::getw() const {
    return W;
}
double SolverLSQ::getBias() const {
    return bias;
}