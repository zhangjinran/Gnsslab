//
// Created by zhang on 2026/5/8.
//

#ifndef BRDC00IGS_R_20250010000_01D_MN_RNX_GNSSFUNC_H
#define BRDC00IGS_R_20250010000_01D_MN_RNX_GNSSFUNC_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include<Eigen/Dense>
#include<boost/math/distributions/chi_squared.hpp>
#include"SolverLSQ.h"


#include "GnssStruct.h"
struct SatData
{
    std::string sat;

    double pseudorange;

    double x;
    double y;
    double z;

    double elevation;
};
struct EpochData
{
    int gpsWeek;

    Vector3d result;

    double gpsSec;

    double Ta=0;

    int satNum;

    double T0=0;

    double thrd;


    std::vector<SatData> sats;
};
std::vector<EpochData> readData(const std::string& filename);
SolverLSQ sovleLsQ(EpochData& epoch_data,SatID sat=SatID());
EquSys linearize(Vector3d xyz,EpochData& epoch_data,SatID sat_id=SatID());
bool inspect(SolverLSQ solver_lsq,double df,EpochData& epoch_data,double alpha=0.05);
bool inspect_bias(SolverLSQ solver_lsq,EpochData& epoch_data,double alpha=0.05);
double chi2_critical_value(int df, double alpha);
bool detect_model(double T0, int df, EpochData& epoch_data,double alpha = 0.05);
void writeData(const vector<EpochData>epoch_datas, map<double,SolverLSQ>& solver_lsqs,string name="");

#endif //BRDC00IGS_R_20250010000_01D_MN_RNX_GNSSFUNC_H
