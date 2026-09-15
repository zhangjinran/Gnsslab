//
// Created by zhang on 2026/5/2.
//

#ifndef BRDC00IGS_R_20250010000_01D_MN_RNX_WRITE_H
#define BRDC00IGS_R_20250010000_01D_MN_RNX_WRITE_H

#endif //BRDC00IGS_R_20250010000_01D_MN_RNX_WRITE_H
#include <string>
#include <algorithm> //replace 函数
#include <gnsslab/TimeConvert.h>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/GnssFunc.h>
#include <gnsslab/ARLambda.hpp>
#include <gnsslab/CoordConvert.h>
#include <gnsslab/RinexNavStore.hpp>
#include<gnsslab/CoordConvert.h>
#include<gnsslab/CoordStruct.h>

typedef std::map<CommonTime,std::map<SatID,double>> satValueEpochMap;
void writeDelay(satValueEpochMap delayMap,string filename);