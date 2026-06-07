//
// Created by zhang on 2026/5/2.
//

#ifndef BRDC00IGS_R_20250010000_01D_MN_RNX_WRITE_H
#define BRDC00IGS_R_20250010000_01D_MN_RNX_WRITE_H

#endif //BRDC00IGS_R_20250010000_01D_MN_RNX_WRITE_H
#include <string>
#include <algorithm> //replace 函数
#include "TimeConvert.h"
#include "GnssStruct.h"
#include "GnssFunc.h"
#include "ARLambda.hpp"
#include "CoordConvert.h"
#include "RinexNavStore.hpp"
#include"CoordConvert.h"
#include"CoordStruct.h"

typedef std::map<CommonTime,std::map<SatID,double>> satValueEpochMap;
void writeDelay(satValueEpochMap delayMap,string filename);