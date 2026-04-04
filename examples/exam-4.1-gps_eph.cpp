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
#include "GnssFunc.h"
#include "TimeStruct.h"
#include "TimeConvert.h"
#include "GnssStruct.h"
#include "NavEphGPS.hpp"
#include "StringUtils.h"
#include "SP3Store.hpp"
#include "RinexNavStore.hpp"
#include "GnssFunc.h"

int main(int argc,char* argv[]) {


    CivilTime civilTimePrediced;
    civilTimePrediced = CivilTime(2025, 1, 1, 0, 0.0, 0.0);

    CommonTime predictedTime;
    predictedTime = CivilTime2CommonTime(civilTimePrediced);

    YDSTime ydsPrediced;
    ydsPrediced = CommonTime2YDSTime(predictedTime);

    CommonTime stoptime=predictedTime+86400;
    MJD mjdPrediced;
    CommonTime2MJD(stoptime,mjdPrediced);
    //cout << "epoch:" << ydsPrediced << endl;

    std::string navfile="/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/BRDC00IGS_R_20250010000_01D_MN.rnx";
    RinexNavStore navStore;
    navStore.loadFile(navfile);

    string sp3File = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/COD0MGXFIN_20250010000_01D_05M_ORB.SP3";
    SP3Store sp3Store;
    sp3Store.loadSP3File(sp3File);


    navStore.getContrastData(sp3Store,predictedTime,stoptime,30);

    navStore.writeFile("/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/output.txt","GPS");
    navStore.writeFile("/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/output2.txt","BDS");

    
    // for (auto it:navStore.gpsEphData) {
    //     while (predictedTime<stoptime) {
    //         Xvt xvtNav=navStore.getXvt(it.first,predictedTime);
    //
    //         Xvt xvtSP3 = sp3Store.getXvt(it.first, predictedTime);
    //         cout << "sp3:" << xvtSP3 << endl;
    //         Vector3d xSP3 = xvtSP3.getPos();
    //
    //         Vector3d diffXYZ = xvtNav.getPos() - xSP3;
    //         Vector3d diffVel = xvtNav.getVel() - xvtSP3.getVel();
    //         double diffClockBias = xvtNav.getClockBias() - xvtSP3.getClockBias();
    //         double diffRelCorr = xvtNav.getRelativityCorr() - xvtSP3.getRelativityCorr();
    //
    //         cout << ydsPrediced << " \n"
    //              << " sat:" << it.first << " \n"
    //              << " nav:\n" << xvtNav << " \n"
    //              << " sp3:\n" << xvtSP3 << " \n"
    //              << " diffXYZ:\n" << diffXYZ << " \n"
    //              << " diffVel:\n" << diffVel << " \n"
    //              << " diffClockBias:\n" << diffClockBias << " \n"
    //              << " diffRelCorr:\n" << diffRelCorr << " \n"
    //              << endl;
    //         predictedTime+=30;
    //     }
    // }





}