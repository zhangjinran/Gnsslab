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
#include <gnsslab/GnssFunc.h>
#include <gnsslab/TimeStruct.h>
#include <gnsslab/TimeConvert.h>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/NavEphGPS.hpp>
#include <gnsslab/StringUtils.h>
#include <gnsslab/SP3Store.hpp>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/GnssFunc.h>
#include <filesystem>

int main(int argc,char* argv[]) {
    namespace fs = std::filesystem;

    const fs::path dataDir = fs::current_path() / "data";
    const fs::path outputDir = fs::current_path() / "outputs" / "gps_eph";
    fs::create_directories(outputDir);



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

    std::string navfile =
        (dataDir / "BRDC00IGS_R_20250010000_01D_MN.rnx").string();
    RinexNavStore navStore;
    navStore.loadFile(navfile);
    
    // 调试：打印 ionoCorrData 的内容
    std::cout << "\n=== Ionosphere Correction Data ===" << std::endl;
    for (const auto& ionoEntry : navStore.ionoCorrData) {
        std::cout << "Type: " << ionoEntry.first << " -> ";
        for (size_t i = 0; i < ionoEntry.second.size(); i++) {
            std::cout << ionoEntry.second[i];
            if (i < ionoEntry.second.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
    }
    
    // 调试：打印 BDS专用电离层参数（按SatID）
    std::cout << "\n=== BDS Ionosphere Correction Data (by SatID) ===" << std::endl;
    for (const auto& entry : navStore.ionoCorrDataBDS) {
        const SatID& sat = entry.first;
        const auto& param = entry.second;
        std::cout << "SatID: " << sat.toString() << std::endl;
        if (param.hasAlpha) {
            std::cout << "  Alpha: [" << param.alpha[0] << ", " << param.alpha[1] 
                      << ", " << param.alpha[2] << ", " << param.alpha[3] << "]" << std::endl;
        }
        if (param.hasBeta) {
            std::cout << "  Beta:  [" << param.beta[0] << ", " << param.beta[1] 
                      << ", " << param.beta[2] << ", " << param.beta[3] << "]" << std::endl;
        }
    }

    string sp3File =
        (dataDir / "COD0MGXFIN_20250010000_01D_05M_ORB.SP3").string();
    SP3Store sp3Store;
    sp3Store.loadSP3File(sp3File);


    navStore.getContrastData(sp3Store,predictedTime,stoptime,30);

    navStore.writeFile((outputDir / "output.txt").string(), "GPS");
    navStore.writeFile((outputDir / "output2.txt").string(), "BDS");

    
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
