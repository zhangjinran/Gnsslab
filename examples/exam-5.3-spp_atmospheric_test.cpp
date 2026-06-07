/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 */

#include <string>
#include <fstream>
#include <iostream>
#include <cstring>
#include <set>
#include <map>
#include <vector>
#include <iomanip>
#include "GnssStruct.h"
#include "TimeConvert.h"
#include "GnssFunc.h"
#include "RinexNavStore.hpp"
#include "RinexObsReader.h"
#include "SPPCode.h"

#define debug 0

using namespace std;

// 系统名称到系统代码的映射
std::map<std::string, std::string> sysNameMap = {
    {"GPS", "G"},
    {"BDS", "C"},
    {"Galileo", "E"},
    {"GLONASS", "R"},
    {"QZSS", "J"},
    {"IRNSS", "I"}
};

// 运行单个系统的 SPP 测试
void runSingleSystemSPP(const string& system, 
                        const string& roverFile, 
                        const string& navFile,
                        const string& outputPath,
                        bool ionoCorrect, 
                        bool tropCorrect,std::map<string, std::set<string>> sysTypes) {
    
    std::cout << "\n--- " << system << " Test ---" << std::endl;
    
    // 获取系统代码
    auto it = sysNameMap.find(system);
    if (it == sysNameMap.end()) {
        std::cerr << "Unknown system: " << system << std::endl;
        return;
    }
    std::string sysCode = it->second;
    
    // 构建输出文件名
    std::string atmosFlag = "";
    if (!ionoCorrect && !tropCorrect) {
        atmosFlag = "_no_atmos";
    } else if (ionoCorrect && !tropCorrect) {
        atmosFlag = "_iono_only";
    } else if (!ionoCorrect && tropCorrect) {
        atmosFlag = "_trop_only";
    } else {
        atmosFlag = "_full_atmos";
    }
    
    std::string solFile = outputPath + "spp_" + system + atmosFlag + ".out";
    
    // 创建导航数据对象（静态变量只加载一次）
    static std::map<std::string, RinexNavStore> navStoreMap;
    RinexNavStore* pNavStore;
    
    auto navIt = navStoreMap.find(navFile);
    if (navIt == navStoreMap.end()) {
        RinexNavStore& newStore = navStoreMap[navFile];
        if (!newStore.loadFile(const_cast<string&>(navFile))) {
            std::cerr << "Error loading nav file for " << system << std::endl;
            return;
        }
        pNavStore = &newStore;
    } else {
        pNavStore = &(navIt->second);
    }
    
    // 创建并配置 SPP 对象
    SPPCode spp;
    spp.setSystemCode(sysCode);
    
    // 调用 full_solve 获取结果（参数顺序：pStore, roverFile, TGD_Bool, Trop_Bool, Iono_Bool）
    std::vector<SPPResult> results = spp.full_solve(pNavStore, const_cast<string&>(roverFile), sysTypes,true, tropCorrect, ionoCorrect);
    
    // 输出到文件（exam5.3 格式：不含 TGD 字段）
    std::fstream solStream(solFile, ios::out);
    if (!solStream) {
        std::cerr << "Error opening output file: " << solFile << std::endl;
        return;
    }
    
    // 文件头
    solStream << "# YDSTime X Y Z E N U PDOP NSAT Sigma0 MeanResidual RMSResidual MaxResidual" << std::endl;
    
    // 输出数据
    for (const auto& result : results) {
        solStream << result.ydsTime
                  << " " << std::fixed << std::setprecision(3) << result.xyz.transpose()
                  << " " << std::fixed << std::setprecision(3) << result.enu.transpose()
                  << " " << std::fixed << std::setprecision(2) << result.pdop
                  << " " << result.nSat
                  << " " << std::fixed << std::setprecision(3) << result.sigma0
                  << " " << std::fixed << std::setprecision(3) << result.meanResidual
                  << " " << std::fixed << std::setprecision(3) << result.rmsResidual
                  << " " << std::fixed << std::setprecision(3) << result.maxResidual
                  << std::endl;
    }
    
    solStream.close();
    std::cout << system << " " << atmosFlag << " -> " << solFile << std::endl;
}

int main() {
    std::cout << "=== SPP Atmospheric Correction Test ===" << std::endl;
    
    // 文件路径配置
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    std::cout << "Rover file: " << roverFile << std::endl;
    std::cout << "Nav file: " << navFile << std::endl;
    
    // 设置输出路径
    std::string outputPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/spp/";
    std::string cmd = "mkdir -p " + outputPath;
    system(cmd.c_str());
    
    // 测试模式：无大气校正、仅电离层、仅对流层、全校正
    std::vector<std::pair<bool, bool>> testModes = {
        {true, true}, // 全校正
        {false, false},  // 无大气校正
        {true, false},   // 仅电离层
        {false, true},   // 仅对流层

    };
    std::map<string, std::set<string>>  sysTypes = {

        {"G", {"C1C"}},   // GPS L1 C/A
        {"C", {"C2I"}},   // BDS-2 B1I（或 C1I，取决版本）
        {"E", {"C1X"}},   // Galileo E1
        {"R", {"C1C"}},   // GLONASS L1
        {"J", {"C1C"}},   // QZSS L1
        {"I", {"C5A"}}    // IRNSS L5
    };

    // // QZSS 单系统测试
    // std::cout << "\n=== QZSS Single System Tests ===" << std::endl;
    // for (const auto& mode : testModes) {
    //     runSingleSystemSPP("QZSS", roverFile, navFile, outputPath, mode.first, mode.second,sysTypes);
    // }
    //
    // // IRNSS 单系统测试
    // std::cout << "\n=== IRNSS Single System Tests ===" << std::endl;
    // for (const auto& mode : testModes) {
    //     runSingleSystemSPP("IRNSS", roverFile, navFile, outputPath, mode.first, mode.second,sysTypes);
    // }

    // BDS 单系统测试
    std::cout << "\n=== BDS Single System Tests ===" << std::endl;
    for (const auto& mode : testModes) {
        runSingleSystemSPP("BDS", roverFile, navFile, outputPath, mode.first, mode.second,sysTypes);
    }

     // Galileo 单系统测试
    std::cout << "\n=== Galileo Single System Tests ===" << std::endl;
    for (const auto& mode : testModes) {
        runSingleSystemSPP("Galileo", roverFile, navFile, outputPath, mode.first, mode.second,sysTypes);
    }
    //
    // GPS 单系统测试
    std::cout << "\n=== GPS Single System Tests ===" << std::endl;
    for (const auto& mode : testModes) {
        runSingleSystemSPP("GPS", roverFile, navFile, outputPath, mode.first, mode.second,sysTypes);
    }
    //
    // GLONASS 单系统测试
    std::cout << "\n=== GLONASS Single System Tests ===" << std::endl;
    for (const auto& mode : testModes) {
        runSingleSystemSPP("GLONASS", roverFile, navFile, outputPath, mode.first, mode.second,sysTypes);
    }
    //




    std::cout << "\n=== All SPP tests completed ===" << std::endl;
    std::cout << "Results saved to: " << outputPath << std::endl;
    
    return 0;
}