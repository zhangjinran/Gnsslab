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
#include <gnsslab/GnssStruct.h>
#include <gnsslab/TimeConvert.h>
#include <gnsslab/GnssFunc.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/RinexObsReader.h>
#include <gnsslab/SPPCode.h>

#define debug 0

using namespace std;

// 运行 SPP 测试（带误差模型开关）
void runSPPWithErrorModel(const string& system,
                          const string& roverFile,
                          const string& navFile,
                          const string& outputPath,
                          bool relativityEnable,
                          bool earthRotationEnable,
                          std::map<string, std::set<string>> sysTypes) {
    
    std::cout << "\n--- " << system << " Test (Relativity:" << (relativityEnable ? "ON" : "OFF") 
              << ", EarthRotation:" << (earthRotationEnable ? "ON" : "OFF") << ") ---" << std::endl;
    
    // 构建输出文件名
    std::string modelFlag = "";
    if (!relativityEnable && !earthRotationEnable) {
        modelFlag = "_no_correction";
    } else if (!relativityEnable) {
        modelFlag = "_no_relativity";
    } else if (!earthRotationEnable) {
        modelFlag = "_no_earth_rotation";
    } else {
        modelFlag = "_full_model";
    }
    
    std::string solFile = outputPath + "spp_" + system + modelFlag + ".out";
    
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
    
    // 设置系统代码
    std::map<std::string, std::string> sysNameMap = {
        {"GPS", "G"},
        {"BDS", "C"},
        {"Galileo", "E"},
        {"GLONASS", "R"}
    };
    spp.setSystemCode(sysNameMap[system]);
    
    // 设置误差模型开关
    spp.setRelativityEnable(relativityEnable);
    spp.setEarthRotationEnable(earthRotationEnable);
    
    // 调用 full_solve 获取结果（TGD、对流层、电离层全部开启）
    std::vector<SPPResult> results = spp.full_solve(pNavStore, const_cast<string&>(roverFile), sysTypes, true, true, true);
    
    // 输出到文件（与 exam5.3 格式一致）
    std::fstream solStream(solFile, ios::out);
    if (!solStream) {
        std::cerr << "Error opening output file: " << solFile << std::endl;
        return;
    }
    
    // 文件头
    solStream << "# YDSTime X Y Z E N U PDOP NSAT Sigma0 MeanResidual RMSResidual MaxResidual MeanRelativity MaxRelativity MinRelativity" << std::endl;
    
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
                  << " " << std::fixed << std::setprecision(3) << result.meanRelativity
                  << " " << std::fixed << std::setprecision(3) << result.maxRelativity
                  << " " << std::fixed << std::setprecision(3) << result.minRelativity
                  << std::endl;
    }
    
    solStream.close();
    std::cout << "Output -> " << solFile << std::endl;
}

int main() {
    std::cout << "=== SPP Error Model Test (Relativity & Earth Rotation) ===" << std::endl;
    
    // 文件路径配置
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    std::cout << "Rover file: " << roverFile << std::endl;
    std::cout << "Nav file: " << navFile << std::endl;
    
    // 设置输出路径
    std::string outputPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/spp_error_model/";
    std::string cmd = "mkdir -p " + outputPath;
    system(cmd.c_str());
    
    // 测试模式：四种误差模型组合
    std::vector<std::pair<bool, bool>> testModes = {
        {true, true},   // 完整模型：相对论效应 + 地球自转改正
        {false, true},  // 无相对论效应，有地球自转改正
        {true, false},  // 有相对论效应，无地球自转改正
        {false, false}  // 无任何改正
    };
    
    // ==================== GPS 单系统测试 ====================
    std::cout << "\n=== GPS Single System Tests ===" << std::endl;
    std::map<string, std::set<string>> gpsTypes = {
        {"G", {"C1W"}}
    };
    for (const auto& mode : testModes) {
        runSPPWithErrorModel("GPS", roverFile, navFile, outputPath, mode.first, mode.second, gpsTypes);
    }

    // ==================== BDS 单系统测试 ====================
    std::cout << "\n=== BDS Single System Tests ===" << std::endl;
    std::map<string, std::set<string>> bdsTypes = {
        {"C", {"C2I"}}
    };
    for (const auto& mode : testModes) {
        runSPPWithErrorModel("BDS", roverFile, navFile, outputPath, mode.first, mode.second, bdsTypes);
    }

    // ==================== GLONASS 单系统测试 ====================
    std::cout << "\n=== GLONASS Single System Tests ===" << std::endl;
    std::map<string, std::set<string>> gloTypes = {
        {"R", {"C1C"}}
    };
    for (const auto& mode : testModes) {
        runSPPWithErrorModel("GLONASS", roverFile, navFile, outputPath, mode.first, mode.second, gloTypes);
    }

    // ==================== Galileo 单系统测试 ====================
    std::cout << "\n=== Galileo Single System Tests ===" << std::endl;
    std::map<string, std::set<string>> galTypes = {
        {"E", {"C1X"}}
    };
    for (const auto& mode : testModes) {
        runSPPWithErrorModel("Galileo", roverFile, navFile, outputPath, mode.first, mode.second, galTypes);
    }

    std::cout << "\n=== All SPP error model tests completed ===" << std::endl;
    std::cout << "Results saved to: " << outputPath << std::endl;
    
    return 0;
}