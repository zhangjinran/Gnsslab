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
#include "SPPIFCode.h"

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

// 运行单个系统的 SPP IF 测试
void runSingleSystemSPPIF(const string& system, 
                          const string& roverFile, 
                          const string& navFile,
                          const string& outputPath,
                          const std::map<string, std::pair<string, string>>& ifCodeTypes,
                          const std::map<string, std::set<string>>& selectedTypes,
                          bool tgdCorrect, 
                          bool tropCorrect) {
    
    std::cout << "\n--- " << system << " IF Test ---" << std::endl;
    
    auto it = sysNameMap.find(system);
    if (it == sysNameMap.end()) {
        std::cerr << "Unknown system: " << system << std::endl;
        return;
    }
    std::string sysCode = it->second;
    
    // 构建输出文件名
    std::string atmosFlag = "";
    if (!tgdCorrect && !tropCorrect) {
        atmosFlag = "_no_atmos";
    } else if (tgdCorrect && !tropCorrect) {
        atmosFlag = "_tgd_only";
    } else if (!tgdCorrect && tropCorrect) {
        atmosFlag = "_trop_only";
    } else {
        atmosFlag = "_full_atmos";
    }
    
    std::string solFile = outputPath + "sppif_" + system + atmosFlag + ".out";
    
    // 创建导航数据对象
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
    
    // 创建并配置 SPPIF 对象
    SPPIFCode sppif;
    sppif.setSystemCode(sysCode);
    sppif.setSelectedTypes(selectedTypes);  // 设置要读取的观测类型
    
    // 调用 full_solve 获取结果
    std::vector<SPPIFResult> results = sppif.full_solve(pNavStore, ifCodeTypes, const_cast<string&>(roverFile), tgdCorrect, tropCorrect);
    
    // 输出到文件（exam5.3 格式）
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
    std::cout << "=== SPP IF Combination Test ===" << std::endl;
    
    // 文件路径配置
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    std::cout << "Rover file: " << roverFile << std::endl;
    std::cout << "Nav file: " << navFile << std::endl;
    
    // 设置输出路径
    std::string outputPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/sppif/";
    std::string cmd = "mkdir -p " + outputPath;
    system(cmd.c_str());
    
    // 测试模式
    std::vector<std::pair<bool, bool>> testModes = {
        {true, true},   // TGD + Trop
        {false, false},  // 无校正
        {true, false},   // 仅 TGD
        {false, true},   // 仅 Trop
    };
    // ==================== GLONASS 单系统测试 ====================
    std::cout << "\n=== GLONASS Single System Tests ===" << std::endl;
    std::map<string, std::pair<string, string>> gloIfCodeTypes = {
        {"R", {"C1", "C2"}}  // GLONASS L1 + L2 做 IF 组合
    };
    std::map<string, std::set<string>> gloSelectedTypes = {
        {"R", {"C1C", "C2C"}}  // 读取 GLONASS 的这些观测类型
    };
    for (const auto& mode : testModes) {
        runSingleSystemSPPIF("GLONASS", roverFile, navFile, outputPath, gloIfCodeTypes, gloSelectedTypes, mode.first, mode.second);
    }
    // ==================== GPS 单系统测试 ====================
    std::cout << "\n=== GPS Single System Tests ===" << std::endl;
    std::map<string, std::pair<string, string>> gpsIfCodeTypes = {
        {"G", {"C1", "C2"}}  // GPS L1 + L2 做 IF 组合
    };
    std::map<string, std::set<string>> gpsSelectedTypes = {
        {"G", {"C1W", "C2W", "L1C", "L2W"}}  // 读取 GPS 的这些观测类型
    };
    for (const auto& mode : testModes) {
        runSingleSystemSPPIF("GPS", roverFile, navFile, outputPath, gpsIfCodeTypes, gpsSelectedTypes, mode.first, mode.second);
    }

    // ==================== BDS 单系统测试 ====================
    std::cout << "\n=== BDS Single System Tests ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsIfCodeTypes = {
        {"C", {"C2", "C7"}}  // BDS B1I(1561MHz) + B2I(1207MHz) IF，匹配 TGD 参数
    };
    std::map<string, std::set<string>> bdsSelectedTypes = {
        {"C", {"C2I", "C7I"}}  // 读取 BDS B1I + B2I 观测值
    };
    for (const auto& mode : testModes) {
        runSingleSystemSPPIF("BDS", roverFile, navFile, outputPath, bdsIfCodeTypes, bdsSelectedTypes, mode.first, mode.second);
    }



    // ==================== Galileo 单系统测试 ====================
    std::cout << "\n=== Galileo Single System Tests ===" << std::endl;
    std::map<string, std::pair<string, string>> galIfCodeTypes = {
        {"E", {"C1", "C5"}}  // Galileo E1 + E5a 做 IF 组合
    };
    std::map<string, std::set<string>> galSelectedTypes = {
        {"E", {"C1X", "C5X"}}  // 读取 Galileo 的这些观测类型
    };
    for (const auto& mode : testModes) {
        runSingleSystemSPPIF("Galileo", roverFile, navFile, outputPath, galIfCodeTypes, galSelectedTypes, mode.first, mode.second);
    }

    std::cout << "\n=== All SPP IF tests completed ===" << std::endl;
    std::cout << "Results saved to: " << outputPath << std::endl;
    
    return 0;
}