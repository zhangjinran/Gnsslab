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
std::map<string, std::set<string>>  sysTypes = {

    {"G", {"C1C"}},   // GPS L1 C/A
    {"C", {"C1X"}},   // BDS-2 B1I（或 C1I，取决版本）
    {"E", {"C1X"}},   // Galileo E1
    {"R", {"C1C"}},   // GLONASS L1
    {"J", {"C1C"}},   // QZSS L1
    {"I", {"C5A"}}    // IRNSS L5
};


// 运行单个系统的 TGD 测试
void runTGDTest(const string& system, 
                const string& roverFile, 
                const string& navFile,
                const string& outputPath,
                bool tgdCorrect) {
    
    std::cout << "\n--- " << system << " TGD Test (" << (tgdCorrect ? "With TGD" : "No TGD") << ") ---" << std::endl;
    
    // 获取系统代码
    auto it = sysNameMap.find(system);
    if (it == sysNameMap.end()) {
        std::cerr << "Unknown system: " << system << std::endl;
        return;
    }
    std::string sysCode = it->second;
    
    // 构建输出文件名
    std::string tgdFlag = tgdCorrect ? "_with_tgd" : "_no_tgd";
    std::string solFile = outputPath + "spp_" + system + tgdFlag + ".out";
    std::string tgdFile = solFile.substr(0, solFile.find_last_of('.')) + "_tgd.out";
    
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
    std::vector<SPPResult> results = spp.full_solve(pNavStore, const_cast<string&>(roverFile), sysTypes, tgdCorrect, true, true);
    
    // 输出到文件（exam5.4 格式：包含 TGD 字段）
    std::fstream solStream(solFile, ios::out);
    std::fstream tgdStream(tgdFile, ios::out);
    
    if (!solStream) {
        std::cerr << "Error opening output file: " << solFile << std::endl;
        return;
    }
    if (!tgdStream) {
        std::cerr << "Error opening TGD output file: " << tgdFile << std::endl;
        return;
    }
    
    // 文件头（包含 TGD 字段）
    solStream << "# YDSTime X Y Z E N U PDOP NSAT Sigma0 MeanResidual RMSResidual MaxResidual MeanTGD MaxTGD MinTGD" << std::endl;
    tgdStream << "# YDSTime SatTGDs(格式: SatID=TGD(m))" << std::endl;
    
    // 输出数据
    for (const auto& result : results) {
        // SPP 结果
        solStream << result.ydsTime
                  << " " << std::fixed << std::setprecision(3) << result.xyz.transpose()
                  << " " << std::fixed << std::setprecision(3) << result.enu.transpose()
                  << " " << std::fixed << std::setprecision(2) << result.pdop
                  << " " << result.nSat
                  << " " << std::fixed << std::setprecision(3) << result.sigma0
                  << " " << std::fixed << std::setprecision(3) << result.meanResidual
                  << " " << std::fixed << std::setprecision(3) << result.rmsResidual
                  << " " << std::fixed << std::setprecision(3) << result.maxResidual
                  << " " << std::fixed << std::setprecision(3) << result.meanTGD
                  << " " << std::fixed << std::setprecision(3) << result.maxTGD
                  << " " << std::fixed << std::setprecision(3) << result.minTGD
                  << std::endl;
        
        // TGD 改正值（每个历元一行）
        tgdStream << result.ydsTime;
        for (const auto& tgd : result.satTGDData) {
            tgdStream << " " << tgd.first << "=" << std::fixed << std::setprecision(3) << tgd.second;
        }
        tgdStream << std::endl;
    }
    
    solStream.close();
    tgdStream.close();
    std::cout << system << " " << tgdFlag << " -> " << solFile << std::endl;
}

int main() {
    std::cout << "=== SPP TGD Correction Test ===" << std::endl;
    
    // 文件路径配置
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    std::cout << "Rover file: " << roverFile << std::endl;
    std::cout << "Nav file: " << navFile << std::endl;
    
    // 设置输出路径（按照项目规范输出到 gnss_draw/data/ 目录）
    std::string outputPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/spp_tgd/";
    std::string cmd = "mkdir -p " + outputPath;
    system(cmd.c_str());
    
    // 测试模式：带 TGD 改正和不带 TGD 改正
    std::vector<bool> tgdModes = {true, false};
    
    // GPS 单系统测试
    std::cout << "\n=== GPS TGD Tests ===" << std::endl;
    for (bool tgdMode : tgdModes) {
        runTGDTest("GPS", roverFile, navFile, outputPath, tgdMode);
    }
    
    // BDS 单系统测试
    std::cout << "\n=== BDS TGD Tests ===" << std::endl;
    for (bool tgdMode : tgdModes) {
        runTGDTest("BDS", roverFile, navFile, outputPath, tgdMode);
    }
    
    // Galileo 单系统测试
    std::cout << "\n=== Galileo TGD Tests ===" << std::endl;
    for (bool tgdMode : tgdModes) {
        runTGDTest("Galileo", roverFile, navFile, outputPath, tgdMode);
    }
    
    // GLONASS 单系统测试（GLONASS 没有 TGD，但也测试一下）
    std::cout << "\n=== GLONASS TGD Tests ===" << std::endl;
    for (bool tgdMode : tgdModes) {
        runTGDTest("GLONASS", roverFile, navFile, outputPath, tgdMode);
    }
    
    // 多系统组合测试 (GPS+BDS)
    std::cout << "\n=== GPS+BDS Combined TGD Tests ===" << std::endl;
    std::string combinedSolFileWithTGD = outputPath + "spp_gps_bds_with_tgd.out";
    std::string combinedSolFileNoTGD = outputPath + "spp_gps_bds_no_tgd.out";
    std::string combinedTgdFileWithTGD = combinedSolFileWithTGD.substr(0, combinedSolFileWithTGD.find_last_of('.')) + "_tgd.out";
    std::string combinedTgdFileNoTGD = combinedSolFileNoTGD.substr(0, combinedSolFileNoTGD.find_last_of('.')) + "_tgd.out";
    
    RinexNavStore navStore;
    if (!navStore.loadFile(const_cast<string&>(navFile))) {
        std::cerr << "Error loading nav file for combined test" << std::endl;
        return -1;
    }
    
    SPPCode sppCombined;
    sppCombined.setSystemCode("");  // 空字符串表示多系统模式
    
    // 带 TGD 改正
    std::vector<SPPResult> resultsWithTGD = sppCombined.full_solve(&navStore, const_cast<string&>(roverFile), sysTypes, true, true);
    
    // 输出结果
    std::fstream solWithTGD(combinedSolFileWithTGD, ios::out);
    std::fstream tgdWithTGD(combinedTgdFileWithTGD, ios::out);
    solWithTGD << "# YDSTime X Y Z E N U PDOP NSAT Sigma0 MeanResidual RMSResidual MaxResidual MeanTGD MaxTGD MinTGD" << std::endl;
    tgdWithTGD << "# YDSTime SatTGDs(格式: SatID=TGD(m))" << std::endl;
    for (const auto& result : resultsWithTGD) {
        solWithTGD << result.ydsTime
                   << " " << std::fixed << std::setprecision(3) << result.xyz.transpose()
                   << " " << std::fixed << std::setprecision(3) << result.enu.transpose()
                   << " " << std::fixed << std::setprecision(2) << result.pdop
                   << " " << result.nSat
                   << " " << std::fixed << std::setprecision(3) << result.sigma0
                   << " " << std::fixed << std::setprecision(3) << result.meanResidual
                   << " " << std::fixed << std::setprecision(3) << result.rmsResidual
                   << " " << std::fixed << std::setprecision(3) << result.maxResidual
                   << " " << std::fixed << std::setprecision(3) << result.meanTGD
                   << " " << std::fixed << std::setprecision(3) << result.maxTGD
                   << " " << std::fixed << std::setprecision(3) << result.minTGD
                   << std::endl;
        tgdWithTGD << result.ydsTime;
        for (const auto& tgd : result.satTGDData) {
            tgdWithTGD << " " << tgd.first << "=" << std::fixed << std::setprecision(3) << tgd.second;
        }
        tgdWithTGD << std::endl;
    }
    solWithTGD.close();
    tgdWithTGD.close();
    std::cout << "GPS+BDS with TGD -> " << combinedSolFileWithTGD << std::endl;
    
    // 不带 TGD 改正
    std::vector<SPPResult> resultsNoTGD = sppCombined.full_solve(&navStore, const_cast<string&>(roverFile), sysTypes, false, true,true);
    
    // 输出结果
    std::fstream solNoTGD(combinedSolFileNoTGD, ios::out);
    std::fstream tgdNoTGD(combinedTgdFileNoTGD, ios::out);
    solNoTGD << "# YDSTime X Y Z E N U PDOP NSAT Sigma0 MeanResidual RMSResidual MaxResidual MeanTGD MaxTGD MinTGD" << std::endl;
    tgdNoTGD << "# YDSTime SatTGDs(格式: SatID=TGD(m))" << std::endl;
    for (const auto& result : resultsNoTGD) {
        solNoTGD << result.ydsTime
                 << " " << std::fixed << std::setprecision(3) << result.xyz.transpose()
                 << " " << std::fixed << std::setprecision(3) << result.enu.transpose()
                 << " " << std::fixed << std::setprecision(2) << result.pdop
                 << " " << result.nSat
                 << " " << std::fixed << std::setprecision(3) << result.sigma0
                 << " " << std::fixed << std::setprecision(3) << result.meanResidual
                 << " " << std::fixed << std::setprecision(3) << result.rmsResidual
                 << " " << std::fixed << std::setprecision(3) << result.maxResidual
                 << " " << std::fixed << std::setprecision(3) << result.meanTGD
                 << " " << std::fixed << std::setprecision(3) << result.maxTGD
                 << " " << std::fixed << std::setprecision(3) << result.minTGD
                 << std::endl;
        tgdNoTGD << result.ydsTime;
        for (const auto& tgd : result.satTGDData) {
            tgdNoTGD << " " << tgd.first << "=" << std::fixed << std::setprecision(3) << tgd.second;
        }
        tgdNoTGD << std::endl;
    }
    solNoTGD.close();
    tgdNoTGD.close();
    std::cout << "GPS+BDS no TGD -> " << combinedSolFileNoTGD << std::endl;
    
    std::cout << "\n=== All TGD tests completed ===" << std::endl;
    std::cout << "Results saved to: " << outputPath << std::endl;
    
    return 0;
}