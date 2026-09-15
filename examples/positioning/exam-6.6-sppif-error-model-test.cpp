/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 */

#include <string>
#include <fstream>
#include <iostream>
#include <set>
#include <map>
#include <vector>
#include <iomanip>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/TimeConvert.h>
#include <gnsslab/GnssFunc.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/RinexObsReader.h>
#include <gnsslab/SPPIFCode.h>

#define debug 0

using namespace std;

void runSPPIFWithErrorModel(const string& system,
                            const string& roverFile,
                            const string& navFile,
                            const string& outputPath,
                            const string& modelFlag,
                            bool relativityEnable,
                            bool earthRotationEnable,
                            bool TGD_bool,
                            bool Trop_bool,
                            const std::map<string, std::set<string>>& selectedTypes,
                            const std::map<string, std::pair<string, string>>& ifCodeTypes) {
    
    std::cout << "  " << system << " - " << modelFlag << std::endl;
    
    std::string solFile = outputPath + "sppif_" + system + modelFlag + ".out";
    
    // 加载导航文件（静态变量只加载一次）
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
    // 根据系统名设置系统代码
    std::map<std::string, std::string> sysCodeMap = {
        {"GPS", "G"}, {"BDS", "C"}, {"Galileo", "E"}, {"GLONASS", "R"}
    };
    sppif.setSystemCode(sysCodeMap[system]);
    sppif.setSelectedTypes(selectedTypes);
    sppif.setIFCodeTypes(const_cast<std::map<string, std::pair<string, string>>&>(ifCodeTypes));
    sppif.setRelativityEnable(relativityEnable);
    sppif.setEarthRotationEnable(earthRotationEnable);
    
    // 调用 full_solve
    std::vector<SPPIFResult> results = sppif.full_solve(
        pNavStore, const_cast<std::map<string, std::pair<string, string>>&>(ifCodeTypes),
        const_cast<string&>(roverFile), TGD_bool, Trop_bool);
    
    // 输出到文件
    std::fstream solStream(solFile, ios::out);
    if (!solStream) {
        std::cerr << "Error opening output file: " << solFile << std::endl;
        return;
    }
    
    solStream << "# YDSTime X Y Z E N U PDOP NSAT Sigma0 MeanResidual RMSResidual MaxResidual" << std::endl;
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
    std::cout << "  Output -> " << solFile << std::endl;
}

int main() {
    std::cout << "=== SPPIF Error Model Test ===" << std::endl;
    
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    std::string outputPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/sppif_error_model/";
    string cmd = "mkdir -p " + outputPath;
    system(cmd.c_str());
    
    // 各系统的 IF 频率对和观测类型
    std::map<std::string, std::pair<std::string, std::string>> gpsIF = {{"G", {"C1", "C2"}}};
    std::map<std::string, std::set<std::string>> gpsTypes = {{"G", {"C1W", "C2W"}}};
    
    std::map<std::string, std::pair<std::string, std::string>> bdsIF = {{"C", {"C2", "C7"}}};
    std::map<std::string, std::set<std::string>> bdsTypes = {{"C", {"C2I", "C7I"}}};
    
    std::map<std::string, std::pair<std::string, std::string>> galIF = {{"E", {"C1", "C5"}}};
    std::map<std::string, std::set<std::string>> galTypes = {{"E", {"C1X", "C5X"}}};
    
    std::map<std::string, std::pair<std::string, std::string>> gloIF = {{"R", {"C1", "C2"}}};
    std::map<std::string, std::set<std::string>> gloTypes = {{"R", {"C1C", "C2C"}}};
    
    struct SystemConfig {
        string name;
        std::map<string, std::set<string>> types;
        std::map<string, std::pair<string, string>> ifCodes;
    };
    
    vector<SystemConfig> systems = {
        {"GPS", gpsTypes, gpsIF},
        {"BDS", bdsTypes, bdsIF},
        {"Galileo", galTypes, galIF},
        {"GLONASS", gloTypes, gloIF}
    };
    
    for (const auto& sys : systems) {
        std::cout << "\n--- " << sys.name << " ---" << std::endl;
        
        // 完整模型
        runSPPIFWithErrorModel(sys.name, roverFile, navFile, outputPath,
                               "_full_model", true, true, true, true,
                               sys.types, sys.ifCodes);
        // 无相对论
        runSPPIFWithErrorModel(sys.name, roverFile, navFile, outputPath,
                               "_no_relativity", false, true, true, true,
                               sys.types, sys.ifCodes);
        // 无地球自转
        runSPPIFWithErrorModel(sys.name, roverFile, navFile, outputPath,
                               "_no_earth_rotation", true, false, true, true,
                               sys.types, sys.ifCodes);
        // 无对流层
        runSPPIFWithErrorModel(sys.name, roverFile, navFile, outputPath,
                               "_no_trop", true, true, true, false,
                               sys.types, sys.ifCodes);
        // 无 TGD
        runSPPIFWithErrorModel(sys.name, roverFile, navFile, outputPath,
                               "_no_tgd", true, true, false, true,
                               sys.types, sys.ifCodes);
    }
    
    std::cout << "\n=== All SPPIF error model tests completed ===" << std::endl;
    return 0;
}
