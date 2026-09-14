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
#include "SPPIFCode.h"

#define debug 0

using namespace std;

// 运行北斗单系统的 SPP 测试（带权重参数）
void runBDSSPPWithWeights(const string& roverFile, 
                          const string& navFile,
                          const string& outputPath,
                          double meoWeight,
                          double igsoWeight,
                          double geoWeight,
                          std::map<string, std::set<string>> sysTypes) {
    
    std::cout << "\n--- BDS Test (MEO:" << meoWeight << ", IGSO:" << igsoWeight << ", GEO:" << geoWeight << ") ---" << std::endl;
    
    // 构建输出文件名（格式：spp_BDS_Mxx_Ixx_Gxx.out）
    char weightStr[50];
    sprintf(weightStr, "_M%.2f_I%.2f_G%.2f", meoWeight, igsoWeight, geoWeight);
    std::string solFile = outputPath + "spp_BDS_weight" + std::string(weightStr) + ".out";
    
    // 创建导航数据对象
    RinexNavStore navStore;
    if (!navStore.loadFile(const_cast<string&>(navFile))) {
        std::cerr << "Error loading nav file" << std::endl;
        return;
    }
    RinexNavStore* pNavStore = &navStore;
    
    // 创建并配置 SPP 对象
    SPPCode spp;
    spp.setSystemCode("C");  // BDS
    spp.setSatTypeWeights(meoWeight, igsoWeight, geoWeight);
    
    // 调用 full_solve 获取结果（TGD、对流层、电离层全部开启）
    std::vector<SPPResult> results = spp.full_solve(pNavStore, const_cast<string&>(roverFile), sysTypes, true, true, true);
    
    // 输出到文件
    std::fstream solStream(solFile, ios::out);
    if (!solStream) {
        std::cerr << "Error opening output file: " << solFile << std::endl;
        return;
    }
    
    // 文件头（与 exam5.3 格式一致）
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
    std::cout << "Output -> " << solFile << std::endl;
}

// 运行北斗单系统的 SPPIF 测试（带权重参数，使用 C2I+C7I 组合）
void runBDSSPPIFWithWeights(const string& roverFile, 
                            const string& navFile,
                            const string& outputPath,
                            double meoWeight,
                            double igsoWeight,
                            double geoWeight) {
    
    std::cout << "\n--- BDS SPPIF Test (C2I+C7I) (MEO:" << meoWeight << ", IGSO:" << igsoWeight << ", GEO:" << geoWeight << ") ---" << std::endl;
    
    // 构建输出文件名（格式：sppif_BDS_C2IC7I_Mxx_Ixx_Gxx.out）
    char weightStr[50];
    sprintf(weightStr, "_M%.2f_I%.2f_G%.2f", meoWeight, igsoWeight, geoWeight);
    std::string solFile = outputPath + "sppif_BDS_C1XC5X_weight" + std::string(weightStr) + ".out";
    
    // 创建导航数据对象
    RinexNavStore navStore;
    if (!navStore.loadFile(const_cast<string&>(navFile))) {
        std::cerr << "Error loading nav file" << std::endl;
        return;
    }
    RinexNavStore* pNavStore = &navStore;
    
    // 创建并配置 SPPIF 对象
    SPPIFCode sppif;
    sppif.setSatTypeWeights(meoWeight, igsoWeight, geoWeight);
    
    // 配置 IF 组合：B1I(1561MHz) + B2I(1207MHz)
    std::map<string, std::pair<string, string>> ifCodeTypes = {{"C", {"C2", "C7"}}};
    std::map<string, std::set<string>> selectedTypes = {{"C", {"C2I", "C7I"}}};
    sppif.setSelectedTypes(selectedTypes);
    
    // 调用 full_solve 获取结果（TGD、对流层全部开启）
    std::vector<SPPIFResult> results = sppif.full_solve(pNavStore, ifCodeTypes, const_cast<string&>(roverFile), true, true);
    
    // 输出到文件
    std::fstream solStream(solFile, ios::out);
    if (!solStream) {
        std::cerr << "Error opening output file: " << solFile << std::endl;
        return;
    }
    
    // 文件头（与 SPP 格式一致）
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
    std::cout << "Output -> " << solFile << std::endl;
}

int main() {
    std::cout << "=== BDS Satellite Type Weight Test ===" << std::endl;
    
    // 文件路径配置
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    std::cout << "Rover file: " << roverFile << std::endl;
    std::cout << "Nav file: " << navFile << std::endl;
    
    // 设置输出路径（单独的文件夹）
    std::string outputPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/spp_bds_weight/";
    std::string cmd = "mkdir -p " + outputPath;
    system(cmd.c_str());
    
    // BDS 观测类型配置
    std::map<string, std::set<string>> sysTypes = {
        {"C", {"C2I"}}   // BDS-2 B1I（或 C1I）
    };
    

    // runBDSSPPWithWeights(roverFile, navFile, outputPath, 1.0, 0.3, 0.25, sysTypes);
    //
    // // 测试用例8：GEO 权重设为 0（彻底排除 GEO）
    // runBDSSPPWithWeights(roverFile, navFile, outputPath, 1.0, 0.3, 0.0, sysTypes);

    // ==================== SPPIF 测试（C2I+C7I）====================
    std::cout << "\n=== BDS SPPIF Weight Test (C2I+C7I) ===" << std::endl;
    
    // 设置 SPPIF 输出路径
    std::string outputPathIF = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/sppif_bds_weight/";
    std::string cmdIF = "mkdir -p " + outputPathIF;
    system(cmdIF.c_str());
    
    // 测试用例1：默认权重（MEO:1.0, IGSO:0.3, GEO:0.25）
    runBDSSPPIFWithWeights(roverFile, navFile, outputPathIF, 1.0, 0.3, 0.25);
    
    // 测试用例2：GEO 权重设为 0（彻底排除 GEO）
    runBDSSPPIFWithWeights(roverFile, navFile, outputPathIF, 1.0, 0.3, 0.0);
    runBDSSPPIFWithWeights(roverFile, navFile, outputPathIF, 1.0, 0.5, 0.0);
    runBDSSPPIFWithWeights(roverFile, navFile, outputPathIF, 1.0, 1, 0.0);

    std::cout << "\n=== All tests completed ===" << std::endl;
    
    return 0;
}