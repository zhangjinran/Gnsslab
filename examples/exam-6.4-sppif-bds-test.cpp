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

void runBDSSPPIFWithCodeCombination(const string& roverFile,
                                    const string& navFile,
                                    const string& outputPath,
                                    const string& codeCombination,
                                    const std::map<string, std::pair<string, string>>& ifCodeTypes,
                                    const std::map<string, std::set<string>>& selectedTypes) {
    
    std::cout << "\n--- BDS SPPIF Test (" << codeCombination << ") ---" << std::endl;
    
    // 仅输出全改正模式（TGD + 对流层）
    std::string solFile = outputPath + "sppif_BDS_" + codeCombination + ".out";
    
    static std::map<std::string, RinexNavStore> navStoreMap;
    RinexNavStore* pNavStore;
    
    auto navIt = navStoreMap.find(navFile);
    if (navIt == navStoreMap.end()) {
        RinexNavStore& newStore = navStoreMap[navFile];
        if (!newStore.loadFile(const_cast<string&>(navFile))) {
            std::cerr << "Error loading nav file" << std::endl;
            return;
        }
        pNavStore = &newStore;
    } else {
        pNavStore = &(navIt->second);
    }
    
    SPPIFCode sppif;
    sppif.setSystemCode("C");
    sppif.setSelectedTypes(selectedTypes);
    
    // 使用全改正模式：TGD + 对流层
    std::vector<SPPIFResult> results = sppif.full_solve(pNavStore, ifCodeTypes, const_cast<string&>(roverFile), true, true);
    
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
    std::cout << "Output -> " << solFile << std::endl;
}

int main() {
    std::cout << "=== BDS SPPIF Code Combination Test ===" << std::endl;
    
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    std::cout << "Rover file: " << roverFile << std::endl;
    std::cout << "Nav file: " << navFile << std::endl;
    
    std::string outputPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/sppif_bds/";
    std::string cmd = "mkdir -p " + outputPath;
    system(cmd.c_str());
    
    // 测试组合：仅全改正模式（TGD + 对流层）
    std::cout << "\n=== BDS Combination: B1C + B2I (C1X+C2I) ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsC1C2 = {{"C", {"C1", "C2"}}};
    std::map<string, std::set<string>> selC1C2 = {{"C", {"C1X", "C2I"}}};
    runBDSSPPIFWithCodeCombination(roverFile, navFile, outputPath, "C1XC2I", bdsC1C2, selC1C2);

    std::cout << "\n=== BDS Combination: B1C + B5A (C1X+C5X) ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsC1C5 = {{"C", {"C1", "C5"}}};
    std::map<string, std::set<string>> selC1C5 = {{"C", {"C1X", "C5X"}}};
    runBDSSPPIFWithCodeCombination(roverFile, navFile, outputPath, "C1XC5X", bdsC1C5, selC1C5);

    std::cout << "\n=== BDS Combination: B1C + B6I (C1X+C6I) ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsC1C6 = {{"C", {"C1", "C6"}}};
    std::map<string, std::set<string>> selC1C6 = {{"C", {"C1X", "C6I"}}};
    runBDSSPPIFWithCodeCombination(roverFile, navFile, outputPath, "C1XC6I", bdsC1C6, selC1C6);

    std::cout << "\n=== BDS Combination: B1C + B7I (C1X+C7I) ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsC1C7 = {{"C", {"C1", "C7"}}};
    std::map<string, std::set<string>> selC1C7 = {{"C", {"C1X", "C7I"}}};
    runBDSSPPIFWithCodeCombination(roverFile, navFile, outputPath, "C1XC7I", bdsC1C7, selC1C7);

    std::cout << "\n=== BDS Combination: B2I + B5A (C2I+C5X) ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsC2C5 = {{"C", {"C2", "C5"}}};
    std::map<string, std::set<string>> selC2C5 = {{"C", {"C2I", "C5X"}}};
    runBDSSPPIFWithCodeCombination(roverFile, navFile, outputPath, "C2IC5X", bdsC2C5, selC2C5);

    std::cout << "\n=== BDS Combination: B2I + B6I (C2I+C6I) ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsC2C6 = {{"C", {"C2", "C6"}}};
    std::map<string, std::set<string>> selC2C6 = {{"C", {"C2I", "C6I"}}};
    runBDSSPPIFWithCodeCombination(roverFile, navFile, outputPath, "C2IC6I", bdsC2C6, selC2C6);

    std::cout << "\n=== BDS Combination: B2I + B7I (C2I+C7I) ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsC2C7 = {{"C", {"C2", "C7"}}};
    std::map<string, std::set<string>> selC2C7 = {{"C", {"C2I", "C7I"}}};
    runBDSSPPIFWithCodeCombination(roverFile, navFile, outputPath, "C2IC7I", bdsC2C7, selC2C7);

    std::cout << "\n=== BDS Combination: B5A + B6I (C5X+C6I) ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsC5C6 = {{"C", {"C5", "C6"}}};
    std::map<string, std::set<string>> selC5C6 = {{"C", {"C5X", "C6I"}}};
    runBDSSPPIFWithCodeCombination(roverFile, navFile, outputPath, "C5XC6I", bdsC5C6, selC5C6);

    std::cout << "\n=== BDS Combination: B5A + B7I (C5X+C7I) ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsC5C7 = {{"C", {"C5", "C7"}}};
    std::map<string, std::set<string>> selC5C7 = {{"C", {"C5X", "C7I"}}};
    runBDSSPPIFWithCodeCombination(roverFile, navFile, outputPath, "C5XC7I", bdsC5C7, selC5C7);

    std::cout << "\n=== BDS Combination: B1C + B1C-2 (C1X+C8X) ===" << std::endl;
    std::map<string, std::pair<string, string>> bdsC1C8 = {{"C", {"C1", "C8"}}};
    std::map<string, std::set<string>> selC1C8 = {{"C", {"C1X", "C8X"}}};
    runBDSSPPIFWithCodeCombination(roverFile, navFile, outputPath, "C1XC8X", bdsC1C8, selC1C8);

    std::cout << "\n=== All BDS SPPIF tests completed ===" << std::endl;
    std::cout << "Results saved to: " << outputPath << std::endl;
    
    return 0;
}