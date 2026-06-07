/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 */

#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <iomanip>
#include "GnssStruct.h"
#include "RinexNavStore.hpp"
#include "SPPCode.h"

#define debug 1

using namespace std;

int main() {
    std::cout << "=== BDS Diagnostic Test ===" << std::endl;
    
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    std::cout << "\n1. Loading Navigation File..." << std::endl;
    std::cout << "   Nav file: " << navFile << std::endl;
    
    RinexNavStore navStore;
    if (!navStore.loadFile(const_cast<string&>(navFile))) {
        std::cerr << "Error loading nav file!" << std::endl;
        return -1;
    }
    
    // 创建 SPPCode 对象用于调用 getBDSSatType
    SPPCode spp;
    spp.setRinexNavStore(&navStore);
    
    // 获取第一个星历的时间作为参考时间
    CommonTime refEpoch;
    if (!navStore.bdsEphData.empty()) {
        refEpoch = navStore.bdsEphData.begin()->second.begin()->first;
    }
    
    // 诊断1: 打印ionoCorrData存储格式
    std::cout << "\n2. Ionospheric Correction Data Analysis" << std::endl;
    std::cout << "=========================================" << std::endl;
    if (navStore.ionoCorrData.empty()) {
        std::cout << "   No ionospheric correction data found!" << std::endl;
    } else {
        std::cout << "   Number of iono entries: " << navStore.ionoCorrData.size() << std::endl;
        for (const auto& entry : navStore.ionoCorrData) {
            std::cout << "   Key: '" << entry.first << "' -> ";
            for (size_t i = 0; i < entry.second.size(); ++i) {
                std::cout << std::scientific << std::setprecision(4) << entry.second[i];
                if (i < entry.second.size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;
        }
    }
    
    // 诊断2: 统计BDS星历数据
    std::cout << "\n3. BDS Ephemeris Data Analysis" << std::endl;
    std::cout << "=================================" << std::endl;
    
    std::map<std::string, int> bdsSatTypeCount;
    
    std::cout << "   Total BDS satellites with ephemeris: " << navStore.bdsEphData.size() << std::endl;
    std::cout << "   PRN list: ";
    for (const auto& entry : navStore.bdsEphData) {
        std::cout << entry.first << " ";
        std::string type = spp.getBDSSatType(entry.first, refEpoch);
        bdsSatTypeCount[type]++;
    }
    std::cout << std::endl;
    
    std::cout << "\n   Satellite type distribution:" << std::endl;
    for (const auto& entry : bdsSatTypeCount) {
        std::cout << "   - " << std::setw(12) << std::left << entry.first 
                  << ": " << entry.second << " satellites" << std::endl;
    }
    
    // 诊断3: 输出所有BDS卫星详细信息
    std::cout << "\n4. All BDS Satellite Detailed Info" << std::endl;
    std::cout << "===================================" << std::endl;
    
    for (const auto& entry : navStore.bdsEphData) {
        const SatID& sat = entry.first;
        std::string type = spp.getBDSSatType(sat, refEpoch);
        
        std::cout << "\n   Satellite " << sat << " (" << type << "):" << std::endl;
        
        if (!entry.second.empty()) {
            NavEphBDS eph = entry.second.begin()->second;
            
            std::cout << "     sqrt_A: " << std::fixed << std::setprecision(6) << eph.sqrt_A << " (sqrt(m))" << std::endl;
            std::cout << "     ecc: " << std::fixed << std::setprecision(9) << eph.ecc << std::endl;
            std::cout << "     i0: " << std::fixed << std::setprecision(6) << eph.i0 << " (semi-circles)" << std::endl;
            std::cout << "     Omega_0: " << std::fixed << std::setprecision(6) << eph.OMEGA_0 << " (semi-circles)" << std::endl;
            std::cout << "     omega: " << std::fixed << std::setprecision(6) << eph.omega << " (semi-circles)" << std::endl;
            std::cout << "     IDOT: " << std::scientific << std::setprecision(6) << eph.IDOT << " (semi-circles/sec)" << std::endl;
            std::cout << "     OMEGA_DOT: " << std::scientific << std::setprecision(6) << eph.OMEGA_DOT << " (semi-circles/sec)" << std::endl;
        }
    }
    
    std::cout << "\n=== Diagnostic test completed ===" << std::endl;
    
    return 0;
}