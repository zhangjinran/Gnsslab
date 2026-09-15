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
#include <vector>
#include <iomanip>
#include <algorithm>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/TimeConvert.h>
#include <gnsslab/GnssFunc.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/RinexObsReader.h>
#include <gnsslab/SPPIFCode.h>

#define debug 0

using namespace std;

int main() {
    std::cout << "=== BDS IF Code (C1X+C5X) Satellite Statistics ===" << std::endl;
    
    // 文件路径配置
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    std::cout << "\nRover file: " << roverFile << std::endl;
    std::cout << "Nav file: " << navFile << std::endl;
    
    // 配置 IF 组合类型：C1X + C5X
    std::map<string, std::pair<string, string>> ifCodeTypes = {
        {"C", {"C2I", "C7I"}}  // BDS B1I + B2I 做 IF 组合
    };
    std::map<string, std::set<string>> selectedTypes = {
        {"C", {"C2I", "C7I"}}  // 读取 BDS 的这些观测类型
    };
    
    // 统计数据结构
    std::map<SatID, int> satEpochCount;  // 每个卫星参与的历元数
    int totalEpochs = 0;
    
    std::cout << "\n=== Processing Epochs ===" << std::endl;
    
    // 读取观测数据获取每个历元的卫星信息
    RinexObsReader obsReader;
    
    // 设置要读取的观测类型
    obsReader.setSelectedTypes(const_cast<std::map<string, std::set<string>>&>(selectedTypes));
    
    if (!obsReader.loadFile(roverFile)) {
        std::cerr << "Error loading obs file!" << std::endl;
        return -1;
    }
    
    // 解析 RINEX 头部
    obsReader.parseRinexHeader();
    
    // 读取观测数据
    ObsData obsData;
    while (true) {
        try {
            obsData = obsReader.parseRinexObs();
        } catch (...) {
            // 读取完毕或出错
            break;
        }
        
        totalEpochs++;
        
        // 获取当前历元时间
        YDSTime yds = CommonTime2YDSTime(obsData.epoch);
        
        // 筛选北斗卫星（只统计有 C1X 和 C5X 观测值的卫星）
        std::vector<SatID> currentSats;
        for (const auto& stv : obsData.satTypeValueData) {
            if (stv.first.system == "C") {  // 只统计北斗卫星
                const auto& types = stv.second;
                // 检查是否有 C1X 和 C5X 观测值
                bool hasC1X = (types.find("C2I") != types.end());
                bool hasC5X = (types.find("C7I") != types.end());
                
                if (hasC1X && hasC5X) {
                    currentSats.push_back(stv.first);
                    satEpochCount[stv.first]++;
                }
            }
        }
        
        // 输出当前历元信息
        std::cout << "\nEpoch " << std::setw(3) << totalEpochs << ": " 
                  << yds.year << " " << std::setw(3) << yds.doy << " " 
                  << std::fixed << std::setprecision(3) << yds.sod << " GPS" << std::endl;
        std::cout << "BDS Satellites (" << currentSats.size() << "): ";
        for (const auto& sat : currentSats) {
            std::cout << sat << " ";
        }
        std::cout << std::endl;
    }
    
    // 输出统计结果
    std::cout << "\n=== Satellite Participation Statistics ===" << std::endl;
    std::cout << "Total epochs processed: " << totalEpochs << std::endl;
    std::cout << "Total BDS satellites used: " << satEpochCount.size() << std::endl;
    
    // 按卫星 ID 排序输出
    std::vector<std::pair<SatID, int>> sortedSatCounts(satEpochCount.begin(), satEpochCount.end());
    std::sort(sortedSatCounts.begin(), sortedSatCounts.end(), 
              [](const std::pair<SatID, int>& a, const std::pair<SatID, int>& b) {
                  return a.first.id < b.first.id;
              });
    
    std::cout << "\nSatellite ID | Epoch Count | Participation Rate" << std::endl;
    std::cout << "-----------------------------------------------" << std::endl;
    for (const auto& entry : sortedSatCounts) {
        double rate = (double)entry.second / totalEpochs * 100;
        std::cout << std::setw(14) << entry.first 
                  << std::setw(13) << entry.second 
                  << std::setw(19) << std::fixed << std::setprecision(1) << rate << "%" << std::endl;
    }
    
    // 统计卫星类型分布
    std::map<std::string, int> satTypeCount;
    for (const auto& entry : sortedSatCounts) {
        int satId = entry.first.id;
        std::string satType;
        if (satId >= 1 && satId <= 5) {
            satType = "GEO";
        } else if (satId >= 6 && satId <= 10) {
            satType = "IGSO";
        } else {
            satType = "MEO";
        }
        satTypeCount[satType]++;
    }
    
    std::cout << "\n=== BDS Satellite Type Distribution ===" << std::endl;
    std::cout << "GEO satellites: " << satTypeCount["GEO"] << std::endl;
    std::cout << "IGSO satellites: " << satTypeCount["IGSO"] << std::endl;
    std::cout << "MEO satellites: " << satTypeCount["MEO"] << std::endl;
    
    // 统计参与率分布
    int highParticipation = 0;  // > 80%
    int mediumParticipation = 0;  // 50% - 80%
    int lowParticipation = 0;  // < 50%
    
    for (const auto& entry : sortedSatCounts) {
        double rate = (double)entry.second / totalEpochs * 100;
        if (rate > 80) {
            highParticipation++;
        } else if (rate >= 50) {
            mediumParticipation++;
        } else {
            lowParticipation++;
        }
    }
    
    std::cout << "\n=== Participation Rate Distribution ===" << std::endl;
    std::cout << "High participation (>80%): " << highParticipation << std::endl;
    std::cout << "Medium participation (50-80%): " << mediumParticipation << std::endl;
    std::cout << "Low participation (<50%): " << lowParticipation << std::endl;
    
    std::cout << "\n=== Statistics completed ===" << std::endl;
    
    return 0;
}