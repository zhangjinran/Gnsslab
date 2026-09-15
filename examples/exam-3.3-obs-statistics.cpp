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
#include <algorithm>
#include <filesystem>
#include "GnssStruct.h"
#include "TimeConvert.h"
#include "GnssFunc.h"
#include "RinexObsReader.h"

#define debug 0

using namespace std;

// 系统名称映射
std::map<std::string, std::string> sysNameMap = {
    {"G", "GPS"},
    {"C", "BDS"},
    {"E", "Galileo"},
    {"R", "GLONASS"},
    {"J", "QZSS"},
    {"I", "IRNSS"},
    {"S", "SBAS"}
};

int main() {
    namespace fs = std::filesystem;

    const fs::path dataDir = fs::current_path() / "data";
    const fs::path outputDir =
        fs::current_path() / "outputs" / "obs_statistics_epoch";

    std::cout << "=== GNSS Observation Data Statistics (Epoch-based) ===" << std::endl;
    
    // 文件路径配置
    std::string roverFile =
        (dataDir / "WUH200CHN_R_20250010000_01D_30S_MO.rnx").string();
    
    std::cout << "Rover file: " << roverFile << std::endl;
    
    // 创建观测数据读取器
    RinexObsReader obsReader;
    if (!obsReader.loadFile(roverFile)) {
        std::cerr << "Error loading observation file: " << roverFile << std::endl;
        return 1;
    }
    
    // 先读取第一个历元以解析头文件
    ObsData firstObsData;
    try {
        firstObsData = obsReader.parseRinexObs();
    } catch (EndOfFile &e) {
        std::cerr << "Error: Empty observation file" << std::endl;
        return 1;
    }
    
    // 现在头文件已经解析，可以获取系统和观测类型信息
    const RinexHeader& header = obsReader.getHeader();
    
    // 获取所有系统和观测类型
    std::set<std::string> allSystems;
    std::map<std::string, std::set<std::string>> sysObsTypes;
    for (const auto& sys : header.mapObsTypes) {
        allSystems.insert(sys.first);
        sysObsTypes[sys.first] = std::set<std::string>(sys.second.begin(), sys.second.end());
    }
    
    // 输出文件头信息
    std::cout << "\n--- RINEX Header Information ---" << std::endl;
    std::cout << "Station: " << header.station << std::endl;
    std::cout << "Version: " << header.version << std::endl;
    std::cout << "Approx Position: " << header.antennaPosition.transpose() << std::endl;
    
    // 创建输出目录
    fs::create_directories(outputDir);
    
    // 为每个系统创建输出流
    std::map<std::string, std::fstream> sysStreams;
    std::map<std::string, std::vector<std::string>> sysObsTypeList;
    
    for (const auto& sys : allSystems) {
        std::string fileName =
            (outputDir / ("obs_epoch_" + sys + ".txt")).string();
        sysStreams[sys].open(fileName, ios::out);
        if (!sysStreams[sys]) {
            std::cerr << "Error opening file: " << fileName << std::endl;
            return 1;
        }
        
        // 保存观测类型列表（排序）
        std::vector<std::string> obsTypes(sysObsTypes[sys].begin(), sysObsTypes[sys].end());
        std::sort(obsTypes.begin(), obsTypes.end());
        sysObsTypeList[sys] = obsTypes;
        
        // 写表头
        sysStreams[sys] << "Epoch";
        for (const auto& obsType : obsTypes) {
            sysStreams[sys] << " " << obsType;
        }
        sysStreams[sys] << std::endl;
        
        std::cout << "Created: obs_epoch_" << sys << ".txt (" << sysNameMap[sys] << ")" << std::endl;
    }
    
    // 读取所有历元并统计
    std::cout << "\n--- Processing Observations ---" << std::endl;
    int epochCount = 0;
    
    // 处理第一个历元
    if (firstObsData.epoch != BEGINNING_OF_TIME) {
        std::map<std::string, std::map<std::string, int>> epochStats;
        for (const auto& sys : allSystems) {
            for (const auto& obsType : sysObsTypes[sys]) {
                epochStats[sys][obsType] = 0;
            }
        }
        
        for (const auto& stv : firstObsData.satTypeValueData) {
            const SatID& sat = stv.first;
            std::string sys = sat.system;
            if (allSystems.count(sys) == 0) continue;
            
            for (const auto& tv : stv.second) {
                std::string obsType = tv.first;
                if (sysObsTypes[sys].count(obsType)) {
                    epochStats[sys][obsType]++;
                }
            }
        }
        
        // 转换时间为 MJD 格式
        MJD mjd;
        CommonTime2MJD(firstObsData.epoch, mjd);
        CivilTime civil = CommonTime2CivilTime(firstObsData.epoch);
        double mjdSod = mjd.mjd + civil.second / 86400.0;
        
        for (const auto& sys : allSystems) {
            sysStreams[sys] << std::fixed << std::setprecision(9) << mjdSod;
            for (const auto& obsType : sysObsTypeList[sys]) {
                sysStreams[sys] << " " << epochStats[sys][obsType];
            }
            sysStreams[sys] << std::endl;
        }
        epochCount++;
    }
    
    // 继续处理剩余历元
    while (true) {
        ObsData obsData;
        try {
            obsData = obsReader.parseRinexObs();
        } catch (EndOfFile &e) {
            break;
        }
        
        if (obsData.epoch == BEGINNING_OF_TIME) {
            continue;
        }
        
        // 统计每个系统每个观测类型的卫星数量
        std::map<std::string, std::map<std::string, int>> epochStats;
        for (const auto& sys : allSystems) {
            for (const auto& obsType : sysObsTypes[sys]) {
                epochStats[sys][obsType] = 0;
            }
        }
        
        // 遍历所有卫星
        for (const auto& stv : obsData.satTypeValueData) {
            const SatID& sat = stv.first;
            std::string sys = sat.system;
            
            if (allSystems.count(sys) == 0) continue;
            
            for (const auto& tv : stv.second) {
                std::string obsType = tv.first;
                if (sysObsTypes[sys].count(obsType)) {
                    epochStats[sys][obsType]++;
                }
            }
        }
        
        // 转换时间为 MJD 格式
        MJD mjd;
        CommonTime2MJD(obsData.epoch, mjd);
        CivilTime civil = CommonTime2CivilTime(obsData.epoch);
        double mjdSod = mjd.mjd + civil.second / 86400.0;
        
        // 写入各系统文件
        for (const auto& sys : allSystems) {
            sysStreams[sys] << std::fixed << std::setprecision(9) << mjdSod;
            for (const auto& obsType : sysObsTypeList[sys]) {
                sysStreams[sys] << " " << epochStats[sys][obsType];
            }
            sysStreams[sys] << std::endl;
        }
        
        epochCount++;
        if (epochCount % 100 == 0) {
            std::cout << "\rProcessed " << epochCount << " epochs..." << std::flush;
        }
    }
    std::cout << "\rProcessed " << epochCount << " epochs... Done!" << std::endl;
    
    // 关闭所有文件
    for (auto& stream : sysStreams) {
        stream.second.close();
    }
    
    std::cout << "\n=== Statistics completed ===" << std::endl;
    std::cout << "Output directory: " << outputDir.string() << std::endl;
    
    return 0;
}
