/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *
 * Author: Shoujian Zhang, shjzhang@sgg.whu.edu.cn, 2024-10-10
 *
 * References:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
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
#include <gnsslab/Const.h>

#define DEBUG 1

using namespace std;

// 大气延迟数据结构
struct AtmosphericDelayData {
    CommonTime epoch;
    std::map<SatID, double> ionoDelay;    // 电离层延迟 (米)
    std::map<SatID, double> tropDelay;    // 对流层延迟 (米)
    std::map<SatID, double> elevation;    // 卫星仰角 (度)
    std::map<SatID, double> azimuth;      // 卫星方位角 (度)
};

// 输出大气延迟数据到文件（每颗卫星一个文件）
void writeAtmosphericData(const std::map<SatID, std::vector<std::tuple<CommonTime, double, double, double, double>>>& satDelayData, 
                          const std::string& basePath) {
    
    // 确保目录存在
    std::string cmd = "mkdir -p " + basePath;
    system(cmd.c_str());
    
    // 为每颗卫星创建一个数据文件
    for (const auto& entry : satDelayData) {
        SatID sat = entry.first;
        const auto& dataList = entry.second;
        
        std::string filename = basePath + sat.toString() + "_delay.txt";
        std::ofstream outFile(filename);
        
        outFile << "# Atmospheric Delay Data for Satellite " << sat.toString() << "\n";
        outFile << "# Format: MJD+SOD,epoch_str,iono_delay(m),trop_delay(m),elev(deg),azim(deg)\n";
        
        for (const auto& data : dataList) {
            CommonTime epoch = std::get<0>(data);
            double iono = std::get<1>(data);
            double trop = std::get<2>(data);
            double elev = std::get<3>(data);
            double azim = std::get<4>(data);
            
            CivilTime civil = CommonTime2CivilTime(epoch);
            MJD mjd;
            CommonTime2MJD(epoch, mjd);
            double mjdSod = mjd.mjd + civil.second / 86400.0;
            
            outFile << std::fixed << std::setprecision(9) << mjdSod << ","
                    << civil << ","
                    << std::fixed << std::setprecision(3) << iono << ","
                    << std::fixed << std::setprecision(3) << trop << ","
                    << std::fixed << std::setprecision(2) << elev << ","
                    << std::fixed << std::setprecision(2) << azim << "\n";
        }
        
        outFile.close();
        std::cout << "Created file: " << filename << std::endl;
    }
    
    std::cout << "\nTotal " << satDelayData.size() << " satellite files created in " << basePath << std::endl;
}

int main() {
    std::cout << "=== Atmospheric Delay Calculation Test ===" << std::endl;
    
    // 数据文件路径
    std::string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    // 打开观测文件
    std::fstream roverObsStream(roverFile);
    if (!roverObsStream) {
        std::cerr << "Error: Cannot open rover observation file: " << roverFile << std::endl;
        return -1;
    }
    
    // 加载导航数据
    RinexNavStore navStore;
    if (!navStore.loadFile(navFile)) {
        std::cerr << "Error: Cannot load navigation file: " << navFile << std::endl;
        return -1;
    }
    
    std::cout << "Navigation data loaded successfully" << std::endl;
    
    // 设置观测类型过滤器 - 支持多系统
    std::map<std::string, std::set<std::string>> selectedTypes;
    selectedTypes["G"].insert("C1");
    selectedTypes["C"].insert("C2");
    selectedTypes["E"].insert("C1");
    selectedTypes["R"].insert("C1");




    // 读取观测数据
    RinexObsReader readObsRover;
    readObsRover.setFileStream(&roverObsStream);
    readObsRover.setSelectedTypes(selectedTypes);
    
    // 存储大气延迟数据（按卫星组织）
    std::map<SatID, std::vector<std::tuple<CommonTime, double, double, double, double>>> satDelayData;
    
    // 用于记录每个系统筛选的卫星（只在第一个历元筛选）
    std::map<std::string, std::set<SatID>> selectedSatsBySystem;
    bool firstEpoch = true;
    
    int epochCount = 0;
    
    try {
        while (true) {
            ObsData roverData = readObsRover.parseRinexObs();
            
            CommonTime epoch = roverData.epoch;
            CivilTime civil = CommonTime2CivilTime(epoch);
            
            if (DEBUG && epochCount == 0) {
                std::cout << "\n=== Atmospheric Delay Test ===" << std::endl;
                std::cout << "Processing epoch: " << civil << std::endl;
            }
            
            // 转换观测类型
            convertObsType(roverData);
            
            // 计算卫星位置
            std::map<SatID, Xvt> satXvtTransTime = computeSatPos(roverData, navStore, 0);
            
            if (satXvtTransTime.empty()) {
                epochCount++;
                continue;
            }
            
            // 地球自转改正
            Vector3d xyz = roverData.antennaPosition;
            std::map<SatID, Xvt> satXvtRecTime = earthRotation(xyz, satXvtTransTime);
            
            // 计算仰角和方位角
            SatValueMap satElevData, satAzimData;
            if (std::abs(xyz.norm() - RadiusEarth) < 100000.0) {
                computeElevAzim(xyz, satXvtRecTime, satElevData, satAzimData);
                
                // 第一个历元：按高度角筛选卫星（每个系统高、中、低各一颗）
                if (firstEpoch) {
                    firstEpoch = false;
                    
                    // 按系统分组卫星并按高度角排序（computeElevAzim返回的是度数）
                    std::map<std::string, std::vector<std::pair<double, SatID>>> sysSatElev;
                    for (const auto& entry : satElevData) {
                        std::string sys = entry.first.system;
                        double elev_deg = entry.second;  // 已经是度数，不需要转换
                        sysSatElev[sys].push_back({elev_deg, entry.first});
                    }
                    
                    // 每个系统筛选3颗卫星
                    for (auto& entry : sysSatElev) {
                        std::string sys = entry.first;
                        auto& satElevList = entry.second;
                        
                        // 按高度角排序
                        std::sort(satElevList.begin(), satElevList.end(), 
                                  [](const auto& a, const auto& b) { return a.first > b.first; });
                        
                        int n = satElevList.size();
                        if (n >= 1) selectedSatsBySystem[sys].insert(satElevList[0].second);        // 最高
                        if (n >= 2) selectedSatsBySystem[sys].insert(satElevList[n/2].second);     // 中间
                        if (n >= 3) selectedSatsBySystem[sys].insert(satElevList[n-1].second);     // 最低
                        
                        if (DEBUG) {
                            std::cout << "\nSystem " << sys << " selected satellites:" << std::endl;
                            for (const SatID& sat : selectedSatsBySystem[sys]) {
                                double elev = satElevData.at(sat);  // 已经是度数，不需要转换
                                std::cout << "  " << sat.toString() << " - Elevation: " << std::fixed << std::setprecision(2) << elev << " deg" << std::endl;
                            }
                        }
                    }
                    
                    if (DEBUG) {
                        std::cout << "\nPress Enter to continue processing all epochs..." << std::endl;
                        std::cin.get();
                    }
                }
                
                // 计算电离层延迟
                std::map<SatID, double> ionoDelayMap;
                try {
                    std::map<std::string, std::set<std::string>> systypes = readObsRover.getSystemTypes();
                    ionoDelayMap = ionoDelay(xyz, epoch, satElevData, satAzimData, navStore, &systypes);
                } catch (const std::exception& e) {
                    std::cerr << "Ionosphere delay calculation error: " << e.what() << std::endl;
                }
                
                // 计算对流层延迟（相对湿度50%）
                std::map<SatID, double> tropDelayMap = tropDelay(xyz, satElevData, 0.5);
                
                // 存储筛选卫星的数据（computeElevAzim返回的是度数）
                for (const auto& sysEntry : selectedSatsBySystem) {
                    for (const SatID& sat : sysEntry.second) {
                        if (ionoDelayMap.count(sat) && tropDelayMap.count(sat) && 
                            satElevData.count(sat) && satAzimData.count(sat)) {
                            satDelayData[sat].push_back({
                                epoch,
                                ionoDelayMap[sat],
                                tropDelayMap[sat],
                                satElevData[sat],  // 已经是度数，不需要转换
                                satAzimData[sat]   // 已经是度数，不需要转换
                            });
                        }
                    }
                }
                
                epochCount++;
            }
            
            epochCount++;
        }
    } catch (const EndOfFile& e) {
        std::cout << "\nEnd of observation file reached." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error processing observations: " << e.what() << std::endl;
        return -1;
    }
    
    roverObsStream.close();
    
    // 输出数据
    std::string outputPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/atmospheric/";
    writeAtmosphericData(satDelayData, outputPath);
    
    std::cout << "\n=== Test completed ===" << std::endl;
    std::cout << "Total epochs processed: " << epochCount << std::endl;
    std::cout << "Total satellites selected: " << satDelayData.size() << std::endl;
    
    return 0;
}