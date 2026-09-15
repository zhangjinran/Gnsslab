//-------------------
// exam3.2 - 工厂函数实际用途测试
// 调试版本：添加详细日志输出
//-------------------

#include <string>
#include <fstream>
#include <iostream>
#include <cstring>
#include <set>
#include <cmath>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <numeric>
#include <filesystem>
#include "GnssStruct.h"
#include "TimeConvert.h"
#include "GnssFunc.h"
#include "RinexNavStore.hpp"
#include "RinexObsReader.h"
#include "SPPIFCode.h"
#include "NavEphGPS.hpp"
#include "NavEphBDS.hpp"
#include "NavEphGLONASS.hpp"
#include "NavEphGalileo.hpp"
#include "NavEphQZSS.hpp"
#include "NavEphIRNSS.hpp"
#include "NavEphBase.hpp"
#include "OrbitExporter.h"

extern void ensureNavEphRegistered();
using namespace std;

#define DEBUG_MODE true

void logInfo(const string& msg) {
    if (DEBUG_MODE) {
        cout << "[INFO] " << msg << endl;
    }
}

void logDebug(const string& msg) {
    if (DEBUG_MODE) {
        cout << "[DEBUG] " << msg << endl;
    }
}

void logSuccess(const string& msg) {
    cout << "[SUCCESS] " << msg << endl;
}

void logError(const string& msg) {
    cerr << "[ERROR] " << msg << endl;
}

void compare(SatID sat, CommonTime epoch,SP3Store& sp3Store,RinexNavStore& navStore) {
    Xvt xvtsp3 = sp3Store.getXvt(sat, epoch);
    Xvt xvt = navStore.getXvt(sat, epoch);

    cout<<"********************************************"<<endl;
    cout<<"satellite "<<sat<<endl;
    cout<<"epoch "<<epoch<<endl;
    cout << "======================" << endl;
    cout << "Satellite Position norm" << endl;
    cout << "X = " << xvt.getPos()(0) << endl;
    cout << "Y = " << xvt.getPos()(1) << endl;
    cout << "Z = " << xvt.getPos()(2) << endl;

    cout << "Norm = "
         << xvt.getPos().norm()
         << endl;
    cout << "SP3 Position norm" << endl;
    cout << "X = " << xvtsp3.getPos()(0) << endl;
    cout << "Y = " << xvtsp3.getPos()(1) << endl;
    cout << "Z = " << xvtsp3.getPos()(2) << endl;
    cout << "Norm = "
         << xvtsp3.getPos().norm()
         << endl;

    cout<<"======================"<<endl;
    cout<<"diff between nav and sp3"<<endl;
    cout<<"X = "<<xvt.getPos()(0)-xvtsp3.getPos()(0)<<endl;
    cout<<"Y = "<<xvt.getPos()(1)-xvtsp3.getPos()(1)<<endl;
    cout<<"Z = "<<xvt.getPos()(2)-xvtsp3.getPos()(2)<<endl;
    cout<<"Norm = "<<(xvt.getPos()-xvtsp3.getPos()).norm()<<endl;

}

void irnssSelfConsistencyCheck(RinexNavStore& navStore, const SatID& sat,
                               const CommonTime& toe,
                               const std::filesystem::path& outputPath) {
    cout << "\n===== IRNSS Self-Consistency Check for " << sat << " =====" << endl;
    
    const double interval = 300.0; // 300秒间隔
    const double halfWindow = 30 * 60; // 前后30分钟
    const int numPoints = static_cast<int>(2 * halfWindow / interval) + 1;
    
    std::filesystem::create_directories(outputPath.parent_path());
    ofstream outFile(outputPath);
    if (!outFile) {
        cerr << "[ERROR] Cannot open output file: " << outputPath.string() << endl;
        return;
    }
    
    outFile << fixed << setprecision(6);
    outFile << "# IRNSS Self-Consistency Check Report" << endl;
    outFile << "# Satellite: " << sat << endl;
    outFile << "# Reference Time (toe): " << toe << endl;
    outFile << "# Interval: " << interval << " s" << endl;
    outFile << "# Window: ±" << halfWindow/60 << " min" << endl;
    outFile << "# Columns: time, x(m), y(m), z(m), radius(m), dr_prev(km)" << endl;
    
    vector<Eigen::Vector3d> positions;
    vector<double> radii;
    vector<CommonTime> times;
    
    for (int i = 0; i < numPoints; ++i) {
        double offset = (i - (numPoints-1)/2.0) * interval;
        CommonTime currentTime = toe;
        currentTime.set(currentTime.m_day, currentTime.m_sod + offset);
        
        try {
            Xvt xvt = navStore.getXvt(sat, currentTime);
            Eigen::Vector3d pos = xvt.getPos();
            double radius = pos.norm();
            
            positions.push_back(pos);
            radii.push_back(radius);
            times.push_back(currentTime);
            
            double dr_prev = 0.0;
            if (i > 0) {
                dr_prev = (pos - positions[i-1]).norm() / 1000.0; // km
            }
            
            outFile << currentTime << ", "
                    << pos(0) << ", "
                    << pos(1) << ", "
                    << pos(2) << ", "
                    << radius << ", "
                    << dr_prev << endl;
                    
            cout << "[" << setw(3) << i << "] " << currentTime 
                 << " | r = " << fixed << setprecision(3) << radius/1000.0 << " km";
            if (i > 0) {
                cout << " | dr = " << fixed << setprecision(3) << dr_prev << " km";
            }
            cout << endl;
            
        } catch (const exception& e) {
            cerr << "[WARNING] Failed to compute position at " << currentTime << ": " << e.what() << endl;
            outFile << currentTime << ", NaN, NaN, NaN, NaN, NaN" << endl;
        }
    }
    
    outFile.close();
    cout << "\n[INFO] Output written to: " << outputPath.string() << endl;
    
    // 分析结果
    if (radii.size() >= 2) {
        double minRadius = *min_element(radii.begin(), radii.end());
        double maxRadius = *max_element(radii.begin(), radii.end());
        double avgRadius = accumulate(radii.begin(), radii.end(), 0.0) / radii.size();
        
        cout << "\n===== Analysis Results =====" << endl;
        cout << "Average radius: " << fixed << setprecision(3) << avgRadius/1000.0 << " km" << endl;
        cout << "Radius variation: " << fixed << setprecision(3) << (maxRadius - minRadius)/1000.0 << " km" << endl;
        
        // 检查轨道连续性
        double maxDr = 0.0;
        for (size_t i = 1; i < positions.size(); ++i) {
            double dr = (positions[i] - positions[i-1]).norm() / 1000.0; // km
            maxDr = max(maxDr, dr);
        }
        cout << "Maximum position change (dr): " << fixed << setprecision(3) << maxDr << " km" << endl;
        
        // 计算速度估计
        double avgSpeed = 0.0;
        int validSpeedCount = 0;
        for (size_t i = 1; i < positions.size(); ++i) {
            double dr = (positions[i] - positions[i-1]).norm(); // m
            avgSpeed += dr / interval; // m/s
            validSpeedCount++;
        }
        if (validSpeedCount > 0) {
            avgSpeed /= validSpeedCount;
            cout << "Average speed: " << fixed << setprecision(3) << avgSpeed/1000.0 << " km/s" << endl;
        }
        
        // 对称性检查（toe附近）
        if (radii.size() >= 3) {
            int midIdx = radii.size() / 2;
            if (midIdx > 0 && midIdx < radii.size() - 1) {
                double dr_forward = (positions[midIdx+1] - positions[midIdx]).norm() / 1000.0;
                double dr_backward = (positions[midIdx] - positions[midIdx-1]).norm() / 1000.0;
                double symmetry_ratio = min(dr_forward, dr_backward) / max(dr_forward, dr_backward);
                
                cout << "\n===== Symmetry Check at toe =====" << endl;
                cout << "dr(toe+300) - dr(toe): " << fixed << setprecision(3) << dr_forward << " km" << endl;
                cout << "dr(toe) - dr(toe-300): " << fixed << setprecision(3) << dr_backward << " km" << endl;
                cout << "Symmetry ratio: " << fixed << setprecision(3) << symmetry_ratio << endl;
                
                if (symmetry_ratio > 0.8) {
                    cout << "[PASS] Symmetry check passed" << endl;
                } else {
                    cout << "[WARNING] Symmetry check failed - possible tk issue" << endl;
                }
            }
        }
        
        // 合理性评估
        cout << "\n===== Validity Assessment =====" << endl;
        
        // 轨道半径检查 (IRNSS GEO/IGSO ~42164 km)
        const double expectedRadius = 42164000.0; // m
        const double radiusTolerance = 5000000.0; // ±5000 km
        
        if (fabs(avgRadius - expectedRadius) < radiusTolerance) {
            cout << "[PASS] Orbit radius is reasonable" << endl;
        } else {
            cout << "[FAIL] Orbit radius is outside expected range! Expected: ~42164 km, Got: " 
                 << avgRadius/1000.0 << " km" << endl;
        }
        
        // 轨道连续性检查
        if (maxDr < 5000.0) { // < 5000 km
            cout << "[PASS] Orbit continuity is good" << endl;
        } else if (maxDr < 10000.0) { // < 10000 km
            cout << "[WARNING] Orbit has moderate discontinuity: " << maxDr << " km" << endl;
        } else {
            cout << "[FAIL] Orbit has severe discontinuity! dr = " << maxDr << " km" << endl;
        }
        
        // 速度检查 (GEO/IGSO ~2-4 km/s)
        if (avgSpeed > 0 && avgSpeed < 6000.0) { // < 6 km/s
            cout << "[PASS] Speed is reasonable: " << avgSpeed/1000.0 << " km/s" << endl;
        } else {
            cout << "[FAIL] Speed is outside expected range! Got: " << avgSpeed/1000.0 << " km/s" << endl;
        }
    }
    
    cout << "\n===== Self-Consistency Check Complete =====" << endl;
}

int main()
{
    namespace fs = std::filesystem;
    const fs::path dataDir = fs::current_path() / "data";
    const fs::path selfCheckOutput =
        fs::current_path() / "outputs" / "read_rinex_data" /
        "irnss_self_consistency_check.txt";

    string obsFile =
        (dataDir / "WUH200CHN_R_20250010000_01D_30S_MO.rnx").string();

    RinexObsReader obsReader;
    obsReader.loadFile(obsFile);
    ObsData obsData=obsReader.parseRinexObs();
    ObsDataStaticSum obsDataStaticSum;
    obsReader.static_Obs(obsData, &obsDataStaticSum);
    cout<<"obsDataStaticSum "<<obsDataStaticSum<<endl;
    //cout<<"obsData "<<obsData<<endl;

    RinexNavStore navStore;

    string navFile =
        (dataDir / "BRDC00IGS_R_20250010000_01D_MN.rnx").string();

    SP3Store sp3Store;
    string sp3File =
        (dataDir / "WUM0MGXFIN_20250010000_01D_05M_ORB.SP3").string();
    string sp3File2 =
        (dataDir / "COD0MGXFIN_20250010000_01D_05M_ORB.SP3").string();
    sp3Store.loadSP3File(sp3File);
    sp3Store.loadSP3File(sp3File2);



    if (!navStore.loadFile(navFile))
    {
        cerr << "load nav failed" << endl;
        return -1;
    }


    cout << "===== Loaded Systems =====" << endl;

    for (auto& sys : navStore.getSystems())
    {
        cout << sys << endl;
    }

    cout << endl;

    SatID sat("G01");

    CivilTime ct(
        2025, 1, 1,
        0, 0, 0,
        TimeSystem::GPS);

    CommonTime epoch = CivilTime2CommonTime(ct);





    NavEphGPS eph =
        navStore.findGPSEph(sat, epoch);

    compare(sat, epoch, sp3Store, navStore);
    SatID sat_bds("C06");

    CivilTime ct_bds(
        2025, 1, 1,
        5, 0, 0,
        TimeSystem::GPS);

    CommonTime epoch_bds =
        CivilTime2CommonTime(ct_bds);

    compare(sat_bds, epoch_bds, sp3Store, navStore);

    SatID sat_GLO("R01");

    CivilTime ct_GLO(
        2025, 1, 1,
        0, 0, 0,
        TimeSystem::GPS);

    CommonTime epoch_GLO =
        CivilTime2CommonTime(ct_GLO);

    compare(sat_GLO, epoch_GLO, sp3Store, navStore);

    SatID sat_Gal("E02");

    CivilTime ct_Gal(
        2025, 1, 1,
        0, 0, 0,
        TimeSystem::GPS);

    CommonTime epoch_Gal =
        CivilTime2CommonTime(ct_Gal);

    compare(sat_Gal, epoch_Gal, sp3Store, navStore);

    SatID sat_QZSS("J02");

    CivilTime ct_QZSS(
        2025, 1, 1,
        0, 0, 0,
        TimeSystem::GPS);

    CommonTime epoch_QZSS =
        CivilTime2CommonTime(ct_QZSS);

    compare(sat_QZSS, epoch_QZSS, sp3Store, navStore);

    SatID sat_IRNSS("I02");

    CivilTime ct_IRNSS(
        2025, 1, 1,
        0, 0, 0,
        TimeSystem::GPS);

    CommonTime epoch_IRNSS =
        CivilTime2CommonTime(ct_IRNSS);

// IRNSS Self-Consistency Check (no SP3 available for IRNSS)
    try {
        NavEphIRNSS irnssEph = navStore.findIRNSSEph(sat_IRNSS, epoch_IRNSS);
        
        CommonTime toeTime = epoch_IRNSS;
        if (irnssEph.CivilToc.year > 0) {
            toeTime = CivilTime2CommonTime(irnssEph.CivilToc);
        }
        
        irnssSelfConsistencyCheck(navStore, sat_IRNSS, toeTime, selfCheckOutput);
    } catch (const exception& e) {
        cerr << "[WARNING] Cannot perform IRNSS self-consistency check: " << e.what() << endl;
    }

    // ==================== GNSS 星历验证实验 ====================
    cout << "\n===== GNSS Orbit Validation Experiment =====" << endl;
    
    // 定义需要验证的卫星
    std::vector<SatID> testSats = {

        SatID("C01"),  // BDS GEO
        SatID("C02"),  // BDS GEO
        SatID("C03"),  // BDS GEO
        SatID("C04"),  // BDS GEO
        SatID("C05"),  // BDS GEO
        SatID("C06"),  // BDS IGSO
        SatID("C07"),  // BDS IGSO
        SatID("C08"),  // BDS IGSO
        SatID("C09"),  // BDS IGSO
        SatID("C10"),  // BDS IGSO
        SatID("C11"),  // BDS MEO
        SatID("C12"),  // BDS MEO
        SatID("C13"),  // BDS MEO
        SatID("C14"),  // BDS MEO
        SatID("C19"),// BDS MEO
        SatID("C46"),
        SatID("C56"),// BDS MEO
        SatID("C58"),// BDS MEO
        SatID("C60"),// BDS MEO
        SatID("C61"),
        SatID("C62"),
    };

    CivilTime ct_test(
        2025, 1, 1,
        0, 0, 0,
        TimeSystem::GPS);

    CommonTime epoch_test = CivilTime2CommonTime(ct_test);
    
    // 轨道计算参数
    double durationHours = 24.0;
    double intervalSeconds = 300.0;
    
    // 计算并导出轨道数据
    gnss::OrbitDataMap orbitDataMap;
    
    for (const SatID& sat : testSats) {
        cout << "[INFO] Processing satellite: " << sat << endl;
        try {
            std::vector<gnss::OrbitPoint> points = 
                gnss::OrbitExporter::computeOrbitPoints(navStore, sat, epoch_test, durationHours, intervalSeconds);
            
            if (!points.empty()) {
                orbitDataMap[sat] = points;
                
                // 导出单颗卫星轨道
                gnss::OrbitExporter::exportSingleSatOrbit(points, sat, "");
                cout << "[SUCCESS] Orbit data exported for " << sat << endl;
            } else {
                cout << "[WARNING] No orbit data for " << sat << endl;
            }
        } catch (const std::exception& e) {
            cout << "[ERROR] Failed to process " << sat << ": " << e.what() << endl;
        }
    }
    
    // 导出综合分析数据
    if (!orbitDataMap.empty()) {
        gnss::OrbitExporter::exportOrbitData(orbitDataMap, "");
        gnss::OrbitExporter::exportRadiusAnalysis(orbitDataMap, "");
        gnss::OrbitExporter::exportVelocityAnalysis(orbitDataMap, &sp3Store, "");
        gnss::OrbitExporter::exportGroundTrack(orbitDataMap, "");
        
        // Export combined broadcast and SP3 precise ephemeris
        gnss::OrbitExporter::exportCombinedOrbitData(orbitDataMap, &sp3Store, "");
        cout << "\n[SUCCESS] Combined orbit data (Broadcast + SP3) exported to gnss_draw/data/orbit/" << endl;
        
        cout << "\n[SUCCESS] All orbit analysis data exported to gnss_draw/data/orbit/" << endl;
    }
    
    cout << "\n===== Orbit Validation Experiment Complete =====" << endl;

    return 0;
}
