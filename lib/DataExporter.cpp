#include "DataExporter.h"
#include "TimeConvert.h"
#include "CoordConvert.h"
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdlib>

namespace gnss {

std::string DataExporter::getOutputBasePath() {
    return std::string(getenv("HOME")) + 
           "/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/";
}

bool DataExporter::ensureDirectoryExists(const std::string& path) {
#ifdef _WIN32
    return CreateDirectory(path.c_str(), NULL) || 
           GetLastError() == ERROR_ALREADY_EXISTS;
#else
    size_t pos = 0;
    std::string temp = path;
    while ((pos = temp.find('/', pos + 1)) != std::string::npos) {
        std::string dir = temp.substr(0, pos);
        mkdir(dir.c_str(), 0755);
    }
    return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

std::string DataExporter::getTimeSystemOutputPath(const std::string& subFolder) {
    std::string path = getOutputBasePath() + "time_system/";
    if (!subFolder.empty()) {
        path += subFolder + "/";
    }
    ensureDirectoryExists(path);
    return path;
}

std::string DataExporter::getCoordSystemOutputPath(const std::string& subFolder) {
    std::string path = getOutputBasePath() + "coord_system/";
    if (!subFolder.empty()) {
        path += subFolder + "/";
    }
    ensureDirectoryExists(path);
    return path;
}

std::string DataExporter::getFigureOutputPath(const std::string& subFolder) {
    std::string path = std::string(getenv("HOME")) + 
                       "/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/figure/";
    if (!subFolder.empty()) {
        path += subFolder + "/";
    }
    ensureDirectoryExists(path);
    return path;
}

bool DataExporter::exportTimeConversionData(const CommonTime& gpsTime, 
                                           const std::string& subFolder) {
    std::string path = getTimeSystemOutputPath(subFolder);
    std::ofstream file(path + "time_conversion.txt");
    
    if (!file) return false;
    
    long mjd_int = gpsTime.m_day;
    double sod = gpsTime.m_sod;
    MJD full_mjd ;
    CommonTime2MJD(gpsTime, full_mjd);
    file << "# 时间系统转换结果 (基准: GPS时间)\n";
    file << "# 输入: GPS MJD=" << std::fixed << std::setprecision(10) << full_mjd << "\n";
    file << "#       (MJD整数=" << static_cast<long>(mjd_int) 
         << ", SOD=" << std::fixed << std::setprecision(6) << sod << ")\n";
    file << "系统,周数,SOW,MJD(完整),MJD(整数),SOD\n";
    
    std::vector<TimeSystem> systems = {TimeSystem::GPS, TimeSystem::UTC, 
                                       TimeSystem::BDT, TimeSystem::GAL, 
                                       TimeSystem::GLO, TimeSystem::QZS, 
                                       TimeSystem::IRN};
    
    for (const auto& sys : systems) {
        CommonTime converted = convertTimeSystem(gpsTime, sys);
        long mjd_int = converted.m_day;
        double sod = converted.m_sod;
        MJD full_mjd ;
        CommonTime2MJD(converted, full_mjd);
        int week = 0;
        double sow = sod;
        if (sys != TimeSystem::GLO) {

            WeekSecond* ws = createWeekSecond(sys.system, 0, 0.0);

            if (ws) {
                CommonTime2WeekSecond(converted, *ws);

                week = ws->getWeek();
                sow = ws->getSOW();

                delete ws;
            }
        }
        
        file << sys.toString() << "," 
             << week << "," 
             << std::fixed << std::setprecision(6) << sow << ","
             << std::fixed << std::setprecision(10) << full_mjd << ","
             << mjd_int << ","
             << std::fixed << std::setprecision(6) << sod << "\n";
    }
    
    return true;
}

bool DataExporter::exportTimeSystemParams(const std::string& subFolder) {
    std::string path = getTimeSystemOutputPath(subFolder);
    std::ofstream file(path + "time_system_params.txt");
    
    if (!file) return false;
    
    file << "# 各导航系统周秒参数\n";
    file << "系统,周数范围(位),位掩码,MJD起始历元\n";
    

    std::vector<TimeSystem> params = {
        {TimeSystem::GPS},
        {TimeSystem::BDT},
        {TimeSystem::GAL},
        {TimeSystem::QZS},
        {TimeSystem::IRN},
    };

    
    for (const auto& p : params) {
        WeekSecond* ws = createWeekSecond(p.system, 0, 0.0);
        file << p.toString() << "," << ws->Nbits() << ","
             << "0x" << std::hex << ws->bitmask() << std::dec << ","
             << ws->MJDEpoch() << "\n";
        delete ws;
    }
    
    return true;
}

bool DataExporter::exportTimeConversionErrors(const std::string& subFolder) {
    std::string path = getTimeSystemOutputPath(subFolder);
    std::ofstream file(path + "time_conversion_errors.txt");
    
    if (!file) return false;
    
    file << "# 时间格式转换精度验证 - 以 CommonTime 为核心\n";
    file << "# 测试时间: GPS 日 59644, 秒 43200 \n";
    file << "转换路径,输入值,输出值,往返误差(ns)\n";
    
    // 基准时间
    CommonTime ct_base(59644, 43200.0, TimeSystem::GPS);
    
    // 1. CommonTime -> MJD -> CommonTime
    MJD mjd;
    CommonTime ct_mjd_back;
    CommonTime2MJD(ct_base, mjd);
    MJD2CommonTime(mjd, ct_mjd_back);
    double err_mjd = fabs((ct_base.m_day - ct_mjd_back.m_day) * 86400.0 + 
                          (ct_base.m_sod - ct_mjd_back.m_sod)) * 1e9;
    file << "CommonTime->MJD->CommonTime,"
         << std::fixed << std::setprecision(15) << mjd.mjd << ","
         << std::fixed << std::setprecision(15) << ct_mjd_back.m_day + ct_mjd_back.m_sod/86400.0 << ","
         << std::fixed << std::setprecision(3) << err_mjd << "\n";
    
    // 2. CommonTime -> CivilTime -> CommonTime
    CivilTime civil = CommonTime2CivilTime(ct_base);
    CommonTime ct_civil_back = CivilTime2CommonTime(civil);
    double err_civil = fabs((ct_base.m_day - ct_civil_back.m_day) * 86400.0 + 
                            (ct_base.m_sod - ct_civil_back.m_sod)) * 1e9;
    file << "CommonTime->CivilTime->CommonTime,"
         << civil.year << "/" << civil.month << "/" << civil.day << " " 
         << civil.hour << ":" << civil.minute << ":" << std::fixed << std::setprecision(6) << civil.second << ","
         << ct_civil_back.m_day << " " << std::fixed << std::setprecision(12) << ct_civil_back.m_sod << ","
         << std::fixed << std::setprecision(3) << err_civil << "\n";
    

    // 5. CommonTime -> GPSWeekSecond -> CommonTime
    GPSWeekSecond gps_ws;
    CommonTime2WeekSecond(ct_base, gps_ws);
    CommonTime ct_gpsws_back;
    WeekSecond2CommonTime(gps_ws, ct_gpsws_back);
    double err_gpsws = fabs((ct_base.m_day - ct_gpsws_back.m_day) * 86400.0 + 
                            (ct_base.m_sod - ct_gpsws_back.m_sod)) * 1e9;
    file << "CommonTime->GPSWeekSecond->CommonTime,"
         << "W" << gps_ws.getWeek() << "S" << std::fixed << std::setprecision(6) << gps_ws.getSOW() << ","
         << ct_gpsws_back.m_day << " " << std::fixed << std::setprecision(15) << ct_gpsws_back.m_sod << ","
         << std::fixed << std::setprecision(3) << err_gpsws << "\n";
    
    // 6. CommonTime -> BDTWeekSecond -> CommonTime
    BDTWeekSecond bdt_ws;
    CommonTime ct_bdtws = convertTimeSystem(ct_base, TimeSystem::BDT);
    CommonTime2WeekSecond(ct_bdtws, bdt_ws);
    CommonTime ct_bdtws_back;
    WeekSecond2CommonTime(bdt_ws, ct_bdtws_back);
    ct_bdtws_back = convertTimeSystem(ct_bdtws_back, TimeSystem::GPS);
    double err_bdtws = fabs((ct_base.m_day - ct_bdtws_back.m_day) * 86400.0 + 
                            (ct_base.m_sod - ct_bdtws_back.m_sod)) * 1e9;
    file << "CommonTime->BDTWeekSecond->CommonTime,"
         << "W" << bdt_ws.getWeek() << "S" << std::fixed << std::setprecision(6) << bdt_ws.getSOW() << ","
         << ct_bdtws_back.m_day << " " << std::fixed << std::setprecision(15) << ct_bdtws_back.m_sod << ","
         << std::fixed << std::setprecision(3) << err_bdtws << "\n";
    return true;
}

bool DataExporter::exportCoordConversionData(const XYZ& xyz,
                                            const std::vector<ReferenceFrame*>& frames,
                                            const std::string& subFolder) {
    std::string path = getCoordSystemOutputPath(subFolder);
    std::ofstream file(path + "coord_conversion.txt");
    
    if (!file) return false;
    
    file << "# 坐标转换结果 (XYZ -> BLH)\n";
    file << "# 输入点: X=" << std::fixed << std::setprecision(2) << xyz.X() 
         << ", Y=" << std::fixed << std::setprecision(2) << xyz.Y() 
         << ", Z=" << std::fixed << std::setprecision(2) << xyz.Z() << "\n";
    file << "系统,纬度(度),经度(度),高度(米),纬度差异(秒),高度差异(米)\n";
    
    WGS84 wgs84;
    BLH blh_wgs84 = xyz2blh(xyz, wgs84);
    double ref_lat = blh_wgs84.B();
    double ref_h = blh_wgs84.H();
    
    for (const auto& frame : frames) {
        BLH blh = xyz2blh(xyz, *frame);
        double lat_diff = rad2deg(blh.B() - ref_lat) * 3600;
        double h_diff = blh.H() - ref_h;
        
        file << frame->getName() << ","
             << std::fixed << std::setprecision(8) << rad2deg(blh.B()) << ","
             << std::fixed << std::setprecision(8) << rad2deg(blh.L()) << ","
             << std::fixed << std::setprecision(4) << blh.H() << ","
             << std::fixed << std::setprecision(6) << lat_diff << ","
             << std::fixed << std::setprecision(6) << h_diff << "\n";
    }
    
    return true;
}

bool DataExporter::exportEllipsoidParams(const std::string& subFolder) {
    std::string path = getCoordSystemOutputPath(subFolder);
    std::ofstream file(path + "ellipsoid_params.txt");
    
    if (!file) return false;
    
    file << "# 各导航系统椭球参数\n";
    file << "系统,长半轴a(米),扁率f,GM(×10^14 m³/s²),J2(×10^-3)\n";
    
    WGS84 wgs84;
    GPSEllipsoid gps;
    BDSEllipsoid bds;
    PZ90 pz90;
    Galileo galileo;
    GPSEllipsoid qzss;
    IRNSS irnss;
    
    auto writeParams = [&](const std::string& name, const ReferenceFrame& frame) {
        file << name << ","
             << std::fixed << std::setprecision(1) << frame.getA() << ","
             << std::fixed << std::setprecision(9) << frame.getF() << ","
             << std::fixed << std::setprecision(7) << frame.getGM() / 1e14 << "\n";
    };
    
    writeParams("WGS84", wgs84);
    writeParams("GPS", gps);
    writeParams("BDS", bds);
    writeParams("GLONASS", pz90);
    writeParams("Galileo", galileo);
    writeParams("QZSS", qzss);
    writeParams("IRNSS", irnss);
    
    return true;
}

bool DataExporter::exportCoordConversionErrors(const XYZ& testPoint,
                                              const std::string& subFolder) {
    std::string path = getCoordSystemOutputPath(subFolder);
    std::ofstream file(path + "coord_conversion_errors.txt");
    
    if (!file) return false;
    
    file << "# 坐标转换精度验证 (XYZ -> BLH -> XYZ)\n";
    file << "框架,输入X,输入Y,输入Z,输出X,输出Y,输出Z,误差(m)\n";
    
    std::vector<std::unique_ptr<ReferenceFrame>> frames;
    frames.push_back(std::make_unique<WGS84>());
    frames.push_back(std::make_unique<GPSEllipsoid>());
    frames.push_back(std::make_unique<BDSEllipsoid>());
    frames.push_back(std::make_unique<PZ90>());
    frames.push_back(std::make_unique<Galileo>());
    
    for (const auto& frame : frames) {
        BLH blh = xyz2blh(testPoint, *frame);
        XYZ xyz_back = blh2xyz(blh, *frame);
        
        double error = sqrt(pow(testPoint.X() - xyz_back.X(), 2) +
                           pow(testPoint.Y() - xyz_back.Y(), 2) +
                           pow(testPoint.Z() - xyz_back.Z(), 2));
        
        file << frame->getName() << ","
             << std::fixed << std::setprecision(2) << testPoint.X() << ","
             << std::fixed << std::setprecision(2) << testPoint.Y() << ","
             << std::fixed << std::setprecision(2) << testPoint.Z() << ","
             << std::fixed << std::setprecision(2) << xyz_back.X() << ","
             << std::fixed << std::setprecision(2) << xyz_back.Y() << ","
             << std::fixed << std::setprecision(2) << xyz_back.Z() << ","
             << std::fixed << std::setprecision(12) << error << "\n";
    }
    
    return true;
}

bool DataExporter::exportFrameDifferenceMatrix(const XYZ& testPoint,
                                              const std::string& subFolder) {
    std::string path = getCoordSystemOutputPath(subFolder);
    std::ofstream file(path + "frame_difference_matrix.txt");
    
    if (!file) return false;
    
    file << "# 参考框架间坐标差异矩阵（高度差异，单位：米）\n";
    
    std::vector<std::pair<std::string, std::unique_ptr<ReferenceFrame>>> frames;
    frames.emplace_back("WGS84", std::make_unique<WGS84>());
    frames.emplace_back("GPS", std::make_unique<GPSEllipsoid>());
    frames.emplace_back("BDS", std::make_unique<BDSEllipsoid>());
    frames.emplace_back("GLONASS", std::make_unique<PZ90>());
    frames.emplace_back("Galileo", std::make_unique<Galileo>());
    frames.emplace_back("QZSS", std::make_unique<GPSEllipsoid>());
    frames.emplace_back("IRNSS", std::make_unique<IRNSS>());
    
    file << ",";
    for (const auto& f : frames) {
        file << f.first << ",";
    }
    file << "\n";
    
    for (size_t i = 0; i < frames.size(); ++i) {
        file << frames[i].first << ",";
        BLH blh_i = xyz2blh(testPoint, *frames[i].second);
        
        for (size_t j = 0; j < frames.size(); ++j) {
            BLH blh_j = xyz2blh(testPoint, *frames[j].second);
            double diff = blh_i.H() - blh_j.H();
            file << std::fixed << std::setprecision(6) << diff << ",";
        }
        file << "\n";
    }
    
    return true;
}

bool DataExporter::exportSatelliteSkyplotData(const std::vector<Satellite>& satellites,
                                             const std::string& subFolder) {
    std::string path = getFigureOutputPath(subFolder);
    std::ofstream file(path + "satellite_data.txt");
    
    if (!file) return false;
    
    file << "# 卫星天空图数据\n";
    file << "卫星ID,系统,方位角(度),仰角(度)\n";
    
    for (const auto& sat : satellites) {
        file << sat.id << "," << sat.system << ","
             << std::fixed << std::setprecision(2) << sat.azimuth << ","
             << std::fixed << std::setprecision(2) << sat.elevation << "\n";
    }
    
    return true;
}

bool DataExporter::exportENUConversionData(const XYZ& targetXYZ,
                                          const XYZ& refXYZ,
                                          const std::vector<ReferenceFrame*>& frames,
                                          const std::string& subFolder) {
    std::string path = getCoordSystemOutputPath(subFolder);
    std::ofstream file(path + "enu_conversion.txt");
    
    if (!file) return false;
    
    file << "# ENU坐标转换结果\n";
    file << "# 参考点(原点) XYZ: (" << std::fixed << std::setprecision(2) 
         << refXYZ.X() << ", " << refXYZ.Y() << ", " << refXYZ.Z() << ")\n";
    file << "# 目标点 XYZ: (" << std::fixed << std::setprecision(2) 
         << targetXYZ.X() << ", " << targetXYZ.Y() << ", " << targetXYZ.Z() << ")\n";
    file << "系统,参考点纬度(度),参考点经度(度),参考点高度(米),E(米),N(米),U(米),方位角(度),仰角(度)\n";
    
    for (const auto& frame : frames) {
        BLH refBLH = xyz2blh(refXYZ, *frame);
        XYZ enu = blh2ENU(xyz2blh(targetXYZ, *frame), *frame, refXYZ);
        
        double az = azimuth(refXYZ, targetXYZ);
        double el = elevation(refXYZ, targetXYZ);
        
        file << frame->getName() << ","
             << std::fixed << std::setprecision(8) << rad2deg(refBLH.B()) << ","
             << std::fixed << std::setprecision(8) << rad2deg(refBLH.L()) << ","
             << std::fixed << std::setprecision(4) << refBLH.H() << ","
             << std::fixed << std::setprecision(4) << enu.X() << ","
             << std::fixed << std::setprecision(4) << enu.Y() << ","
             << std::fixed << std::setprecision(4) << enu.Z() << ","
             << std::fixed << std::setprecision(4) << az << ","
             << std::fixed << std::setprecision(4) << el << "\n";
    }
    
    return true;
}

} // namespace gnss