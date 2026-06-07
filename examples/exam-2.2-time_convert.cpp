/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *
 * Author: shoujian zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * Description:
 *  时间格式转换演示程序
 *  展示各种时间格式和时间系统的转换方法
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <stdexcept>

#include "Exception.h"
#include "TimeStruct.h"
#include "TimeConvert.h"
#include "Time2020Convert.h"
#include "DataExporter.h"

// 调试模式开关
#define DEBUG_MODE 1

using namespace std;

int main() {
    cout << "==========================================" << endl;
    cout << "         时间格式转换演示程序" << endl;
    cout << "==========================================" << endl;

    try {
        // --------------------------
        // 1. YMD 转 JD（儒略日）
        // --------------------------
        cout << "\n【1. YMD → JD（儒略日）】" << endl;
        int yy = 2025, month = 1, day = 1;
        double jd = convertYMD2JD(yy, month, day);
        cout << fixed << setprecision(6);
        cout << "  输入: " << yy << "/" << month << "/" << day << endl;
        cout << "  输出: JD = " << jd << endl;

        // --------------------------
        // 2. CivilTime 转 CommonTime
        // --------------------------
        cout << "\n【2. CivilTime → CommonTime】" << endl;
        CivilTime civilTime(2025, 1, 4, 9, 0, 0.0, TimeSystem::UTC);
        CommonTime commonTime = CivilTime2CommonTime(civilTime);
        cout << "  输入: " << civilTime << endl;
        cout << "  输出: " << commonTime << endl;

        // --------------------------
        // 3. CommonTime 转 JulianDate
        // --------------------------
        cout << "\n【3. CommonTime → JulianDate】" << endl;
        JulianDate julianDate = CommonTime2JulianDate(commonTime);
        cout << "  输入: " << commonTime << endl;
        cout << "  输出: " << fixed << julianDate << endl;

        // --------------------------
        // 4. 时间系统转换（GPS → UTC）
        // --------------------------
        cout << "\n【4. 时间系统转换（GPS → UTC）】" << endl;
        CommonTime gpsTime(49169, 0.0, TimeSystem::GPS);
        CommonTime utcTime = convertTimeSystem(gpsTime, TimeSystem::UTC);
        cout << "  输入: " << gpsTime << endl;
        cout << "  输出: " << utcTime << endl;

        // --------------------------
        // 5. BDT周秒格式转换
        // --------------------------
        cout << "\n【5. BDT周秒格式 → CommonTime】" << endl;
        BDTWeekSecond bdtWeekSecond(2, 800);
        CommonTime ctBDT;
        WeekSecond2CommonTime(bdtWeekSecond, ctBDT);
        long mjd_day;
        double sod;
        TimeSystem ts_out;
        ctBDT.get(mjd_day, sod, ts_out);
        cout << "  输入: BDT周=" << bdtWeekSecond.week << ", SOW=" << bdtWeekSecond.sow << endl;
        cout << "  输出: MJD=" << mjd_day << ", SOD=" << fixed << sod << endl;

        // --------------------------
        // 6. CommonTime2020 转换
        // --------------------------
        cout << "\n【6. CommonTime2020 转换】" << endl;
        CommonTime ct2020(0);
        JulianDate jd2020 = CommonTime20202JulianDate(ct2020);
        CivilTime civil2020 = CommonTime20202CivilTime(ct2020);
        CommonTime ct_mjd = CommonTime20202CommonTime(ct2020);
        
        cout << "  输入: CommonTime2020(0)" << endl;
        cout << "  → JulianDate: " << fixed << jd2020 << endl;
        cout << "  → CivilTime: " << civil2020 << endl;
        cout << "  → CommonTime: " << ct_mjd << endl;

        // --------------------------
        // 7. JD 转 YMD（反向转换验证）
        // --------------------------
        cout << "\n【7. JD → YMD（验证）】" << endl;
        int year_out, month_out, day_out;
        convertJD2YMD(jd, year_out, month_out, day_out);
        cout << "  输入: JD = " << fixed << jd << endl;
        cout << "  输出: " << year_out << "/" << month_out << "/" << day_out << endl;

        // --------------------------
        // 8. HMS 转 SOD
        // --------------------------
        cout << "\n【8. HMS → SOD】" << endl;
        double sod_out = convertHMS2SOD(9, 30, 45.5);
        cout << "  输入: 9:30:45.5" << endl;
        cout << "  输出: SOD = " << fixed << sod_out << endl;

        // --------------------------
        // 9. SOD 转 HMS（反向转换验证）
        // --------------------------
        cout << "\n【9. SOD → HMS（验证）】" << endl;
        int hh, mm;
        double sec;
        convertSOD2HMS(sod_out, hh, mm, sec);
        cout << "  输入: SOD = " << fixed << sod_out << endl;
        cout << "  输出: " << hh << ":" << mm << ":" << fixed << sec << endl;

        // --------------------------
        // 10. YDSTime 转换
        // --------------------------
        cout << "\n【10. YDSTime ↔ CommonTime】" << endl;
        YDSTime ydsTime(2025, 100, 3600.0);  // 2025年第100天，1小时
        CommonTime ctYDS = YDSTime2CommonTime(ydsTime);
        YDSTime ydsBack = CommonTime2YDSTime(ctYDS);
        cout << "  输入: " << ydsTime << endl;
        cout << "  → CommonTime: " << ctYDS << endl;
        cout << "  → YDSTime(还原): " << ydsBack << endl;

        // --------------------------
        // 11. MJD 转换
        // --------------------------
        cout << "\n【11. MJD ↔ CommonTime】" << endl;
        MJD mjd(58849.5);
        CommonTime ctMJD;
        MJD2CommonTime(mjd, ctMJD);
        MJD mjdBack;
        CommonTime2MJD(ctMJD, mjdBack);
        cout << "  输入: MJD = " << mjd.mjd << endl;
        cout << "  → CommonTime: " << ctMJD << endl;
        cout << "  → MJD(还原): " << mjdBack.mjd << endl;

        // --------------------------
        // 12. WeekSecond 工厂函数测试（GPS）
        // --------------------------
        cout << "\n【12. WeekSecond 工厂函数测试（GPS）】" << endl;
        WeekSecond* gpsWS = createWeekSecond(TimeSystem::GPS, 2200, 43200.0);
        cout << "  创建 GPS WeekSecond: week=" << gpsWS->getWeek() 
             << ", sow=" << fixed << gpsWS->getSOW() 
             << ", system=" << gpsWS->timeSystem.toString() << endl;
        cout << "  GPS Nbits=" << gpsWS->Nbits() 
             << ", bitmask=0x" << hex << gpsWS->bitmask() << dec 
             << ", MJDEpoch=" << gpsWS->MJDEpoch() << endl;
        
        CommonTime ctGPS;
        WeekSecond2CommonTime(*gpsWS, ctGPS);
        cout << "  → CommonTime: " << ctGPS << endl;
        
        // 反向转换验证
        GPSWeekSecond gpsBack;
        CommonTime2WeekSecond(ctGPS, gpsBack);
        cout << "  → 还原 GPS WeekSecond: week=" << gpsBack.getWeek() 
             << ", sow=" << fixed << gpsBack.getSOW() << endl;
        
        delete gpsWS;

        // --------------------------
        // 13. WeekSecond 工厂函数测试（BDS）
        // --------------------------
        cout << "\n【13. WeekSecond 工厂函数测试（BDS）】" << endl;
        WeekSecond* bdsWS = createWeekSecond(TimeSystem::BDT, 1000, 86399.0);
        cout << "  创建 BDS WeekSecond: week=" << bdsWS->getWeek() 
             << ", sow=" << fixed << bdsWS->getSOW() 
             << ", system=" << bdsWS->timeSystem.toString() << endl;
        cout << "  BDS Nbits=" << bdsWS->Nbits() 
             << ", bitmask=0x" << hex << bdsWS->bitmask() << dec 
             << ", MJDEpoch=" << bdsWS->MJDEpoch() << endl;
        
        CommonTime ctBDS;
        WeekSecond2CommonTime(*bdsWS, ctBDS);
        cout << "  → CommonTime: " << ctBDS << endl;
        
        // 反向转换验证
        BDTWeekSecond bdsBack;
        CommonTime2WeekSecond(ctBDS, bdsBack);
        cout << "  → 还原 BDS WeekSecond: week=" << bdsBack.getWeek() 
             << ", sow=" << fixed << bdsBack.getSOW() << endl;
        
        delete bdsWS;

        // --------------------------
        // 14. WeekSecond 工厂函数测试（Galileo）
        // --------------------------
        cout << "\n【14. WeekSecond 工厂函数测试（Galileo）】" << endl;
        WeekSecond* galWS = createWeekSecond(TimeSystem::GAL, 1300, 43200.0);
        cout << "  创建 Galileo WeekSecond: week=" << galWS->getWeek() 
             << ", sow=" << fixed << galWS->getSOW() 
             << ", system=" << galWS->timeSystem.toString() << endl;
        cout << "  Galileo Nbits=" << galWS->Nbits() 
             << ", bitmask=0x" << hex << galWS->bitmask() << dec 
             << ", MJDEpoch=" << galWS->MJDEpoch() << endl;
        
        CommonTime ctGAL;
        WeekSecond2CommonTime(*galWS, ctGAL);
        cout << "  → CommonTime: " << ctGAL << endl;
        
        delete galWS;

        // --------------------------
        // 15. WeekSecond 工厂函数测试（GLONASS）
        // --------------------------
        // 注意：GLONASS不使用周秒格式，直接使用UTC时间
        // 这里为了API一致性保留WeekSecond接口，但Nbits=0表示无周数概念
        cout << "\n【15. WeekSecond 工厂函数测试（GLONASS）】" << endl;
        WeekSecond* gloWS = createWeekSecond(TimeSystem::GLO, 0, 43200.0);
        cout << "  创建 GLONASS WeekSecond: week=" << gloWS->getWeek() 
             << ", sow=" << fixed << gloWS->getSOW() 
             << ", system=" << gloWS->timeSystem.toString() << endl;
        cout << "  GLONASS Nbits=" << gloWS->Nbits() 
             << "(无周数概念), bitmask=0x" << hex << gloWS->bitmask() << dec 
             << ", MJDEpoch=" << gloWS->MJDEpoch() << endl;
        
        CommonTime ctGLO;
        WeekSecond2CommonTime(*gloWS, ctGLO);
        cout << "  → CommonTime: " << ctGLO << endl;
        
        delete gloWS;

        // --------------------------
        // 16. WeekSecond 工厂函数测试（QZSS）
        // --------------------------
        cout << "\n【16. WeekSecond 工厂函数测试（QZSS）】" << endl;
        WeekSecond* qzsWS = createWeekSecond(TimeSystem::QZS, 2200, 43200.0);
        cout << "  创建 QZSS WeekSecond: week=" << qzsWS->getWeek() 
             << ", sow=" << fixed << qzsWS->getSOW() 
             << ", system=" << qzsWS->timeSystem.toString() << endl;
        cout << "  QZSS Nbits=" << qzsWS->Nbits() 
             << ", bitmask=0x" << hex << qzsWS->bitmask() << dec 
             << ", MJDEpoch=" << qzsWS->MJDEpoch() << endl;
        
        CommonTime ctQZS;
        WeekSecond2CommonTime(*qzsWS, ctQZS);
        cout << "  → CommonTime: " << ctQZS << endl;
        
        delete qzsWS;

        // --------------------------
        // 17. WeekSecond 工厂函数测试（IRNSS）
        // --------------------------
        cout << "\n【17. WeekSecond 工厂函数测试（IRNSS）】" << endl;
        WeekSecond* irnWS = createWeekSecond(TimeSystem::IRN, 500, 43200.0);
        cout << "  创建 IRNSS WeekSecond: week=" << irnWS->getWeek() 
             << ", sow=" << fixed << irnWS->getSOW() 
             << ", system=" << irnWS->timeSystem.toString() << endl;
        cout << "  IRNSS Nbits=" << irnWS->Nbits() 
             << ", bitmask=0x" << hex << irnWS->bitmask() << dec 
             << ", MJDEpoch=" << irnWS->MJDEpoch() << endl;
        
        CommonTime ctIRN;
        WeekSecond2CommonTime(*irnWS, ctIRN);
        cout << "  → CommonTime: " << ctIRN << endl;
        
        delete irnWS;

        // --------------------------
        // 18. 多系统时间转换测试
        // --------------------------
        cout << "\n【18. 多系统时间转换测试】" << endl;
        CommonTime gps_Time(59644, 43200.0, TimeSystem::GPS);
        cout << "  GPS时间: " << gps_Time << endl;
        
        CommonTime bdtTime = convertTimeSystem(gps_Time, TimeSystem::BDT);
        cout << "  → BDT时间: " << bdtTime << endl;
        
        CommonTime galTime = convertTimeSystem(gps_Time, TimeSystem::GAL);
        cout << "  → GAL时间: " << galTime << endl;
        
        CommonTime gloTime = convertTimeSystem(gps_Time, TimeSystem::GLO);
        cout << "  → GLO时间: " << gloTime << endl;
        
        CommonTime qzsTime = convertTimeSystem(gps_Time, TimeSystem::QZS);
        cout << "  → QZS时间: " << qzsTime << endl;
        
        CommonTime irnTime = convertTimeSystem(gps_Time, TimeSystem::IRN);
        cout << "  → IRN时间: " << irnTime << endl;

        cout << "\n==========================================" << endl;
        cout << "              所有转换成功完成！" << endl;
        cout << "==========================================" << endl;

        // --------------------------
        // 19. 导出时间系统数据
        // --------------------------
        cout << "\n【19. 导出时间系统数据】" << endl;
        CommonTime exportTime(59644, 43200.0, TimeSystem::GPS);
        
        if (gnss::DataExporter::exportTimeConversionData(exportTime)) {
            cout << "  ✓ 时间转换数据导出成功" << endl;
        } else {
            cout << "  ✗ 时间转换数据导出失败" << endl;
        }
        
        if (gnss::DataExporter::exportTimeSystemParams()) {
            cout << "  ✓ 时间系统参数导出成功" << endl;
        } else {
            cout << "  ✗ 时间系统参数导出失败" << endl;
        }
        
        if (gnss::DataExporter::exportTimeConversionErrors()) {
            cout << "  ✓ 时间转换误差数据导出成功" << endl;
        } else {
            cout << "  ✗ 时间转换误差数据导出失败" << endl;
        }

    } catch (const BaseException& e) {
        cerr << "\n错误: " << e.what() << endl;
        return 1;
    } catch (const std::exception& e) {
        cerr << "\n标准异常: " << e.what() << endl;
        return 1;
    }

    return 0;
}