/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *  As stipulated by the MulanPSL-2.0, you are granted the following freedoms:
 *      To copy, use, and modify the software;
 *      To use the software for commercial purposes;
 *      To redistribute the software.
 *
 * Author: Shoujian Zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * References:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
 * 2. Eckel, Bruce. Thinking in C++. 2nd ed., Prentice Hall, 2000.
 */
#include <cmath>
#include <map>
#include "TimeConvert.h"
#include "TimeStruct.h"
#include "Const.h"

#define debug 0
#define debugYDS 0

using namespace std;

// 跳秒数据结构
struct LeapSecondData {
    double mjd;      // Modified Julian Date
    double leapSec;  // 跳秒数
};

// 静态跳秒数据（仅在第一次调用时初始化）
const std::vector<LeapSecondData>& getLeapSecondData() {
    static const std::vector<LeapSecondData> leapData = {
        {41317.0, 10},   // 1972-01-01
        {41499.0, 11},   // 1972-07-01
        {41683.0, 12},   // 1973-01-01
        {42048.0, 13},   // 1974-01-01
        {42413.0, 14},   // 1975-01-01
        {42778.0, 15},   // 1976-01-01
        {43144.0, 16},   // 1977-01-01
        {43509.0, 17},   // 1978-01-01
        {43874.0, 18},   // 1979-01-01
        {44239.0, 19},   // 1980-01-01
        {44786.0, 20},   // 1981-07-01
        {45151.0, 21},   // 1982-07-01
        {45516.0, 22},   // 1983-07-01
        {46247.0, 23},   // 1985-07-01
        {47161.0, 24},   // 1988-01-01
        {47892.0, 25},   // 1990-01-01
        {48257.0, 26},   // 1991-01-01
        {48804.0, 27},   // 1992-07-01
        {49169.0, 28},   // 1993-07-01
        {49534.0, 29},   // 1994-07-01
        {50083.0, 30},   // 1996-01-01
        {50630.0, 31},   // 1997-07-01
        {51179.0, 32},   // 1999-01-01
        {53736.0, 33},   // 2006-01-01
        {54832.0, 34},   // 2009-01-01
        {56109.0, 35},   // 2012-07-01
        {57204.0, 36},   // 2015-07-01
        {57754.0, 37}    // 2017-01-01
    };
    return leapData;
}

// 在给定的历史记录中查找跳秒
double getLeapSeconds(const CommonTime &ct) {
    double mjd_ct = ct.m_day;

    // 1972.1.1 之前不支持跳秒查询
    const double MJD_1972 = 41317.0;
    if (mjd_ct < MJD_1972) {
        InvalidRequest e("Time MUST be greater than 1972-01-01 for leap seconds!");
        throw e;
    }

    const auto& leapData = getLeapSecondData();
    double leapSec = 0.0;

    // 查找不大于当前MJD的最大跳秒值
    for (const auto& ld : leapData) {
        if (ld.mjd <= mjd_ct) {
            leapSec = ld.leapSec;
        } else {
            break;  // 数据已按MJD升序排列，可提前退出
        }
    }

    return leapSec;
}


// 将时间转换为TAI时间系统
// 返回值为需要调整的秒数（输入时间 + 返回值 = TAI时间）
double convertToTAI(const CommonTime &ct) {
    TimeSystem ts = ct.m_timeSystem;
    
    switch (ts.system) {
        case TimeSystem::GPS:
            return 19.0;  // GPS = TAI - 19s
        case TimeSystem::UTC:
            return getLeapSeconds(ct);  // UTC = TAI - leapSec
        case TimeSystem::BDT:
            return 33.0;  // BDT = TAI - 33s (RINEX 3.02)
        case TimeSystem::TAI:
            return 0.0;   // 已经是TAI
        case TimeSystem::GAL:
            return 19.0;  // Galileo = TAI - 19s
        case TimeSystem::GLO:
            return 0.0;   // GLONASS与UTC同步，此处简化处理
        case TimeSystem::QZS:
            return 19.0;  // QZSS = TAI - 19s (与GPS相同)
        case TimeSystem::IRN:
            return 19.0;  // IRNSS = TAI - 19s
        default:
            InvalidRequest e("Unsupported input TimeSystem: " + ts.toString());
            throw e;
    }
}

// 从TAI时间系统转换为目标时间系统
// 返回值为需要调整的秒数（TAI时间 + 返回值 = 目标时间）
double convertFromTAI(const CommonTime &ct, const TimeSystem &targetTS) {
    switch (targetTS.system) {
        case TimeSystem::GPS:
            return -19.0;  // GPS = TAI - 19s
        case TimeSystem::UTC: {
            // UTC转换需要考虑跳秒变化
            double dt_to_tai = getLeapSeconds(ct);
            CommonTime tai_time = ct + dt_to_tai;
            return -getLeapSeconds(tai_time);
        }
        case TimeSystem::BDT:
            return -33.0;  // BDT = TAI - 33s
        case TimeSystem::TAI:
            return 0.0;    // 目标就是TAI
        case TimeSystem::GAL:
            return -19.0;  // Galileo = TAI - 19s
        case TimeSystem::GLO:
        {
            // UTC转换需要考虑跳秒变化
            double dt_to_tai = getLeapSeconds(ct);
            CommonTime tai_time = ct + dt_to_tai;
            return -getLeapSeconds(tai_time);
        }
        case TimeSystem::QZS:
            return -19.0;  // QZSS = TAI - 19s (与GPS相同)
        case TimeSystem::IRN:
            return -19.0;  // IRNSS = TAI - 19s
        default:
            InvalidRequest e("Unsupported output TimeSystem: " + targetTS.toString());
            throw e;
    }
}

// 时间系统转换
CommonTime convertTimeSystem(
        const CommonTime &ct,
        const TimeSystem &outTS) {

    TimeSystem inTS = ct.m_timeSystem;

    // 如果输入输出时间系统相同，直接返回
    if (inTS == outTS) {
        return ct;
    }

    // 转换策略：先转换到TAI，再从TAI转换到目标系统
    double dt_to_tai = convertToTAI(ct);
    CommonTime tai_time = ct + dt_to_tai;
    
    double dt_from_tai = convertFromTAI(tai_time, outTS);
    CommonTime outT = tai_time + dt_from_tai;
    outT.setTimeSystem(outTS);

    return outT;
}

//
void convertJD2YMD(double jd,
                   int &iyear,
                   int &imonth,
                   int &iday) {
    double a = std::floor(jd + 0.5);
    double b = a + 1537;
    double c = std::floor((b - 122.1) / 365.25);
    double d = std::floor(365.25 * c);
    double e = std::floor((b - d) / 30.6001);
    iday = b - d - std::floor(30.6001 * e) + (jd + 0.5) - std::floor(jd + 0.5);
    imonth = e - 1 - 12. * std::floor(e / 14);
    iyear = c - 4715 - std::floor((7 + imonth) / 10);
}

double convertYMD2JD(int yy, int mm, int dd) {

    if (mm <= 2) {
        mm += 12;
        yy -= 1;
    }

    double B = 2 - floor(yy / 100) + floor(yy / 400);
    double jd_double = floor(365.25 * (yy + 4716)) + floor(30.6001 * (mm + 1)) + B + dd - 1524.5;
    return jd_double;

}

void convertSOD2HMS(double sod,
                    int &hh,
                    int &mm,
                    double &sec) {
    // Get us to within one day.
    if (sod < 0) {
        sod += (1 +
                static_cast<unsigned long>(sod / SEC_PER_DAY)) * SEC_PER_DAY;
    } else if (sod >= SEC_PER_DAY) {
        sod -= static_cast<unsigned long>(sod / SEC_PER_DAY) * SEC_PER_DAY;
    }

    double temp;               // variable to hold the integer part of sod
    sod = modf(sod, &temp);    // sod holds the fraction, temp the integer
    long seconds = static_cast<long>(temp); // get temp into a real integer

    hh = seconds / 3600;
    mm = (seconds % 3600) / 60;
    sec = double(seconds % 60) + sod;

}

double convertHMS2SOD(int hh,
                      int mm,
                      double sec) {
    return (sec + 60. * (mm + 60. * hh));
}

CommonTime CivilTime2CommonTime(const CivilTime &civilt) {
    CommonTime ct;
    // get the julian day
    double jday = convertYMD2JD(civilt.year, civilt.month, civilt.day);

    // convert jday to mjd day.
    int mjd_day = jday - MJD_TO_JD;

    // get the second of day
    double sod = convertHMS2SOD(civilt.hour, civilt.minute, civilt.second);

    // mjd_day + sod
    ct.set(mjd_day, sod, civilt.timeSys);

    return ct;
}

CivilTime CommonTime2CivilTime(const CommonTime &ct) {
    CivilTime civilt;
    long mjd_day;

    double sod;
    TimeSystem sys;
    // get the julian day, second of day, and fractional second of day
    ct.get(mjd_day, sod, sys);
    civilt.timeSys = sys;

    double jday;
    jday = mjd_day + MJD_TO_JD;

    // convert the julian day to calendar "year/month/day of month"
    convertJD2YMD(jday, civilt.year, civilt.month, civilt.day);

    // convert the (whole) second of day to "hour/minute/second"
    convertSOD2HMS(sod, civilt.hour, civilt.minute, civilt.second);

    return civilt;
}

CommonTime JulianDate2CommonTime(JulianDate &jd) {
    CommonTime ct;
    long double temp_jd(jd.jd);

    long mjd_day = static_cast<long>(temp_jd - MJD_TO_JD);

    long double sod = (temp_jd - std::floor(temp_jd)) * SEC_PER_DAY;

    ct.set(mjd_day,
           static_cast<double>(sod),
           jd.timeSystem);
    return ct;
}

JulianDate CommonTime2JulianDate(const CommonTime &ct) {
    JulianDate jd;
    long mjd_day;
    double sod;
    ct.get(mjd_day, sod, jd.timeSystem);

    double jday = mjd_day + MJD_TO_JD;
    jd.jd = static_cast<long double>(jday) +
            (static_cast<long double>(sod)) * DAY_PER_SEC;

    return jd;
};


CommonTime YDSTime2CommonTime(YDSTime &ydst) {
    CommonTime ct;
    long jday = convertYMD2JD(ydst.year, 1, 1) + ydst.doy - 1;
    ct.set(jday, ydst.sod, ydst.timeSystem);
    return ct;
}

YDSTime CommonTime2YDSTime(const CommonTime &ct) {
    YDSTime ydst;
    long mjday;
    double secDay;
    ct.get(mjday, secDay, ydst.timeSystem);
    ydst.sod = static_cast<double>(secDay);

    double jday;
    jday = mjday + MJD_TO_JD;

    int month, day;
    convertJD2YMD(jday, ydst.year, month, day);
    if (debugYDS)
        cout << "year:" << ydst.year << "month:" << month << "day:" << day;

    ydst.doy = jday - convertYMD2JD(ydst.year, 1, 1) + 1 ;
    return ydst;
}

void MJD2CommonTime(MJD& mjd, CommonTime& ct)
{
    try
    {

        long double mday = (mjd.mjd);
        // tmp now holds the partial days
        double sod =  mday - static_cast<long>(mday);
        // convert tmp to seconds of day
        sod *= SEC_PER_DAY;

        ct.set(mday,
               sod,
                mjd.timeSystem);
    }
    catch (InvalidRequest& ip)
    {
        InvalidRequest ir(ip);
        throw(ip);
    }
}

void CommonTime2MJD(const CommonTime& ct, MJD& mjd )
{
    long mday;
    double sod;
    double fsod;
    ct.get(mday, sod, mjd.timeSystem);
    mjd.mjd = static_cast<long double>(mday ) + static_cast<long double>(sod) * DAY_PER_SEC;
}

void CommonTime2WeekSecond(const CommonTime& ct, WeekSecond& wk )
{
    if (ct.m_timeSystem!=wk.timeSystem)
        cout<<"Warning,the Timesystem is differnent!!!"<<endl;
    MJD mjd;
    CommonTime2MJD(ct,mjd );
    if (mjd.mjd < wk.MJDEpoch())
    {
        InvalidRequest ir("Unable to convert to Week/Second - before Epoch.");
        throw(ir);
    }

    long mday;
    double sod;
    ct.get(mday, sod, wk.timeSystem);

    // find the number of days since the beginning of the Epoch
    mday -= wk.MJDEpoch();
    // find out how many weeks that is
    wk.week = static_cast<int>(mday / 7);
    // find out what the day of week is
    mday %= 7;

    wk.sow = static_cast<double>(mday * SEC_PER_DAY + sod) ;
}

void WeekSecond2CommonTime(WeekSecond& wk, CommonTime& ct)
{
    try
    {
        //int dow = static_cast<int>( sow * DAY_PER_SEC );
        // Appears to have rounding issues on 32-bit platforms

        int dow = static_cast<int>(wk.sow / SEC_PER_DAY);
        // NB this assumes MJDEpoch is an integer - what if epoch H:M:S != 0:0:0 ?

        long mday = wk.MJDEpoch() + (7 * wk.week) + dow;
        double sod(wk.sow - SEC_PER_DAY * dow);
        ct.set(mday,sod,wk.timeSystem);
    }
    catch (InvalidRequest& ip)
    {
        throw(ip);
    }
}