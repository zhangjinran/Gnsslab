//
// Created by zhang on 2026/3/14.
//
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
#include "TimeStruct.h"
#include "Const.h"
#include "TimeConvert.h"
#include"Time2020Convert.h"

#define debug 0
#define debugYDS 0

using namespace std;





CommonTime CivilTime20202CommonTime(const CivilTime &civilt) {
    CommonTime ct;
    // get the julian day
    double jday = convertYMD2JD(civilt.year, civilt.month, civilt.day);

    // convert jday to mjd day.
    int jd2020_day = jday - JD2020_TO_JD;

    // get the second of day
    double sod = convertHMS2SOD(civilt.hour, civilt.minute, civilt.second);

    // mjd_day + sod
    ct.set(jd2020_day, sod, civilt.timeSys);

    return ct;
}

CivilTime CommonTime20202CivilTime(const CommonTime &ct) {
    CivilTime civilt;
    long jd2020_day;

    double sod;
    TimeSystem sys;
    // get the julian day, second of day, and fractional second of day
    ct.get(jd2020_day, sod, sys);
    civilt.timeSys = sys;

    double jday;
    jday = jd2020_day + JD2020_TO_JD;

    // convert the julian day to calendar "year/month/day of month"
    convertJD2YMD(jday, civilt.year, civilt.month, civilt.day);

    // convert the (whole) second of day to "hour/minute/second"
    convertSOD2HMS(sod, civilt.hour, civilt.minute, civilt.second);

    return civilt;
}

CommonTime JulianDate2CommonTime2020(JulianDate &jd) {
    CommonTime ct;
    long double temp_jd(jd.jd);

    long jd2020_day = static_cast<long>(temp_jd - JD2020_TO_JD);

    long double sod = (temp_jd - std::floor(temp_jd)) * SEC_PER_DAY;

    ct.set(jd2020_day,
           static_cast<double>(sod),
           jd.timeSystem);
    return ct;
}

JulianDate CommonTime20202JulianDate(CommonTime &ct) {
    JulianDate jd;
    long jd2020_day;
    double sod;
    ct.get(jd2020_day, sod, jd.timeSystem);

    double jday = jd2020_day + JD2020_TO_JD;
    jd.jd = static_cast<long double>(jday) +
            (static_cast<long double>(sod)) * DAY_PER_SEC;

    return jd;
};


CommonTime YDSTime2CommonTime2020(YDSTime &ydst) {
    CommonTime ct;
    long jday = convertYMD2JD(ydst.year, 1, 1) + ydst.doy - 1;
    jday-=JD2020_TO_JD;
    ct.set(jday, ydst.sod, ydst.timeSystem);
    return ct;
}

YDSTime CommonTime20202YDSTime(const CommonTime &ct) {
    YDSTime ydst;
    long mjday;
    double secDay;
    ct.get(mjday, secDay, ydst.timeSystem);
    ydst.sod = static_cast<double>(secDay);

    double jday;
    jday = mjday + JD2020_TO_JD;

    int month, day;
    convertJD2YMD(jday, ydst.year, month, day);
    if (debugYDS)
        cout << "year:" << ydst.year << "month:" << month << "day:" << day;

    ydst.doy = jday - convertYMD2JD(ydst.year, 1, 1) + 1 ;
    return ydst;
}

void JD20202CommonTime2020(JD2020& jd2020, CommonTime& ct)
{
    try
    {

        long double mday = (jd2020.jd2020);
        // tmp now holds the partial days
        double sod =  mday - static_cast<long>(mday);
        // convert tmp to seconds of day
        sod *= SEC_PER_DAY;

        ct.set(mday,
               sod,
                jd2020.timeSystem);
    }
    catch (InvalidRequest& ip)
    {
        InvalidRequest ir(ip);
        throw(ip);
    }
}

void CommonTime20202JD2020(const CommonTime& ct, JD2020& jd2020 )
{
    long mday;
    double sod;
    double fsod;
    ct.get(mday, sod, jd2020.timeSystem);
    jd2020.jd2020 = static_cast<long double>(mday ) + static_cast<long double>(sod) * DAY_PER_SEC;
}

void CommonTime20202WeekSecond(const CommonTime& ct, WeekSecond& wk )
{
    JD2020 jd2020;
    CommonTime20202JD2020(ct,jd2020 );
    if (jd2020.mjd < wk.MJDEpoch())
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

void WeekSecond2CommonTime2020(WeekSecond& wk, CommonTime& ct)
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

CommonTime CommonTime20202CommonTime(CommonTime& ct) {
    long day;
    day=ct.m_day+JD2020_TO_JD-MJD_TO_JD;
        CommonTime ct_mjd;
    ct_mjd.set(day,ct.m_sod,ct.m_timeSystem);
    return ct_mjd;

}

CommonTime CommonTime2CommonTime2020(CommonTime& ct) {
    long day;
    day=ct.m_day-JD2020_TO_JD+MJD_TO_JD;
    CommonTime ct_mjd;
    ct_mjd.set(day,ct.m_sod,ct.m_timeSystem);
    return ct_mjd;

}