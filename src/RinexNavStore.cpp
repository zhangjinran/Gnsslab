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

#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/StringUtils.h>
#include <gnsslab/NavEphBase.hpp>
#include <gnsslab/TimeConvert.h>
#include <Eigen/Core>
#include <map>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <map>
#include <iostream>
#include <filesystem>
#include <thread>
#include <chrono>



using namespace std;
#define debug 0

const string RinexNavStore::stringVersion = "RINEX VERSION / TYPE";
const string RinexNavStore::stringRunBy = "PGM / RUN BY / DATE";
const string RinexNavStore::stringComment = "COMMENT";
const string RinexNavStore::stringIonoCorr = "IONOSPHERIC CORR";
const string RinexNavStore::stringTimeSysCorr = "TIME SYSTEM CORR";
const string RinexNavStore::stringLeapSeconds = "LEAP SECONDS";
//R2.10GLO
const string RinexNavStore::stringCorrSysTime = "CORR TO SYSTEM TIME";
//R2.11GPS
const string RinexNavStore::stringDeltaUTC = "DELTA-UTC: A0,A1,T,W";
//R2.11GEO
const string RinexNavStore::stringDUTC = "D-UTC A0,A1,T,W,S,U";
//R2.11
const string RinexNavStore::stringIonAlpha = "ION ALPHA";
//R2.11
const string RinexNavStore::stringIonBeta = "ION BETA";
const string RinexNavStore::stringEoH = "END OF HEADER";


void RinexNavStore::loadGPSEph(NavEphGPS &gpsEph,
                               string &line,
                               fstream &navFileStream)
{
    SatID sat(line.substr(0,3));

    /// add each sat into the satTable
    vector<SatID>::iterator result =
            find(satTable.begin(), satTable.end(), sat);

    if (result == satTable.end()) {
        satTable.push_back(sat);
    }

    //=========================================================
    // 读取 Toc 时间（RINEX 第一行）
    //=========================================================

    int yr  = safeStoi(line.substr(4, 4));
    int mo  = safeStoi(line.substr(9, 2));
    int day = safeStoi(line.substr(12, 2));
    int hr  = safeStoi(line.substr(15, 2));
    int min = safeStoi(line.substr(18, 2));
    double sec = safeStod(line.substr(21, 2));

    /// Fix RINEX epochs like: 23 12 31 23 59 60.0
    short ds = 0;

    if (sec >= 60.0) {
        ds = (short)sec;
        sec = 0.0;
    }

    CivilTime cvt(yr, mo, day, hr, min, sec);

    gpsEph.CivilToc = cvt;

    //---------------------------------------------------------
    // 正确：这里是 ctToc
    //---------------------------------------------------------

    gpsEph.ctToc = CivilTime2CommonTime(cvt);

    if (ds != 0)
        gpsEph.ctToc += ds;

    gpsEph.ctToc.setTimeSystem(TimeSystem::GPS);

    GPSWeekSecond gws;
    CommonTime2WeekSecond(gpsEph.ctToc, gws);

    gpsEph.Toc = gws.sow;

    //=========================================================
    // Clock parameters
    //=========================================================

    gpsEph.af0 = safeStod(line.substr(23, 19));
    gpsEph.af1 = safeStod(line.substr(42, 19));
    gpsEph.af2 = safeStod(line.substr(61, 19));

    //=========================================================
    // orbit-1
    //=========================================================

    int n = 4;

    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');

    gpsEph.IODE = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.Crs = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.Delta_n = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.M0 = safeStod(line.substr(n, 19));

    //=========================================================
    // orbit-2
    //=========================================================

    n = 4;

    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');

    gpsEph.Cuc = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.ecc = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.Cus = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.sqrt_A = safeStod(line.substr(n, 19));

    //=========================================================
    // orbit-3
    //=========================================================

    n = 4;

    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');

    //---------------------------------------------------------
    // Toe (seconds of week)
    //---------------------------------------------------------

    gpsEph.Toe = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.Cic = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.OMEGA_0 = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.Cis = safeStod(line.substr(n, 19));

    //=========================================================
    // orbit-4
    //=========================================================

    n = 4;

    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');

    gpsEph.i0 = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.Crc = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.omega = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.OMEGA_DOT = safeStod(line.substr(n, 19));

    //=========================================================
    // orbit-5
    //=========================================================

    n = 4;

    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');

    gpsEph.IDOT = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.L2Codes = safeStod(line.substr(n, 19));
    n += 19;

    //---------------------------------------------------------
    // GPS Week
    //---------------------------------------------------------

    gpsEph.GPSWeek = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.L2Pflag = safeStod(line.substr(n, 19));

    //=========================================================
    // 正确构造 ctToe
    //=========================================================

    GPSWeekSecond toeWS(
            gpsEph.GPSWeek,
            gpsEph.Toe,
            TimeSystem::GPS);

    WeekSecond2CommonTime(toeWS, gpsEph.ctToe);

    gpsEph.ctToe.setTimeSystem(TimeSystem::GPS);

    //=========================================================
    // orbit-6
    //=========================================================

    n = 4;

    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');

    gpsEph.URA = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.SV_health = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.TGD = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.IODC = safeStod(line.substr(n, 19));

    //=========================================================
    // orbit-7
    //=========================================================

    n = 4;

    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');

    gpsEph.HOWtime = safeStod(line.substr(n, 19));
    n += 19;

    gpsEph.fitInterval = safeStod(line.substr(n, 19));

    //=========================================================
    // Some process
    //=========================================================

    while (gpsEph.HOWtime < 0) {
        gpsEph.HOWtime += (long)FULLWEEK;
        gpsEph.GPSWeek--;
    }

    //---------------------------------------------------------
    // weeknum in RINEX is Toe week
    // internally convert to HOW week
    //---------------------------------------------------------

    if (gpsEph.HOWtime - gpsEph.Toe > HALFWEEK)
        gpsEph.GPSWeek--;
    else if (gpsEph.HOWtime - gpsEph.Toe < -HALFWEEK)
        gpsEph.GPSWeek++;

    //=========================================================
    // Build ctToc
    //=========================================================

    long adjHOWtime = gpsEph.HOWtime;
    short adjWeeknum = gpsEph.GPSWeek;

    long lToc = (long)gpsEph.Toc;

    if ((gpsEph.HOWtime % SEC_PER_DAY) == 0 &&
        (lToc % SEC_PER_DAY) == 0 &&
        gpsEph.HOWtime == lToc)
    {
        adjHOWtime = gpsEph.HOWtime - 30;

        if (adjHOWtime < 0) {
            adjHOWtime += FULLWEEK;
            adjWeeknum--;
        }
    }

    double dt = gpsEph.Toc - adjHOWtime;

    int week = gpsEph.GPSWeek;

    if (dt < -HALFWEEK)
        week++;
    else if (dt > HALFWEEK)
        week--;

    GPSWeekSecond gws2(
            week,
            gpsEph.Toc,
            TimeSystem::GPS);

    WeekSecond2CommonTime(gws2, gpsEph.ctToc);

    gpsEph.ctToc.setTimeSystem(TimeSystem::GPS);

    //=========================================================
    // store ephemeris
    //=========================================================

    gpsEphData[sat][gpsEph.ctToe] = gpsEph;
}
void RinexNavStore::loadBDSEph(NavEphBDS &bdsEph, string &line, fstream &navFileStream) {

    SatID sat(line.substr(0,3));
    //读取前三个字符串

    ///add each sat into the satTable
    vector<SatID>::iterator result = find(satTable.begin(), satTable.end(), sat);
    //iterator是迭代器，这里说迭代器是指针。
    if (result == satTable.end()) {
        satTable.push_back(sat);
    }


    //safeStoi是一个安全读取字符串，并转换为int类型的函数。
    int yr = safeStoi(line.substr(4, 4));
    int mo = safeStoi(line.substr(9, 2));
    int day = safeStoi(line.substr(12, 2));
    int hr = safeStoi(line.substr(15, 2));
    int min = safeStoi(line.substr(18, 2));
    double sec = safeStod(line.substr(21, 2));

    /// Fix RINEX epochs of the form 'yy mm dd hr 59 60.0'
    short ds = 0;
    if (sec >= 60.) {
        ds = sec;
        sec = 0;
    }

    CivilTime cvt(yr, mo, day, hr, min, sec,TimeSystem::BDT);
    bdsEph.CivilToc = cvt;
//      bdsEph.ctToe = cvt.convertToCommonTime();
    bdsEph.ctToe = CivilTime2CommonTime(cvt);

    if (ds != 0) bdsEph.ctToe += ds;

    BDTWeekSecond gws;
    CommonTime2WeekSecond(bdsEph.ctToe, gws);     // sow is system-independent

    bdsEph.Toc = gws.sow;
    bdsEph.af0 = safeStod(line.substr(23, 19));
    bdsEph.af1 = safeStod(line.substr(42, 19));
    bdsEph.af2 = safeStod(line.substr(61, 19));

    ///orbit-1
    int n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.IODE = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Crs = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Delta_n = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.M0 = safeStod(line.substr(n, 19));
    ///orbit-2
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.Cuc = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.ecc = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Cus = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.sqrt_A = safeStod(line.substr(n, 19));
    ///orbit-3
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.Toe = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Cic = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.OMEGA_0 = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Cis = safeStod(line.substr(n, 19));
    ///orbit-4
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.i0 = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Crc = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.omega = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.OMEGA_DOT = safeStod(line.substr(n, 19));
    ///orbit-5
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.IDOT = safeStod(line.substr(n, 19));
    n += 19;
    n+=19;
    bdsEph.BDSWeek = safeStod(line.substr(n, 19));
    n+=19;

    ///orbit-6
    ///北斗特有的五个参数，双延迟。
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.URA = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.SV_health = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.TGD1 = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.TGD2 = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.IODC = safeStod(line.substr(n, 19));
    ///orbit-7
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.HOWtime = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.fitInterval = safeStod(line.substr(n, 19));
    n += 19;

    /// some process
    /// Some RINEX files have HOW < 0.
    while (bdsEph.HOWtime < 0) {
        bdsEph.HOWtime += (long) FULLWEEK;
        bdsEph.BDSWeek--;
    }

    /// In RINEX *files*, weeknum is the week of TOE.
    /// Internally (Rx3NavData), weeknum is week of HOW
    if (bdsEph.HOWtime - bdsEph.Toe > HALFWEEK)
        bdsEph.BDSWeek--;
    else if (bdsEph.HOWtime - bdsEph.Toe < -HALFWEEK)
        bdsEph.BDSWeek++;

    /// Get week for clock, to build Toc
    long adjHOWtime = bdsEph.HOWtime;
    short adjWeeknum = bdsEph.BDSWeek;
    long lToc = (long) bdsEph.Toc;
    if ((bdsEph.HOWtime % SEC_PER_DAY) == 0 &&
        ((lToc) % SEC_PER_DAY) == 0 &&
        bdsEph.HOWtime == lToc) {
        adjHOWtime = bdsEph.HOWtime - 30;
        if (adjHOWtime < 0) {
            adjHOWtime += FULLWEEK;
            adjWeeknum--;
        }
    }

    double dt = bdsEph.Toc - adjHOWtime;
    int week = bdsEph.BDSWeek;
    if (dt < -HALFWEEK) week++; else if (dt > HALFWEEK) week--;
    BDTWeekSecond gws2 = BDTWeekSecond(week, bdsEph.Toc, TimeSystem::BDT);
    WeekSecond2CommonTime(gws2, bdsEph.ctToc);
    bdsEphData[sat][bdsEph.ctToe] = bdsEph;
}

void RinexNavStore::loadGLOEph(NavEphGLONASS& gloEph,
                               string& line,
                               fstream& navFileStream)
{
    //----------------------------------------------------------------------
    // Satellite ID
    //----------------------------------------------------------------------
    SatID sat(line.substr(0, 3));

    auto result = find(satTable.begin(), satTable.end(), sat);
    if (result == satTable.end())
    {
        satTable.push_back(sat);
    }

    //----------------------------------------------------------------------
    // Epoch
    //----------------------------------------------------------------------
    int yr  = safeStoi(line.substr(4, 4));
    int mo  = safeStoi(line.substr(9, 2));
    int day = safeStoi(line.substr(12, 2));
    int hr  = safeStoi(line.substr(15, 2));
    int min = safeStoi(line.substr(18, 2));

    // 秒字段必须允许小数
    double sec = safeStod(line.substr(21, 5));

    CivilTime cvt(yr, mo, day, hr, min, sec, TimeSystem::GLO);

    gloEph.CivilToc = cvt;

    //----------------------------------------------------------------------
    // GLONASS 时间系统
    //----------------------------------------------------------------------
    gloEph.ctToe = CivilTime2CommonTime(cvt);
    gloEph.ctToe.setTimeSystem(TimeSystem::GLO);

    gloEph.ctToc = gloEph.ctToe;
    gloEph.ctToc.setTimeSystem(TimeSystem::GLO);

    //----------------------------------------------------------------------
    // Line 1
    //
    // tau_n
    // gamma_n
    // tk (message frame time)
    //----------------------------------------------------------------------
    replace(line.begin(), line.end(), 'D', 'e');

    gloEph.tau_n   = safeStod(line.substr(23, 19));
    gloEph.gamma_n = safeStod(line.substr(42, 19));
    gloEph.Toc     = safeStod(line.substr(61, 19));

    //----------------------------------------------------------------------
    // Line 2
    //
    // X  Vx  Ax  health
    //----------------------------------------------------------------------
    int n = 4;

    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');

    gloEph.X = safeStod(line.substr(n, 19)) * 1000.0;
    n += 19;

    gloEph.Vx = safeStod(line.substr(n, 19)) * 1000.0;
    n += 19;

    gloEph.Ax = safeStod(line.substr(n, 19)) * 1e-6;
    n += 19;

    gloEph.health = safeStod(line.substr(n, 19));

    //----------------------------------------------------------------------
    // Line 3
    //
    // Y  Vy  Ay  freqNum
    //----------------------------------------------------------------------
    n = 4;

    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');

    gloEph.Y = safeStod(line.substr(n, 19)) * 1000.0;
    n += 19;

    gloEph.Vy = safeStod(line.substr(n, 19)) * 1000.0;
    n += 19;

    gloEph.Ay = safeStod(line.substr(n, 19)) * 1e-6;
    n += 19;

    gloEph.freqNum = static_cast<int>(safeStod(line.substr(n, 19)));

    //----------------------------------------------------------------------
    // Line 4
    //
    // Z  Vz  Az  ageOfData
    //----------------------------------------------------------------------
    n = 4;

    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');

    gloEph.Z = safeStod(line.substr(n, 19)) * 1000.0;
    n += 19;

    gloEph.Vz = safeStod(line.substr(n, 19)) * 1000.0;
    n += 19;

    gloEph.Az = safeStod(line.substr(n, 19)) * 1e-6;
    n += 19;

    gloEph.ageOfData = safeStod(line.substr(n, 19));

    //----------------------------------------------------------------------
    // Store ephemeris
    //----------------------------------------------------------------------
    gloEphData[sat][gloEph.ctToe] = gloEph;
}

void RinexNavStore::loadGalileoEph(NavEphGalileo &galEph, string &line, fstream &navFileStream) {
    SatID sat(line.substr(0,3));

    vector<SatID>::iterator result = find(satTable.begin(), satTable.end(), sat);
    if (result == satTable.end()) {
        satTable.push_back(sat);
    }

    int yr = safeStoi(line.substr(4, 4));
    int mo = safeStoi(line.substr(9, 2));
    int day = safeStoi(line.substr(12, 2));
    int hr = safeStoi(line.substr(15, 2));
    int min = safeStoi(line.substr(18, 2));
    double sec = safeStod(line.substr(21, 2));

    short ds = 0;
    if (sec >= 60.) {
        ds = sec;
        sec = 0;
    }

    CivilTime cvt(yr, mo, day, hr, min, sec, TimeSystem::GAL);
    galEph.CivilToc = cvt;
    galEph.ctToe = CivilTime2CommonTime(cvt);

    if (ds != 0) galEph.ctToe += ds;
    galEph.ctToe.setTimeSystem(TimeSystem::GAL);

    GALWeekSecond gws;
    CommonTime2WeekSecond(galEph.ctToe, gws);
    if (debug)
        cout<<"gws="<<gws.toString()<<endl;
    galEph.Toc = gws.sow;
    galEph.af0 = safeStod(line.substr(23, 19));
    galEph.af1 = safeStod(line.substr(42, 19));
    galEph.af2 = safeStod(line.substr(61, 19));

    int n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    galEph.IODE = safeStod(line.substr(n, 19));
    n += 19;
    galEph.Crs = safeStod(line.substr(n, 19));
    n += 19;
    galEph.Delta_n = safeStod(line.substr(n, 19));
    n += 19;
    galEph.M0 = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    galEph.Cuc = safeStod(line.substr(n, 19));
    n += 19;
    galEph.ecc = safeStod(line.substr(n, 19));
    n += 19;
    galEph.Cus = safeStod(line.substr(n, 19));
    n += 19;
    galEph.sqrt_A = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    galEph.Toe = safeStod(line.substr(n, 19));
    n += 19;
    galEph.Cic = safeStod(line.substr(n, 19));
    n += 19;
    galEph.OMEGA_0 = safeStod(line.substr(n, 19));
    n += 19;
    galEph.Cis = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    galEph.i0 = safeStod(line.substr(n, 19));
    n += 19;
    galEph.Crc = safeStod(line.substr(n, 19));
    n += 19;
    galEph.omega = safeStod(line.substr(n, 19));
    n += 19;
    galEph.OMEGA_DOT = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    galEph.IDOT = safeStod(line.substr(n, 19));
    n += 19;
    n += 19;
    galEph.GalileoWeek = safeStod(line.substr(n, 19))+(GPS_EPOCH_MJD-GAL_EPOCH_MJD)/7.0;

    n += 19;

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    galEph.SISA = safeStod(line.substr(n, 19));
    n += 19;

    galEph.SV_health = safeStod(line.substr(n, 19));
    n += 19;

    galEph.BGD_E5aE1 = safeStod(line.substr(n, 19));
    n += 19;

    galEph.BGD_E5bE1 = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    galEph.HOWtime = safeStod(line.substr(n, 19));
    n += 19;
    galEph.fitInterval = safeStod(line.substr(n, 19));
    n += 19;

    if (debug)
        cout<<"galileowweek="<<galEph.GalileoWeek<<endl;

    while (galEph.HOWtime < 0) {
        galEph.HOWtime += (long) FULLWEEK;
        galEph.GalileoWeek--;
    }

    if (galEph.HOWtime - galEph.Toe > HALFWEEK)
        galEph.GalileoWeek--;
    else if (galEph.HOWtime - galEph.Toe < -HALFWEEK)
        galEph.GalileoWeek++;

    long adjHOWtime = galEph.HOWtime;
    short adjWeeknum = galEph.GalileoWeek;
    long lToc = (long) galEph.Toc;
    if ((galEph.HOWtime % SEC_PER_DAY) == 0 &&
        ((lToc) % SEC_PER_DAY) == 0 &&
        galEph.HOWtime == lToc) {
        adjHOWtime = galEph.HOWtime - 30;
        if (adjHOWtime < 0) {
            adjHOWtime += FULLWEEK;
            adjWeeknum--;
        }
    }

    double dt = galEph.Toc - adjHOWtime;
    int week = galEph.GalileoWeek;
    if (debug)
        cout<<"week="<<week<<endl;
    if (dt < -HALFWEEK) week++; else if (dt > HALFWEEK) week--;
    GALWeekSecond gws2 = GALWeekSecond(week, galEph.Toc, TimeSystem::GAL);
    if (debug)
        cout<<"gws2="<<gws2.toString()<<endl;
    WeekSecond2CommonTime(gws2, galEph.ctToc);
    if (debug) {
        cout<<"ctToc="<<galEph.ctToc<<endl;
        cout<<"ctToe="<<galEph.ctToe<<endl;
    }

    galEphData[sat][galEph.ctToe] = galEph;
}

void RinexNavStore::loadQZSSEph(NavEphQZSS &qzssEph, string &line, fstream &navFileStream) {
    SatID sat(line.substr(0,3));

    vector<SatID>::iterator result = find(satTable.begin(), satTable.end(), sat);
    if (result == satTable.end()) {
        satTable.push_back(sat);
    }

    int yr = safeStoi(line.substr(4, 4));
    int mo = safeStoi(line.substr(9, 2));
    int day = safeStoi(line.substr(12, 2));
    int hr = safeStoi(line.substr(15, 2));
    int min = safeStoi(line.substr(18, 2));
    double sec = safeStod(line.substr(21, 2));

    short ds = 0;
    if (sec >= 60.) {
        ds = sec;
        sec = 0;
    }

    CivilTime cvt(yr, mo, day, hr, min, sec);
    qzssEph.CivilToc = cvt;
    qzssEph.ctToe = CivilTime2CommonTime(cvt);

    if (ds != 0) qzssEph.ctToe += ds;
    qzssEph.ctToe.setTimeSystem(TimeSystem::QZS);

    WeekSecond* gws = createWeekSecond(TimeSystem::QZS,0,0);
    CommonTime2WeekSecond(qzssEph.ctToe, *gws);
    qzssEph.Toc = gws->sow;
    qzssEph.af0 = safeStod(line.substr(23, 19));
    qzssEph.af1 = safeStod(line.substr(42, 19));
    qzssEph.af2 = safeStod(line.substr(61, 19));

    int n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    qzssEph.IODE = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.Crs = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.Delta_n = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.M0 = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    qzssEph.Cuc = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.ecc = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.Cus = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.sqrt_A = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    qzssEph.Toe = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.Cic = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.OMEGA_0 = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.Cis = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    qzssEph.i0 = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.Crc = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.omega = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.OMEGA_DOT = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    qzssEph.IDOT = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.L2Codes = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.QZSSWeek = safeStod(line.substr(n, 19));
    n += 19;

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    qzssEph.URA = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.SV_health = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.TGD = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.IODC = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    qzssEph.HOWtime = safeStod(line.substr(n, 19));
    n += 19;
    qzssEph.fitInterval = safeStod(line.substr(n, 19));
    n += 19;

    while (qzssEph.HOWtime < 0) {
        qzssEph.HOWtime += (long) FULLWEEK;
        qzssEph.QZSSWeek--;
    }

    if (qzssEph.HOWtime - qzssEph.Toe > HALFWEEK)
        qzssEph.QZSSWeek--;
    else if (qzssEph.HOWtime - qzssEph.Toe < -HALFWEEK)
        qzssEph.QZSSWeek++;

    long adjHOWtime = qzssEph.HOWtime;
    short adjWeeknum = qzssEph.QZSSWeek;
    long lToc = (long) qzssEph.Toc;
    if ((qzssEph.HOWtime % SEC_PER_DAY) == 0 &&
        ((lToc) % SEC_PER_DAY) == 0 &&
        qzssEph.HOWtime == lToc) {
        adjHOWtime = qzssEph.HOWtime - 30;
        if (adjHOWtime < 0) {
            adjHOWtime += FULLWEEK;
            adjWeeknum--;
        }
    }

    double dt = qzssEph.Toc - adjHOWtime;
    int week = qzssEph.QZSSWeek;
    if (dt < -HALFWEEK) week++; else if (dt > HALFWEEK) week--;
    WeekSecond* gws2 = createWeekSecond(TimeSystem::QZS,week, qzssEph.Toc);
    WeekSecond2CommonTime(*gws2, qzssEph.ctToc);
    qzssEph.ctToc.setTimeSystem(TimeSystem::QZS);
    qzssEphData[sat][qzssEph.ctToe] = qzssEph;
}

void RinexNavStore::loadIRNSSEph(NavEphIRNSS &irnssEph, string &line, fstream &navFileStream) {
    SatID sat(line.substr(0,3));

    vector<SatID>::iterator result = find(satTable.begin(), satTable.end(), sat);
    if (result == satTable.end()) {
        satTable.push_back(sat);
    }

    int yr = safeStoi(line.substr(4, 4));
    int mo = safeStoi(line.substr(9, 2));
    int day = safeStoi(line.substr(12, 2));
    int hr = safeStoi(line.substr(15, 2));
    int min = safeStoi(line.substr(18, 2));
    double sec = safeStod(line.substr(21, 2));

    short ds = 0;
    if (sec >= 60.) {
        ds = sec;
        sec = 0;
    }

    CivilTime cvt(yr, mo, day, hr, min, sec);
    irnssEph.CivilToc = cvt;
    irnssEph.ctToe = CivilTime2CommonTime(cvt);

    if (ds != 0) irnssEph.ctToe += ds;
    irnssEph.ctToe.setTimeSystem(TimeSystem::IRN);

    WeekSecond* gws = createWeekSecond(TimeSystem::IRN,0,0);
    CommonTime2WeekSecond(irnssEph.ctToe, *gws);
    irnssEph.Toc = gws->sow;
    irnssEph.af0 = safeStod(line.substr(23, 19));
    irnssEph.af1 = safeStod(line.substr(42, 19));
    irnssEph.af2 = safeStod(line.substr(61, 19));

    int n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    irnssEph.IODE = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.Crs = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.Delta_n = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.M0 = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    irnssEph.Cuc = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.ecc = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.Cus = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.sqrt_A = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    irnssEph.Toe = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.Cic = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.OMEGA_0 = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.Cis = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    irnssEph.i0 = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.Crc = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.omega = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.OMEGA_DOT = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    irnssEph.IDOT = safeStod(line.substr(n, 19));
    n += 19;
    n += 19;
    irnssEph.IRNSSWeek = safeStod(line.substr(n, 19));
    n += 19;

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    irnssEph.URA = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.SV_health = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.TGD = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.IODC = safeStod(line.substr(n, 19));

    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    irnssEph.HOWtime = safeStod(line.substr(n, 19));
    n += 19;
    irnssEph.fitInterval = safeStod(line.substr(n, 19));
    n += 19;

    while (irnssEph.HOWtime < 0) {
        irnssEph.HOWtime += (long) FULLWEEK;
        irnssEph.IRNSSWeek--;
    }

    if (irnssEph.HOWtime - irnssEph.Toe > HALFWEEK)
        irnssEph.IRNSSWeek--;
    else if (irnssEph.HOWtime - irnssEph.Toe < -HALFWEEK)
        irnssEph.IRNSSWeek++;

    long adjHOWtime = irnssEph.HOWtime;
    short adjWeeknum = irnssEph.IRNSSWeek;
    long lToc = (long) irnssEph.Toc;
    if ((irnssEph.HOWtime % SEC_PER_DAY) == 0 &&
        ((lToc) % SEC_PER_DAY) == 0 &&
        irnssEph.HOWtime == lToc) {
        adjHOWtime = irnssEph.HOWtime - 30;
        if (adjHOWtime < 0) {
            adjHOWtime += FULLWEEK;
            adjWeeknum--;
        }
    }

    double dt = irnssEph.Toc - adjHOWtime;
    int week = irnssEph.IRNSSWeek;
    if (dt < -HALFWEEK) week++; else if (dt > HALFWEEK) week--;
    WeekSecond* gws2 = createWeekSecond(TimeSystem::IRN,week, irnssEph.Toc);
    WeekSecond2CommonTime(*gws2, irnssEph.ctToc);
    irnssEph.ctToc.setTimeSystem(TimeSystem::IRN);
    irnssEphData[sat][irnssEph.ctToe] = irnssEph;
}

NavEphGalileo RinexNavStore::findGalileoEph(const SatID &sat, const CommonTime &epoch) {
    CommonTime epoch_gal = epoch;
    if (epoch.m_timeSystem != TimeSystem::GAL)
        epoch_gal = convertTimeSystem(epoch, TimeSystem::GAL);

    auto it_map = galEphData.find(sat);

    if (it_map == galEphData.end()) {
        InvalidRequest e("RinexNavStore::findGalileoEph: No ephemeris data for satellite " + sat.toString());
        throw e;
    }

    double diff_temp(0.0);
    double diff_best(1e9);
    NavEphGalileo galEph_best;
    for (auto & it: it_map->second) {
        GALWeekSecond ws;
        CommonTime2WeekSecond(it.first, ws);
        GALWeekSecond targetWS;
        CommonTime2WeekSecond(epoch_gal, targetWS);
        diff_temp = ws.sow + ws.week * FULLWEEK - targetWS.sow - targetWS.week * FULLWEEK;

        if (fabs(diff_temp)<diff_best) {
            diff_best=fabs(diff_temp);
            galEph_best=it.second;
        }
    }

    // 星历有效性检查
    double ageHours = diff_best / 3600.0;
    if (ageHours > 2.0) {
        InvalidRequest e("Ephemeris expired for satellite " + sat.toString() + " (age: " + to_string(ageHours) + "h)");
        throw e;
    } else if (ageHours > 1.0) {
        CivilTime civil = CommonTime2CivilTime(epoch);
        if (debug) {
            cout << "[WARNING] Ephemeris age >1 hour for satellite " << sat
                 << " (age: " << ageHours << "h)"
                 << " Epoch: " << civil
                 << endl;
        }
    }

    return galEph_best;
}

NavEphGLONASS RinexNavStore::findGLOEph(const SatID& sat,
                                        const CommonTime& epoch)
{
    //----------------------------------------------------------------------
    // Convert to GLONASS time system
    //----------------------------------------------------------------------
    CommonTime epoch_glo = epoch;

    if (epoch.m_timeSystem != TimeSystem::GLO)
    {
        epoch_glo = convertTimeSystem(epoch, TimeSystem::GLO);
    }

    //----------------------------------------------------------------------
    // Find satellite
    //----------------------------------------------------------------------
    auto it_map = gloEphData.find(sat);

    if (it_map == gloEphData.end())
    {
        InvalidRequest e(
            "RinexNavStore::findGLOEph: No ephemeris data for satellite "
            + sat.toString());

        throw e;
    }

    //----------------------------------------------------------------------
    // Find nearest PREVIOUS ephemeris
    //----------------------------------------------------------------------
    double diff_temp = 0.0;
    double diff_best = 1e9;

    bool found = false;

    NavEphGLONASS gloEph_best;

    for (auto& entry : it_map->second)
    {
            diff_temp = std::abs(epoch_glo - entry.first);


            if (diff_temp < diff_best)
            {
                diff_best = diff_temp;
                gloEph_best = entry.second;
                found = true;
            }
       
    }

    //----------------------------------------------------------------------
    // No valid ephemeris
    //----------------------------------------------------------------------
    if (!found)
    {
        InvalidRequest e(
            "RinexNavStore::findGLOEph: No valid previous ephemeris for satellite "
            + sat.toString());

        throw e;
    }

    //----------------------------------------------------------------------
    // Validity check
    //----------------------------------------------------------------------
    double ageHours = diff_best / 3600.0;
    if (ageHours > 2.0) {
        InvalidRequest e("Ephemeris expired for satellite " + sat.toString() + " (age: " + to_string(ageHours) + "h)");
        throw e;
    } else if (ageHours > 1.0) {
        CivilTime civil = CommonTime2CivilTime(epoch);
        if (debug) {
            cout << "[WARNING] Ephemeris age >1 hour for satellite " << sat
                 << " (age: " << ageHours << "h)"
                 << " Epoch: " << civil
                 << endl;
        }
    }

    return gloEph_best;
}

bool RinexNavStore::loadFile(string &file) {
    rx3NavFile = file;
    isLoadedFlag = false;

    if (rx3NavFile.size() == 0) {
        cerr << "[错误] 导航文件路径为空!" << endl;
        return false;
    }

    if (debug)
        cout << "RinexNavStore: fileName:" << rx3NavFile << endl;

    fstream navFileStream(rx3NavFile.c_str(), ios::in);
    if (!navFileStream) {
        cerr << "[错误] 无法打开文件:" << rx3NavFile << endl;
        return false;
    }

    int lineNumber(0);

    ///first, we should read nav head
    while (1) {
        string line;
        getline(navFileStream, line);

        if (debug)
            cout << "RinexNavStore:" << line << endl;

        stripTrailing(line);

        if (line.length() == 0) continue;
        else if (line.length() < 60) {
            cout << line << endl;
            cout << "line.length is fault" << line.length() << endl;
            FFStreamError e("Invalid line length, \n"
                            "may be the file is generated by windows, \n"
                            "please use dos2unix to convert the file!");
            throw (e);
        }

        lineNumber++;

        string thisLabel(line, 60, 20);

        /// following is huge if else else ... endif for each record type
        if (thisLabel == stringVersion) {
            /// "RINEX VERSION / TYPE"
            version = safeStod(line.substr(0, 20));
            fileType = strip(line.substr(20, 20));
            if(version<3.0)
            {
                FileMissingException e("don't support navigation file with version less than 3.0");
                throw(e);
            }
            if (version >= 3) {                        // ver 3
                if (fileType[0] != 'N' && fileType[0] != 'n') {
                    FFStreamError e("File type is not NAVIGATION: " + fileType);
                    throw(e);
                }
                fileSys = strip(line.substr(40, 20));   // not in ver 2
            }
            fileType = "NAVIGATION";
        } else if (thisLabel == stringRunBy) {
            /// "PGM / RUN BY / DATE"
            fileProgram = strip(line.substr(0, 20));
            fileAgency = strip(line.substr(20, 20));
            // R2 may not have 'UTC' at end
            date = strip(line.substr(40, 20));
        } else if (thisLabel == stringComment) {
            /// "COMMENT"
            commentList.push_back(strip(line.substr(0, 60)));
        } else if (thisLabel == stringIonoCorr) {
            /// "IONOSPHERIC CORR"
            string ionoCorrType = strip(line.substr(0, 4));
            vector<double> ionoCorrCoeff;
            for (int i = 0; i < 4; i++) {
                double ionoCorr = safeStod(line.substr(5 + 12 * i, 12));
                ionoCorrCoeff.push_back(ionoCorr);
            }
            ionoCorrData[ionoCorrType].clear();
            ionoCorrData[ionoCorrType] = ionoCorrCoeff;
            
            // BDS专用处理：提取PRN编号，转换为SatID
            if (ionoCorrType == "BDSA" || ionoCorrType == "BDSB") {
                // 跳过空格，找到数字部分（PRN编号）
                int prn = -1;
                for (size_t pos = 53; pos < line.size(); pos++) {
                    if (isdigit(line[pos])) {
                        prn = stoi(line.substr(pos));
                        break;
                    }
                }
                if (prn > 0) {
                    // 创建SatID：BDS系统代码为"C"，PRN格式化为两位数（如 2 -> "C02"）
                    char prnStr[4];
                    sprintf(prnStr, "%02d", prn);  // 补零到两位数
                    SatID sat("C" + string(prnStr));
                    
                    if (ionoCorrType == "BDSA") {
                        // 存储alpha参数
                        for (int i = 0; i < 4 && i < ionoCorrCoeff.size(); i++) {
                            ionoCorrDataBDS[sat].alpha[i] = ionoCorrCoeff[i];
                        }
                        ionoCorrDataBDS[sat].hasAlpha = true;
                    } else {
                        // 存储beta参数
                        for (int i = 0; i < 4 && i < ionoCorrCoeff.size(); i++) {
                            ionoCorrDataBDS[sat].beta[i] = ionoCorrCoeff[i];
                        }
                        ionoCorrDataBDS[sat].hasBeta = true;
                    }
                }
            }
        } else if (thisLabel == stringTimeSysCorr) {
            /// "TIME SYSTEM CORR"
            string timeSysCorrType = strip(line.substr(0, 4));

            TimeSysCorr timeSysCorrValue;
            timeSysCorrValue.A0 = safeStod(line.substr(5, 17));
            timeSysCorrValue.A1 = safeStod(line.substr(22, 16));
            timeSysCorrValue.refSOW = safeStoi(line.substr(38, 7));
            timeSysCorrValue.refWeek = safeStoi(line.substr(45, 5));
            timeSysCorrValue.geoProvider = string(" ");
            timeSysCorrValue.geoUTCid = 0;

            timeSysCorrData[timeSysCorrType] = timeSysCorrValue;
        } else if (thisLabel == stringLeapSeconds) {
            /// "LEAP SECONDS"
            leapSeconds = safeStoi(line.substr(0, 6));
            leapDelta = safeStoi(line.substr(6, 6));
            leapWeek = safeStoi(line.substr(12, 6));
            leapDay = safeStoi(line.substr(18, 6));
        } else if (thisLabel == stringEoH) {
            /// "END OF HEADER"
            break;
        }
    }

    ///now, start read nav data

    while (navFileStream.peek() != EOF) {
        string line;
        getline(navFileStream, line);

        if (debug)
            cout << "RinexNavStore:" << line << endl;

        replace(line.begin(), line.end(), 'D', 'e');

        if (line[0] == 'G') {
            EphData["G"]+=1;
            loadedSystems.insert("G");
            NavEphGPS gpsEph;
            loadGPSEph(gpsEph, line, navFileStream);
        }
        else if (line[0] == 'C') {
            EphData["C"]+=1;
            loadedSystems.insert("C");
            NavEphBDS bdsEph;
            loadBDSEph(bdsEph, line, navFileStream);
        }
        else if (line[0] == 'R') {
            EphData["R"]+=1;
            loadedSystems.insert("R");
            NavEphGLONASS gloEph;
            loadGLOEph(gloEph, line, navFileStream);
        }
        else if (line[0] == 'E') {
            EphData["E"]+=1;
            loadedSystems.insert("E");
            NavEphGalileo galEph;
            loadGalileoEph(galEph, line, navFileStream);
        }
        else if (line[0] == 'J') {
            EphData["J"]+=1;
            loadedSystems.insert("J");
            NavEphQZSS qzssEph;
            loadQZSSEph(qzssEph, line, navFileStream);
        }
        else if (line[0] == 'I') {
            EphData["I"]+=1;
            loadedSystems.insert("I");
            NavEphIRNSS irnssEph;
            loadIRNSSEph(irnssEph, line, navFileStream);
        }
        else  {
            if (debug)
                cout << "Don't support this Navigation system." << endl <<line[0]<< endl;
            if (line[0] == 'S')
                EphData["S"]+=1;
        }
    }

    // 设置加载成功标志
    isLoadedFlag = hasEphData();
    
    if (debug) {
        cout << "RinexNavStore: 加载完成，星历数据总数:" << endl;
        for (const auto& entry : EphData) {
            cout << "  系统 " << entry.first << ": " << entry.second << " 条" << endl;
        }
    }

    return isLoadedFlag;
}

bool RinexNavStore::hasEphData(const std::string& system) const {
    if (system == "G") {
        return !gpsEphData.empty();
    } else if (system == "C") {
        return !bdsEphData.empty();
    } else if (system == "R") {
        return !gloEphData.empty();
    } else if (system == "E") {
        return !galEphData.empty();
    } else if (system == "J") {
        return !qzssEphData.empty();
    } else if (system == "I") {
        return !irnssEphData.empty();
    }
    return EphData.at(system) > 0;
}

std::vector<std::string> RinexNavStore::getSystems() const {
    std::vector<std::string> systems;
    for (const auto& sys : loadedSystems) {
        systems.push_back(sys);
    }
    return systems;
}

std::vector<std::string> RinexNavStore::getSupportedSystems() const {
    return NavEphFactory::getSupportedSystems();
}

std::unique_ptr<NavEphBase> RinexNavStore::findEph(const SatID &sat, const CommonTime &epoch) {
    auto eph = NavEphFactory::create(sat.system);
    if (!eph) {
        InvalidRequest e("RinexNavStore: don't support the input satellite system!");
        throw (e);
    }
    
    CommonTime realEpoch = epoch;
    TimeSystem ts = eph->getTimeSystem();
    if (epoch.m_timeSystem != ts) {
        realEpoch = convertTimeSystem(epoch, ts);
    }
    
    if (sat.system == "G") {
        NavEphGPS gpsEph = findGPSEph(sat, realEpoch);
        *static_cast<NavEphGPS*>(eph.get()) = gpsEph;
    } else if (sat.system == "C") {
        NavEphBDS bdsEph = findBDSEph(sat, realEpoch);
        *static_cast<NavEphBDS*>(eph.get()) = bdsEph;
    } else if (sat.system == "R") {
        NavEphGLONASS gloEph = findGLOEph(sat, realEpoch);
        *static_cast<NavEphGLONASS*>(eph.get()) = gloEph;
    } else if (sat.system == "E") {
        NavEphGalileo galEph = findGalileoEph(sat, realEpoch);
        *static_cast<NavEphGalileo*>(eph.get()) = galEph;
    } else if (sat.system == "J") {
        NavEphQZSS qzssEph = findQZSSEph(sat, realEpoch);
        *static_cast<NavEphQZSS*>(eph.get()) = qzssEph;
    } else if (sat.system == "I") {
        NavEphIRNSS irnssEph = findIRNSSEph(sat, realEpoch);
        *static_cast<NavEphIRNSS*>(eph.get()) = irnssEph;
    }
    
    return eph;
}

Xvt RinexNavStore::getXvt(const SatID &sat, const CommonTime &epoch) {
    if (debug)
        cout << sat << endl;

    auto eph = findEph(sat, epoch);
    
    CommonTime realEpoch = epoch;
    TimeSystem ts = eph->getTimeSystem();
    if (epoch.m_timeSystem != ts) {
        realEpoch = convertTimeSystem(epoch, ts);
    }

    if (debug) {
        cout << "RinexNavStore::" << eph->getSystemCode() << " eph:" << endl;
        eph->printData();
    }

    Xvt xvt = eph->svXvt(realEpoch, sat);


    if (debug) {
        cout << "RinexNavStore::xvt:" << endl;
        cout << xvt << endl;
    }

    return xvt;
}

NavEphGPS RinexNavStore::findGPSEph(const SatID &sat, const CommonTime &epoch) {
    CommonTime epoch_gps = epoch;
    if (epoch.m_timeSystem != TimeSystem::GPS)
        epoch_gps = convertTimeSystem(epoch, TimeSystem::GPS);

    GPSWeekSecond targetWS;
    CommonTime2WeekSecond(epoch_gps, targetWS);

    // 安全查找map，不自动插入
    auto it_map = gpsEphData.find(sat);

    // 检查卫星是否存在
    if (it_map == gpsEphData.end()) {
        InvalidRequest e("RinexNavStore::findGPSEph: No ephemeris data for satellite " + sat.toString());
        throw e;
    }

    // 最接近星历查找逻辑
    double diff_temp(0.0);
    double diff_best(1e9);
    NavEphGPS gpsEph_best;

    for (auto &entry : it_map->second) {
        GPSWeekSecond ws;
        CommonTime2WeekSecond(entry.first, ws);
        diff_temp = ws.sow + ws.week * FULLWEEK - targetWS.sow - targetWS.week * FULLWEEK;
        if (fabs(diff_temp) < diff_best) {
            diff_best = fabs(diff_temp);
            gpsEph_best = entry.second;
        }
    }

    // 星历有效性检查
    double ageHours = diff_best / 3600.0;
    if (ageHours > 2.0) {
        InvalidRequest e("Ephemeris expired for satellite " + sat.toString() + " (age: " + to_string(ageHours) + "h)");
        throw e;
    } else if (ageHours > 1.0) {
        CivilTime civil = CommonTime2CivilTime(epoch);
        if (debug) {
            cout << "[WARNING] Ephemeris age >1 hour for satellite " << sat
                 << " (age: " << ageHours << "h)"
                 << " Epoch: " << civil
                 << endl;
        }
    }

    return gpsEph_best;
}


NavEphBDS RinexNavStore::findBDSEph(const SatID &sat, const CommonTime &epoch) {
    CommonTime epoch_bds=epoch;
    if (epoch.m_timeSystem!=TimeSystem::BDT)
        epoch_bds=convertTimeSystem(epoch,TimeSystem::BDT);
    BDTWeekSecond targetWS;
    CommonTime2WeekSecond(epoch_bds, targetWS);

    // 安全查找map，不自动插入
    auto it_map = bdsEphData.find(sat);
    
    // 检查卫星是否存在
    if (it_map == bdsEphData.end()) {
        InvalidRequest e("RinexNavStore::findBDSEph: No ephemeris data for satellite " + sat.toString());
        throw e;
    }

    // 寻找最接近的历元的卫星星历
    double diff_temp(0.0);
    double diff_best(1e9);
    NavEphBDS bdsEph_best;
    for (auto & it: it_map->second) {
        BDTWeekSecond ws;
        CommonTime2WeekSecond(it.first, ws);
        diff_temp = ws.sow + ws.week * FULLWEEK - targetWS.sow - targetWS.week * FULLWEEK;

        if (fabs(diff_temp)<diff_best) {
            diff_best=fabs(diff_temp);
            bdsEph_best=it.second;
        }
    }

    // 星历有效性检查
    double ageHours = diff_best / 3600.0;
    if (ageHours > 2.0) {
        InvalidRequest e("Ephemeris expired for satellite " + sat.toString() + " (age: " + to_string(ageHours) + "h)");
        throw e;
    } else if (ageHours > 1.0) {
        CivilTime civil = CommonTime2CivilTime(epoch);
        if (debug) {
            cout << "[WARNING] Ephemeris age >1 hour for satellite " << sat
                 << " (age: " << ageHours << "h)"
                 << " Epoch: " << civil
                 << endl;
        }
    }

    return bdsEph_best;
}

NavEphQZSS RinexNavStore::findQZSSEph(
    const SatID& sat,
    const CommonTime& epoch)
{
    CommonTime epoch_qzss = epoch;

    if(epoch.m_timeSystem != TimeSystem::QZS)
    {
        epoch_qzss =
            convertTimeSystem(epoch, TimeSystem::QZS);
    }

    auto it_map = qzssEphData.find(sat);

    if(it_map == qzssEphData.end())
    {
        throw InvalidRequest(
            "RinexNavStore::findQZSSEph: "
            "No ephemeris data for satellite "
            + sat.toString());
    }

    double diff_best = 1e99;
    NavEphQZSS bestEph;

    for(const auto& it : it_map->second)
    {
        double diff =
            fabs(it.first - epoch_qzss);

        if(diff < diff_best)
        {
            diff_best = diff;
            bestEph = it.second;
        }
    }

    // 星历有效性检查
    double ageHours = diff_best / 3600.0;
    if (ageHours > 2.0) {
        InvalidRequest e("Ephemeris expired for satellite " + sat.toString() + " (age: " + to_string(ageHours) + "h)");
        throw e;
    } else if (ageHours > 1.0) {
        CivilTime civil = CommonTime2CivilTime(epoch);
        if (debug) {
            cout << "[WARNING] Ephemeris age >1 hour for satellite " << sat
                 << " (age: " << ageHours << "h)"
                 << " Epoch: " << civil
                 << endl;
        }
    }

    return bestEph;
}

NavEphIRNSS RinexNavStore::findIRNSSEph(
    const SatID& sat,
    const CommonTime& epoch)
{
    CommonTime epoch_irnss = epoch;

    if(epoch.m_timeSystem != TimeSystem::IRN)
    {
        epoch_irnss =
            convertTimeSystem(epoch, TimeSystem::IRN);
    }

    auto it_map = irnssEphData.find(sat);

    if(it_map == irnssEphData.end())
    {
        throw InvalidRequest(
            "No ephemeris data for satellite "
            + sat.toString());
    }

    double diff_best = 1e99;
    NavEphIRNSS bestEph;

    for(const auto& it : it_map->second)
    {
        double diff =
            fabs(it.first - epoch_irnss);

        if(diff < diff_best)
        {
            diff_best = diff;
            bestEph = it.second;
        }
    }

    // 星历有效性检查
    double ageHours = diff_best / 3600.0;
    if (ageHours > 2.0) {
        InvalidRequest e("Ephemeris expired for satellite " + sat.toString() + " (age: " + to_string(ageHours) + "h)");
        throw e;
    } else if (ageHours > 1.0) {
        CivilTime civil = CommonTime2CivilTime(epoch);
        if (debug) {
            cout << "[WARNING] Ephemeris age >1 hour for satellite " << sat
                 << " (age: " << ageHours << "h)"
                 << " Epoch: " << civil
                 << endl;
        }
    }

    return bestEph;
}
void RinexNavStore::gerContrast(string system ,int Cout,CommonTime predictedTimeInit,CommonTime stoptime ,SP3Store sp3Store) {
    if (system=="GPS")
        GpsEphMap Dataset=this->gpsEphData;
    else if (system=="BDS")
        BdsEphMap Dataset=this->bdsEphData;
    else {
        cerr<<"The contrast function does not support this system!!!"<<endl;
        return;
    }


}

void RinexNavStore::getContrastData(SP3Store &sp3Store,CommonTime predictedTimeInit,CommonTime stoptime,int period) {
    int Cout=0;
    int Cout2=0;
    if (Cout)
        cout<<"**********************GPS**********************"<<endl;

for (auto it:gpsEphData) {
        //cout<<it.first<<endl;
        ContrastData contrastData;
        contrastData.sat=it.first;
        CommonTime predictedTime=predictedTimeInit;

        while (predictedTime<stoptime) {
            Xvt xvtNav=this->getXvt(it.first,predictedTime);
            contrastData.NavX[predictedTime]=xvtNav.getPos();
            contrastData.NavV[predictedTime]=xvtNav.getVel();

            try {
                Xvt xvtSP3 = sp3Store.getXvt(it.first, predictedTime);
                contrastData.SP3X[predictedTime]=xvtSP3.getPos();
                contrastData.SP3V[predictedTime]=xvtSP3.getVel();

                //cout << "sp3:" << xvtSP3 << endl;
                Vector3d xSP3 = xvtSP3.getPos();

                Vector3d diffXYZ = xvtNav.getPos() - xSP3;
                Vector3d diffVel = xvtNav.getVel() - xvtSP3.getVel();
                double diffClockBias = xvtNav.getClockBias() - xvtSP3.getClockBias();
                double diffRelCorr = xvtNav.getRelativityCorr() - xvtSP3.getRelativityCorr();
                contrastData.Clockbias_diff[predictedTime]=diffClockBias;
                contrastData.RelCorr_diff[predictedTime]=diffRelCorr;
                contrastData.X_diff[predictedTime]=diffXYZ;
                contrastData.V_diff[predictedTime]=diffVel;

                YDSTime ydsPrediced=CommonTime2YDSTime(predictedTime);

                if (Cout)
                {
                    cout << ydsPrediced << " \n"
                         << " sat:" << it.first << " \n"
                         << " nav:\n" << xvtNav << " \n"
                         << " sp3:\n" << xvtSP3 << " \n"
                         << " diffXYZ:\n" << diffXYZ << " \n"
                         << " diffVel:\n" << diffVel << " \n"
                         << " diffClockBias:\n" << diffClockBias << " \n"
                         << " diffRelCorr:\n" << diffRelCorr << " \n"
                         << endl;
                }

            }
            catch (...){
                cout<<"SP3store don't have this time."<<endl;
            }
            predictedTime+=period;
        }

        this->gpscontrastDataSet[it.first]=contrastData;
    }

    if (Cout2)
        cout<<"**********************BDS**********************"<<endl;

    for (auto it:this->bdsEphData) {

        ContrastData contrastData;
        contrastData.sat=it.first;
        CommonTime predictedTime=predictedTimeInit;

        while (predictedTime<stoptime) {
            try{
                Xvt xvtNav=this->getXvt(it.first,predictedTime);
                contrastData.NavX[predictedTime]=xvtNav.getPos();
                contrastData.NavV[predictedTime]=xvtNav.getVel();



                Xvt xvtSP3 = sp3Store.getXvt(it.first, predictedTime);
                contrastData.SP3X[predictedTime]=xvtSP3.getPos();
                contrastData.SP3V[predictedTime]=xvtSP3.getVel();


                Vector3d xSP3 = xvtSP3.getPos();

                Vector3d diffXYZ = xvtNav.getPos() - xSP3;
                Vector3d diffVel = xvtNav.getVel() - xvtSP3.getVel();
                double diffClockBias = xvtNav.getClockBias() - xvtSP3.getClockBias();
                double diffRelCorr = xvtNav.getRelativityCorr() - xvtSP3.getRelativityCorr();
                contrastData.Clockbias_diff[predictedTime]=diffClockBias;
                contrastData.RelCorr_diff[predictedTime]=diffRelCorr;
                contrastData.X_diff[predictedTime]=diffXYZ;
                contrastData.V_diff[predictedTime]=diffVel;

                YDSTime ydsPrediced=CommonTime2YDSTime(predictedTime);

                if (Cout2)
                {
                    cout << ydsPrediced << " \n"
                         << " sat:" << it.first << " \n"
                         << " nav:\n" << xvtNav << " \n"
                         << " sp3:\n" << xvtSP3 << " \n"
                         << " diffXYZ:\n" << diffXYZ << " \n"
                         << " diffVel:\n" << diffVel << " \n"
                         << " diffClockBias:\n" << diffClockBias << " \n"
                         << " diffRelCorr:\n" << diffRelCorr << " \n"
                         << endl;
                }
            }
                catch (...) {

                }
                predictedTime+=period;
        }
        this->bdscontrastDataSet[it.first]=contrastData;
    }
    return ;
}




void RinexNavStore::writeContrastData(std::ofstream& fout, const std::map<SatID, ContrastData>& dataSet) {
    for (const auto& it : dataSet) {
        const SatID& satID = it.first;
        const TimeSequence& time_sequence = it.second.X_diff;
        for (const auto& it2 : time_sequence) {
            fout << std::fixed << std::setprecision(15);
            fout << satID << ' '
                 << it2.first << ' '
                 << it2.second(0, 0) << ' '
                 << it2.second(1, 0) << ' '
                 << it2.second(2, 0) << ' '
                 << it.second.V_diff.at(it2.first)(0, 0) << ' '
                 << it.second.V_diff.at(it2.first)(1, 0) << ' '
                 << it.second.V_diff.at(it2.first)(2, 0) << ' '
                 << it.second.Clockbias_diff.at(it2.first) << ' '
                 << it.second.RelCorr_diff.at(it2.first) << std::endl;
        }
    }
}

void RinexNavStore::writeFile(std::string filename, std::string name) {
    std::ofstream fout(filename.c_str(), ios::out);
    if (!fout) {
        std::cerr << "Unable to open file for writing" << std::endl;
        return;
    }

    const std::map<std::string, const std::map<SatID, ContrastData>*> systemDataMap = {
        {"G", &this->gpscontrastDataSet},
        {"C", &this->bdscontrastDataSet},
        {"R", &this->gloContrastDataSet},
        {"E", &this->galContrastDataSet},
        {"GPS", &this->gpscontrastDataSet},
        {"BDS", &this->bdscontrastDataSet},
        {"GLONASS", &this->gloContrastDataSet},
        {"Galileo", &this->galContrastDataSet}
    };

    auto it = systemDataMap.find(name);
    if (it != systemDataMap.end()) {
        writeContrastData(fout, *it->second);
    } else {
        std::cerr << "Unsupported system: " << name << std::endl;
    }
}
