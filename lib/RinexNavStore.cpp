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

#include "RinexNavStore.hpp"
#include "StringUtils.h"
#include <Eigen/Core>
#include <map>
#include <vector>
#include <string>
#include <fmt/core.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <map>
#include <iostream>
#include <filesystem>
#include <thread>
#include <chrono>
#include <unistd.h>   // readlink
#include <limits.h>   // PATH_MAX



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


void RinexNavStore::loadGPSEph(NavEphGPS &gpsEph, string &line, fstream &navFileStream) {

    SatID sat(line.substr(0,3));
    //读取前三个字符串

    ///add each sat into the satTable
    vector<SatID>::iterator result = find(satTable.begin(), satTable.end(), sat);
    //iterator是迭代器，不知道是用来干什么的。
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

    CivilTime cvt(yr, mo, day, hr, min, sec);
    gpsEph.CivilToc = cvt;
//      gpsEph.ctToe = cvt.convertToCommonTime();
    gpsEph.ctToe = CivilTime2CommonTime(cvt);;

    if (ds != 0) gpsEph.ctToe += ds;
    gpsEph.ctToe.setTimeSystem(TimeSystem::GPS);

    GPSWeekSecond gws;
    CommonTime2WeekSecond(gpsEph.ctToe, gws);     // sow is system-independent

    gpsEph.Toc = gws.sow;
    gpsEph.af0 = safeStod(line.substr(23, 19));
    gpsEph.af1 = safeStod(line.substr(42, 19));
    gpsEph.af2 = safeStod(line.substr(61, 19));

    ///orbit-1
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
    ///orbit-2
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
    ///orbit-3
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    gpsEph.Toe = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.Cic = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.OMEGA_0 = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.Cis = safeStod(line.substr(n, 19));
    ///orbit-4
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
    ///orbit-5
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    gpsEph.IDOT = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.L2Codes = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.GPSWeek = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.L2Pflag = safeStod(line.substr(n, 19));
    ///orbit-6
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
    ///orbit-7
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    gpsEph.HOWtime = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.fitInterval = safeStod(line.substr(n, 19));
    n += 19;

    /// some process
    /// Some RINEX files have HOW < 0.
    while (gpsEph.HOWtime < 0) {
        gpsEph.HOWtime += (long) FULLWEEK;
        gpsEph.GPSWeek--;
    }

    /// In RINEX *files*, weeknum is the week of TOE.
    /// Internally (Rx3NavData), weeknum is week of HOW
    if (gpsEph.HOWtime - gpsEph.Toe > HALFWEEK)
        gpsEph.GPSWeek--;
    else if (gpsEph.HOWtime - gpsEph.Toe < -HALFWEEK)
        gpsEph.GPSWeek++;

    /// Get week for clock, to build Toc
    long adjHOWtime = gpsEph.HOWtime;
    short adjWeeknum = gpsEph.GPSWeek;
    long lToc = (long) gpsEph.Toc;
    if ((gpsEph.HOWtime % SEC_PER_DAY) == 0 &&
        ((lToc) % SEC_PER_DAY) == 0 &&
        gpsEph.HOWtime == lToc) {
        adjHOWtime = gpsEph.HOWtime - 30;
        if (adjHOWtime < 0) {
            adjHOWtime += FULLWEEK;
            adjWeeknum--;
        }
    }

    double dt = gpsEph.Toc - adjHOWtime;
    int week = gpsEph.GPSWeek;
    if (dt < -HALFWEEK) week++; else if (dt > HALFWEEK) week--;
    GPSWeekSecond gws2 = GPSWeekSecond(week, gpsEph.Toc, TimeSystem::GPS);
//      gpsEph.ctToc = GPSWeekSecond(week, gpsEph.Toc, TimeSystem::GPS);
    WeekSecond2CommonTime(gws2, gpsEph.ctToc);

    gpsEph.ctToc.setTimeSystem(TimeSystem::GPS);

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

void RinexNavStore::loadFile(string &file) {
    rx3NavFile = file;
    if (rx3NavFile.size() == 0) {
        cout << "the nav file path is empty!" << endl;
        exit(-1);
    }

    if (debug)
        cout << "RinexNavStore: fileName:" << rx3NavFile << endl;

    fstream navFileStream(rx3NavFile.c_str(), ios::in);
    if (!navFileStream) {
        cerr << "can't open file:" << rx3NavFile << endl;
        exit(-1);
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
            NavEphGPS gpsEph;
            loadGPSEph(gpsEph, line, navFileStream);
        }
        else if (line[0] == 'C') {
            EphData["C"]+=1;
            NavEphBDS bdsEph;
            loadBDSEph(bdsEph, line, navFileStream);
        }
        else  {
            if (debug)
                cout << "Don't support this Navigation system." << endl <<line[0]<< endl;
            if (line[0] == 'R')
                EphData["R"]+=1;
            else if (line[0] == 'E')
                EphData["E"]+=1;
            else if (line[0] == 'I')
                EphData["I"]+=1;
            else if (line[0] == 'S')
                EphData["S"]+=1;
            else if (line[0] == 'J')
                EphData["J"]+=1;


        }
    }
}

Xvt RinexNavStore::getXvt(const SatID &sat, const CommonTime &epoch) {
    Xvt xvt;
    CommonTime realEpoch;
    TimeSystem ts;
    if (debug)
        cout << sat << endl;
    if (sat.system == "G") {
        ts = TimeSystem::GPS;
        realEpoch = convertTimeSystem(epoch, ts);

        if (debug)
            cout << CommonTime2CivilTime(epoch) << endl;

        NavEphGPS gpsEph = findGPSEph(sat, realEpoch);

        if (debug) {
            cout << "RinexNavStore::GPS eph:" << endl;
            gpsEph.printData();
        }

        xvt = gpsEph.svXvt(realEpoch);

        if (debug) {
            cout << "RinexNavStore::xvt:" << endl;
            cout << xvt << endl;
        }
    }
    else if (sat.system == "C")  {
        ts = TimeSystem::BDT;
        realEpoch = convertTimeSystem(epoch, ts);

        if (debug)
            cout << CommonTime2CivilTime(epoch) << endl;

        NavEphBDS bdsEph = findBDSEph(sat, realEpoch);

        if (debug) {
            cout << "RinexNavStore::BDS eph:" << endl;
            bdsEph.printData();
        }

        xvt = bdsEph.svXvt(realEpoch);

        if (debug) {
            cout << "RinexNavStore::xvt:" << endl;
            cout << xvt << endl;
        }
    }
    else {
        InvalidRequest e("RinexNavStore: don't support the input satellite system!");
        throw (e);
    }

    return xvt;
}

NavEphGPS RinexNavStore::findGPSEph(const SatID &sat, const CommonTime &epoch) {
    if (epoch.m_timeSystem!=TimeSystem::GPS)
        convertTimeSystem(epoch, TimeSystem::GPS);
    GPSWeekSecond targetWS;
    CommonTime2WeekSecond(epoch, targetWS);
    NavEphGPS gpsEph;
    int check=0;

    // todo:
    // 这里应该改进，寻找最接近的历元的卫星星历
    for (auto it: gpsEphData[sat]) {
        GPSWeekSecond ws;
        CommonTime2WeekSecond(it.first, ws);
        double diff = ws.sow +ws.week*86400- targetWS.sow-targetWS.week*86400;
        gpsEph = it.second;
        //星历更新间隔是两个小时，所以如果在一个小时之内的，那么就比较近。
        if (diff >= -3601.0 && diff <= 3601.0) {

            check=1;
            break;
        }
    }
    if (!check)
        cout<<"It is the final NavEph.Maybe some problem happen."<<endl;


    return gpsEph;
}

NavEphBDS RinexNavStore::findBDSEph(const SatID &sat, const CommonTime &epoch) {
    if (epoch.m_timeSystem!=TimeSystem::BDT)
        convertTimeSystem(epoch,TimeSystem::BDT);

    BDTWeekSecond targetWS;
    CommonTime2WeekSecond(epoch, targetWS);
    NavEphBDS bdsEph;
    int check=0;
    // todo:
    // 这里应该改进，寻找最接近的历元的卫星星历
    for (auto it: bdsEphData[sat]) {
        BDTWeekSecond ws;
        CommonTime2WeekSecond(it.first, ws);
        double diff = ws.sow +ws.week*86400- targetWS.sow-targetWS.week*86400;
        bdsEph = it.second;
        //星历更新间隔是两个小时，所以如果在一个小时之内的，那么就比较近。
        if (diff >= -3601.0 && diff <= 3601.0) {

            check=1;
            break;
        }
    }
    if (!check) {
        cout<<"It is the final NavEph.Maybe some problem happen."<<endl;
        cout<<sat<<endl;
    }
    return bdsEph;
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




void RinexNavStore::writeFile(std::string filename,std::string name) {
    ofstream fout(filename.c_str(),ios::out);
    if (!fout) {
        cerr << "Unable to open file for writing" << endl;
        return;
    }
    //fout<<setw(10)<<"SatID"<<setw(50)<<"Commonime"<<setw(60)<<"x"<<setw(60)<<"v"<<setw(20)<<"bias"<<setw(20)<<"RelCorr"<<endl;
    if (name=="GPS")
    {
        for (auto it:this->gpscontrastDataSet) {
            SatID satID=it.first;
            TimeSequence time_sequence=it.second.X_diff;
            for (auto it2:time_sequence) {
                // 在循环开始前设置一次浮点输出格式（可选）
                fout << std::fixed << std::setprecision(15);

                // 输出一行数据，字段间用空格分隔
                fout << satID << ' '
                     << it2.first << ' '
                     << it2.second(0,0) << ' '
                     << it2.second(1,0) << ' '
                     << it2.second(2,0) << ' '
                     << it.second.V_diff[it2.first](0,0) << ' '
                     << it.second.V_diff[it2.first](1,0) << ' '
                     << it.second.V_diff[it2.first](2,0) << ' '
                     << it.second.Clockbias_diff[it2.first] << ' '
                     << it.second.RelCorr_diff[it2.first] << std::endl;
            }
        }


    }
    else if (name=="BDS")
    {
        for (auto it:this->bdscontrastDataSet) {
            SatID satID=it.first;
            TimeSequence time_sequence=it.second.X_diff;
            for (auto it2:time_sequence) {
                // 在循环开始前设置一次浮点输出格式（可选）
                fout << std::fixed << std::setprecision(15);

                // 输出一行数据，字段间用空格分隔
                fout << satID << ' '
                     << it2.first << ' '
                     << it2.second(0,0) << ' '
                     << it2.second(1,0) << ' '
                     << it2.second(2,0) << ' '
                     << it.second.V_diff[it2.first](0,0) << ' '
                     << it.second.V_diff[it2.first](1,0) << ' '
                     << it.second.V_diff[it2.first](2,0) << ' '
                     << it.second.Clockbias_diff[it2.first] << ' '
                     << it.second.RelCorr_diff[it2.first] << std::endl;
            }
        }
    }
}








