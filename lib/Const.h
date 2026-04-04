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
#pragma once

#include <string>
#include <iostream>
#include <string>

using namespace std;

//==============
// 数学与物理常数
//====================

//  PI
const double PI = 3.141592653589793238462643383280;
// m/s, speed of light; this value defined by GPS but applies to GAL and GLO.
const double C_MPS = 2.99792458e8;
// Conversion Factor from degrees to radians (unit: degrees^-1)
static const double DEG_TO_RAD = 1.745329251994329576923691e-2;
// Conversion Factor from radians to degrees (unit: degrees)
static const double RAD_TO_DEG = 57.29577951308232087679815;
/// relativity constant (sec/sqrt(m))
const double REL_CONST = -4.442807633e-10;
/// relativity constant for BDS (sec/sqrt(m))
const double REL_CONST_BDS = -4.442807309e-10;

//------------------
//  时间相关常数
//-------------------

/// Add this offset to convert Modified Julian Date to Julian Date.
const double MJD_TO_JD = 2400000.5;
///'Julian day'offset from JD2020
const double JD2020_TO_JD = MJD_TO_JD+58849;
/// 'Julian day' offset from MJD
const long MJD_JDAY = 2400001L;
/// 'Julian day' offset from JD2020
const long JD2020_JDAY = 2400001L+58849;
/// Modified Julian Date of UNIX epoch (Jan. 1, 1970).
const long UNIX_MJD = 40587L;

/// Seconds per half week.
const long HALFWEEK = 302400L;
/// Seconds per whole week.
const long FULLWEEK = 604800L;

/// Seconds per day.
const long SEC_PER_DAY = 86400L;
/// Days per second.
const double DAY_PER_SEC = 1.0 / SEC_PER_DAY;

/// Milliseconds in a second.
const long MS_PER_SEC = 1000L;
/// Seconds per millisecond.
const double SEC_PER_MS = 1.0 / MS_PER_SEC;

/// Milliseconds in a day.
const long MS_PER_DAY = MS_PER_SEC * SEC_PER_DAY;
/// Days per milliseconds.
const double DAY_PER_MS = 1.0 / MS_PER_DAY;

// Nominal mean angular velocity of the Earth (rad/s)
const double OMEGA_EARTH  = 7.292115e-5;

const double RadiusEarth = 6378137.0;
// system-specific constants

// GPS -------------------------------------------
/// 'Julian day' of GPS epoch (Jan. 6, 1980).
const double GPS_EPOCH_JD = 2444244.5;
/// Modified Julian Date of GPS epoch (Jan. 6, 1980).
const long GPS_EPOCH_MJD = 44244L;
/// Weeks per GPS Epoch
const long GPS_WEEK_PER_EPOCH = 1024L;

/// Zcounts in a  day.
const long ZCOUNT_PER_DAY = 57600L;
/// Days in a Zcount
const double DAY_PER_ZCOUNT = 1.0 / ZCOUNT_PER_DAY;
/// Zcounts in a week.
const long ZCOUNT_PER_WEEK = 403200L;
/// Weeks in a Zcount.
const double WEEK_PER_ZCOUNT = 1.0 / ZCOUNT_PER_WEEK;

// BDS -------------------------------------------
/// 'Julian day' of BDS epoch (Jan. 1, 2006).
const double BDS_EPOCH_JD = 2453736.5;
/// Modified Julian Date of BDS epoch (Jan. 1, 2006).
const long BDS_EPOCH_MJD = 53736L;
/// Weeks per BDS Epoch
const long BDS_WEEK_PER_EPOCH = 8192L;

//===================================
// GNSS 系统相关常量
//===================================

// GPS L1 carrier frequency in Hz
const double L1_FREQ_GPS = 1575.42e6;
// GPS L2 carrier frequency in Hz
const double L2_FREQ_GPS = 1227.60e6;
// GPS L5 carrier frequency in Hz
const double L5_FREQ_GPS = 1176.45e6;

// GPS L1 carrier wavelength in meters
const double L1_WAVELENGTH_GPS = 0.190293672798;
// GPS L2 carrier wavelength in meters
const double L2_WAVELENGTH_GPS = 0.244210213425;
// GPS L5 carrier wavelength in meters
const double L5_WAVELENGTH_GPS = 0.254828048791;

const double L5_FREQ_BDS = 1176.450e6; // B2a (BDS-3)
const double L8_FREQ_BDS = 1191.795e6; // B2=B21+B2b/2
const double L7_FREQ_BDS = 1207.140e6; // B2b (BDS-3/BDS-2)
const double L6_FREQ_BDS = 1268.520e6; // B3  (BDS-3/BDS-2)
const double L2_FREQ_BDS = 1561.098e6; // B1I (BDS-3/BDS-2)
const double L1_FREQ_BDS = 1575.420e6; // B1C (BDS-3)

const double L1_WAVELENGTH_BDS = C_MPS / L1_FREQ_BDS;
const double L2_WAVELENGTH_BDS = C_MPS / L2_FREQ_BDS;
const double L6_WAVELENGTH_BDS = C_MPS / L6_FREQ_BDS;
const double L7_WAVELENGTH_BDS = C_MPS / L7_FREQ_BDS;
const double L8_WAVELENGTH_BDS = C_MPS / L8_FREQ_BDS;
const double L5_WAVELENGTH_BDS = C_MPS / L5_FREQ_BDS;

// ========== Galileo ==========
// Galileo E1 carrier frequency in Hz
const double L1_FREQ_GAL = 1575.42e6;   // E1
// Galileo E5a carrier frequency in Hz
const double L5_FREQ_GAL = 1176.45e6;   // E5a
// Galileo E5b carrier frequency in Hz
const double L7_FREQ_GAL = 1207.14e6;   // E5b
// Galileo E6 carrier frequency in Hz
const double L8_FREQ_GAL = 1278.75e6;   // E6

// Galileo E1 carrier wavelength in meters
const double L1_WAVELENGTH_GAL = C_MPS / L1_FREQ_GAL;
// Galileo E5a carrier wavelength in meters
const double L5_WAVELENGTH_GAL = C_MPS / L5_FREQ_GAL;
// Galileo E5b carrier wavelength in meters
const double L7_WAVELENGTH_GAL = C_MPS / L7_FREQ_GAL;
// Galileo E6 carrier wavelength in meters
const double L8_WAVELENGTH_GAL = C_MPS / L8_FREQ_GAL;

// ========== GLONASS ==========
// GLONASS G1 (FDMA) carrier frequency in Hz (k=0, 实际需根据频道号计算)
const double L1_FREQ_GLO = 1602.00e6;   // G1 with channel number 0
// GLONASS G2 (FDMA) carrier frequency in Hz (k=0, 实际需根据频道号计算)
const double L2_FREQ_GLO = 1246.00e6;   // G2 with channel number 0
// GLONASS G3 (CDMA) carrier frequency in Hz
const double L3_FREQ_GLO = 1202.025e6;  // G3

// GLONASS G1 approximate wavelength (for k=0)
const double L1_WAVELENGTH_GLO = C_MPS / L1_FREQ_GLO;
// GLONASS G2 approximate wavelength (for k=0)
const double L2_WAVELENGTH_GLO = C_MPS / L2_FREQ_GLO;
// GLONASS G3 carrier wavelength in meters
const double L3_WAVELENGTH_GLO = C_MPS / L3_FREQ_GLO;

// ========== QZSS ==========
// QZSS L1 carrier frequency in Hz
const double L1_FREQ_QZSS = 1575.42e6;  // L1
// QZSS L2 carrier frequency in Hz
const double L2_FREQ_QZSS = 1227.60e6;  // L2
// QZSS L5 carrier frequency in Hz
const double L5_FREQ_QZSS = 1176.45e6;  // L5

// QZSS L1 carrier wavelength in meters
const double L1_WAVELENGTH_QZSS = C_MPS / L1_FREQ_QZSS;
// QZSS L2 carrier wavelength in meters
const double L2_WAVELENGTH_QZSS = C_MPS / L2_FREQ_QZSS;
// QZSS L5 carrier wavelength in meters
const double L5_WAVELENGTH_QZSS = C_MPS / L5_FREQ_QZSS;

// ========== IRNSS ==========
// IRNSS L1 carrier frequency in Hz
const double L1_FREQ_IRNSS = 1575.42e6;  // L1
// IRNSS L5 carrier frequency in Hz
const double L5_FREQ_IRNSS = 1176.45e6;  // L5

// IRNSS L1 carrier wavelength in meters
const double L1_WAVELENGTH_IRNSS = C_MPS / L1_FREQ_IRNSS;
// IRNSS L5 carrier wavelength in meters
const double L5_WAVELENGTH_IRNSS = C_MPS / L5_FREQ_IRNSS;

inline double getWavelength(const std::string &sys, const int &n)
throw() {
    if (n == 0) {
        std::cerr << "getWavelength():frequency no must be positive integer!" << endl;
        exit(-1);
    }

    if (sys == "G") {
        if (n == 1) return L1_WAVELENGTH_GPS;
        else if (n == 2) return L2_WAVELENGTH_GPS;
        else if (n == 5) return L5_WAVELENGTH_GPS;
    } else if (sys == "C") {
        if (n == 1) return L1_WAVELENGTH_BDS;
        else if (n == 2) return L2_WAVELENGTH_BDS;
        else if (n == 5) return L5_WAVELENGTH_BDS;
        else if (n == 7) return L7_WAVELENGTH_BDS;
        else if (n == 8) return L8_WAVELENGTH_BDS;
        else if (n == 6) return L6_WAVELENGTH_BDS;
    }
    else if (sys == "R") {
        if (n == 1) return L1_WAVELENGTH_GLO;
        else if (n == 2) return L2_WAVELENGTH_GLO;
        else if (n == 3) return L3_WAVELENGTH_GLO;
    }
    else if (sys == "E") {
        if (n == 1) return L1_WAVELENGTH_GAL;
        else if (n==5)return L5_WAVELENGTH_GAL;
        else if (n==7) return L7_WAVELENGTH_GAL;
        else if (n==8) return L8_WAVELENGTH_GAL;
    }
    else if (sys == "J") {
        if (n == 1) return L1_WAVELENGTH_QZSS;
        else if (n == 2) return L2_WAVELENGTH_QZSS;
        else if (n == 5) return L5_WAVELENGTH_QZSS;
    }
    else if (sys == "I") {
        if (n == 1) return L1_WAVELENGTH_IRNSS;
        else if (n == 5) return L5_WAVELENGTH_IRNSS;
    }
    else if (sys == "S") {  // SBAS
        if (n == 1) return L1_WAVELENGTH_GPS;   // SBAS L1
        else if (n == 5) return L5_WAVELENGTH_GPS; // SBAS L5 (如果支持)
    }
    else {
        std::cerr << "don't support this system" << endl;
    }

    return 0.0;
}

inline double getFreq(const string &sys, const int &n)
throw() {

    if (sys == "G") {
        if (n == 1) return L1_FREQ_GPS;
        else if (n == 2) return L2_FREQ_GPS;
        else if (n == 5) return L5_FREQ_GPS;
    } else if (sys == "C") {
        if (n == 1) return L1_FREQ_BDS;
        else if (n == 2) return L2_FREQ_BDS;
        else if (n == 5) return L5_FREQ_BDS;
        else if (n == 7) return L7_FREQ_BDS;
        else if (n == 8) return L8_FREQ_BDS;
        else if (n == 6) return L6_FREQ_BDS;
    } else {
        std::cerr << "don't support system except GPS and Beidou" << endl;
    }
    return 0.0;
}

inline double getFreq(const string &sys, const string &type)
throw() {

    if (sys == "G") {
        if (type == "L1" || type == "C1") return L1_FREQ_GPS;
        else if (type == "L2" || type == "C2") return L2_FREQ_GPS;
        else if (type == "L5" || type == "C5") return L5_FREQ_GPS;
    } else if (sys == "C") {
        if (     type == "L1"|| type == "C1") return L1_FREQ_BDS;
        else if (type == "L2"|| type == "C2") return L2_FREQ_BDS;
        else if (type == "L5"|| type == "C5") return L5_FREQ_BDS;
        else if (type == "L7"|| type == "C7") return L7_FREQ_BDS;
        else if (type == "L8"|| type == "C8") return L8_FREQ_BDS;
        else if (type == "L6"|| type == "C6") return L6_FREQ_BDS;
    } else {
        std::cerr << "don't support system except GPS and Beidou" << endl;
    }
    return 0.0;
}

inline double getGamma(const string &sys,
                       const string &type1,
                       const string &type2
                       )
{
    double f1 = getFreq(sys, type1);
    double f2 = getFreq(sys, type2);
    double gamma;
    gamma = (f1*f1)/(f2*f2);
    return gamma;
};
