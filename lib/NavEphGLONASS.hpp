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
 * 2. GLONASS Interface Control Document (ICD)
 */

#ifndef NavEphGLONASS_HPP
#define NavEphGLONASS_HPP

#include <string>
#include <cmath>

#include "TimeConvert.h"
#include "GnssStruct.h"
#include "NavEphBase.hpp"

class NavEphGLONASS : public NavEphBase {
public:
    /// Default constructor
    NavEphGLONASS(void)
            : beginValid(END_OF_TIME),
              endValid(BEGINNING_OF_TIME) {
        beginValid.m_timeSystem = TimeSystem::GLO;
        endValid.m_timeSystem = TimeSystem::GLO;
        ctToc.setTimeSystem(TimeSystem::GLO);
        ctToe.setTimeSystem(TimeSystem::GLO);
        transmitTime.setTimeSystem(TimeSystem::GLO);
    }

    /// Destructor
    virtual ~NavEphGLONASS(void) {}

    /// Dump the overhead information to the given output stream.
    void printData() const;

    /// Compute the satellite clock bias (seconds) at the given time
    double svClockBias(const CommonTime &t) const;

    /// Compute the satellite clock drift (sec/sec) at the given time
    double svClockDrift(const CommonTime &t) const;

    /// Compute satellite relativity correction (sec) at the given time
    long double svRelativity(const CommonTime &t) const override;
    long double svRelativity(const CommonTime &t, Eigen::Vector3d r, Eigen::Vector3d v) const;

    /// return URA of broadcast
    double svURA(const CommonTime &t) const;

    /// Compute satellite position at the given time.
    Xvt svXvt(const CommonTime &t) const;

    bool isValid(const CommonTime &ct) const;

    /// Ephemeris data - GLONASS specific parameters
    
    /// Header information
    int satPrn;                 ///< Satellite PRN number (1-24)
    int freqNum;               ///< Frequency number (-7 to +12)
    int health;                ///< Health status (0=healthy)
    
    /// Time parameters
    CivilTime CivilToc;
    double Toc;                ///< Time of clock (sec of day)
    int dayNumber;             ///< Day number (1-366)
    int year;                  ///< Year (last two digits)
    
    /// Clock parameters
    double tau_n;              ///< Satellite clock bias (seconds)
    double gamma_n;            ///< Relative frequency bias
    double dtau_n;             ///< Clock drift rate (sec/sec)
    
    /// Position parameters (ECEF, meters)
    double X;                  ///< X coordinate
    double Y;                  ///< Y coordinate
    double Z;                  ///< Z coordinate
    
    /// Velocity parameters (ECEF, m/s)
    double Vx;                 ///< X velocity
    double Vy;                 ///< Y velocity
    double Vz;                 ///< Z velocity
    
    /// Acceleration parameters (ECEF, m/s^2)
    double Ax;                 ///< X acceleration
    double Ay;                 ///< Y acceleration
    double Az;                 ///< Z acceleration
    
    /// Additional parameters
    double ageOfData;          ///< Age of data (days)
    double freqBias;           ///< Frequency bias
    int svType;                ///< Satellite type (0=GLONASS, 1=GLONASS-M)
    
    /// member data
    CommonTime ctToc;          ///< Toc in CommonTime form
    CommonTime ctToe;          ///< Toe in CommonTime form  
    CommonTime transmitTime;   ///< Transmission time in CommonTime form
    CommonTime beginValid;     ///< Time at beginning of validity
    CommonTime endValid;       ///< Time at end of fit validity

    /// 实现 NavEphBase 接口
    virtual TimeSystem getTimeSystem() const override { return TimeSystem::GLO; }
    virtual std::string getSystemCode() const override { return "R"; }

    /// 根据频率槽号计算实际频率 (Hz)
    /// GLONASS FDMA 频率计算公式：
    /// f1 = (1602.0 + 0.5625 * k) MHz
    /// f2 = (1246.0 + 0.4375 * k) MHz
    /// @param freqType 频率类型 "L1" 或 "L2"
    /// @return 实际频率 (Hz)
    double getFreq(const std::string& freqType) const {
        if (freqType == "L1" || freqType == "C1") {
            return (1602.0 + 0.5625 * freqNum) * 1e6;
        } else if (freqType == "L2" || freqType == "C2") {
            return (1246.0 + 0.4375 * freqNum) * 1e6;
        } else if (freqType == "L3" || freqType == "C3") {
            // G3 是 CDMA 信号，频率固定
            return 1202.025e6;
        }
        return 0.0;
    }

}; // end class NavEphGLONASS

#endif // NavEphGLONASS_HPP