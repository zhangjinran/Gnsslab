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
 * 2. Galileo Open Service Signal-In-Space Interface Control Document (OS SIS ICD)
 */

#ifndef NavEphGalileo_HPP
#define NavEphGalileo_HPP

#include <string>
#include <cmath>

#include "TimeConvert.h"
#include "GnssStruct.h"
#include "NavEphBase.hpp"

class NavEphGalileo : public NavEphBase {
public:
    /// Default constructor
    NavEphGalileo(void)
            : beginValid(END_OF_TIME),
              endValid(BEGINNING_OF_TIME) {
        beginValid.m_timeSystem = TimeSystem::GAL;
        endValid.m_timeSystem = TimeSystem::GAL;
        ctToc.setTimeSystem(TimeSystem::GAL);
        ctToe.setTimeSystem(TimeSystem::GAL);
        transmitTime.setTimeSystem(TimeSystem::GAL);
    }

    /// Destructor
    virtual ~NavEphGalileo(void) {}

    /// Dump the overhead information to the given output stream.
    void printData() const;

    /// Compute the satellite clock bias (seconds) at the given time
    double svClockBias(const CommonTime &t) const;

    /// Compute the satellite clock drift (sec/sec) at the given time
    double svClockDrift(const CommonTime &t) const;

    /// Compute satellite relativity correction (sec) at the given time
    long double svRelativity(const CommonTime &t) const;

    /// return URA of broadcast
    double svURA(const CommonTime &t) const;

    /// Compute satellite position at the given time.
    Xvt svXvt(const CommonTime &t) const;

    bool isValid(const CommonTime &ct) const;

    /// Ephemeris data - Galileo specific parameters

    /// SV/EPOCH/SV CLK
    CivilTime CivilToc;
    double Toc;                ///< Time of clock (year/month/day/hour/min/sec GAL)
    double af0;                ///< SV clock bias (seconds)
    double af1;                ///< SV clock drift (sec/sec)
    double af2;                ///< SV clock drift rate (sec/sec2)

    /// BROADCAST ORBIT-1
    double IODE;               ///< IODE Issue of Data, Ephemeris
    double Crs;                ///< (meters)
    double Delta_n;            ///< Mean Motion Difference From Computed Value (semi-circles/sec)
    double M0;                 ///< Mean Anomaly at Reference Time (semi-circles)

    /// BROADCAST ORBIT-2
    double Cuc;                ///< (radians)
    double ecc;                ///< Eccentricity
    double Cus;                ///< (radians)
    double sqrt_A;             ///< Square Root of the Semi-Major Axis (sqrt(m))

    /// BROADCAST ORBIT-3
    double Toe;                ///< Time of Ephemeris (sec of Galileo week)
    double Cic;                ///< (radians)
    double OMEGA_0;            ///< Longitude of Ascending Node of Orbit Plane (semi-circles)
    double Cis;                ///< (radians)

    /// BROADCAST ORBIT-4
    double i0;                 ///< Inclination Angle at Reference Time (semi-circles)
    double Crc;                ///< (meters)
    double omega;              ///< Argument of Perigee (semi-circles)
    double OMEGA_DOT;          ///< Rate of Right Ascension (semi-circles/sec)

    /// BROADCAST ORBIT-5
    double IDOT;               ///< Rate of Inclination Angle (semi-circles/sec)
    double GalileoWeek;        ///< Galileo week number (continuous)
    double SISA;               ///< Signal-in-space accuracy (meters)

    /// BROADCAST ORBIT-6
    double SV_health;          ///< Satellite health status
    double BGD_E5aE1;          ///< BGD(E5a,E1) - Broadcast Group Delay (seconds)
    double BGD_E5bE1;          ///< BGD(E5b,E1) - Broadcast Group Delay (seconds)

    /// BROADCAST ORBIT-7
    long HOWtime;              ///< Transmission time of message (sec of Galileo Week)
    double fitInterval;        ///< Fit Interval in hours

    /// member data
    CommonTime ctToc;          ///< Toc in CommonTime form
    CommonTime ctToe;          ///< Toe in CommonTime form
    CommonTime transmitTime;   ///< Transmission time in CommonTime form
    CommonTime beginValid;     ///< Time at beginning of validity
    CommonTime endValid;       ///< Time at end of fit validity

    /// 实现 NavEphBase 接口
    virtual TimeSystem getTimeSystem() const override { return TimeSystem::GAL; }
    virtual std::string getSystemCode() const override { return "E"; }

private:
    /// Get the fit interval in hours from the fit interval flag and the IODC
    static short getFitInterval(const short IODC, const short fitIntFlag);

}; // end class NavEphGalileo

#endif // NavEphGalileo_HPP