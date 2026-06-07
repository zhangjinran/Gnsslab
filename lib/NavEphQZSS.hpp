#ifndef NavEphQZSS_HPP
#define NavEphQZSS_HPP

#include <string>
#include <cmath>
#include "TimeConvert.h"
#include "GnssStruct.h"
#include "NavEphBase.hpp"

class NavEphQZSS : public NavEphBase {
public:
    NavEphQZSS(void) {
        beginValid = END_OF_TIME;
        endValid = BEGINNING_OF_TIME;
        beginValid.m_timeSystem = TimeSystem::QZS;
        endValid.m_timeSystem = TimeSystem::QZS;
    }

    virtual ~NavEphQZSS(void) {}

    void printData() const;

    double svClockBias(const CommonTime &t) const;

    double svClockDrift(const CommonTime &t) const;

    long double svRelativity(const CommonTime &t) const;

    double svURA(const CommonTime &t) const;

    Xvt svXvt(const CommonTime &t) const;

    bool isValid(const CommonTime &ct) const;

    CivilTime CivilToc;
    double Toc;
    double af0;
    double af1;
    double af2;

    double IODE;
    double Crs;
    double Delta_n;
    double M0;

    double Cuc;
    double ecc;
    double Cus;
    double sqrt_A;

    double Toe;
    double Cic;
    double OMEGA_0;
    double Cis;

    double i0;
    double Crc;
    double omega;
    double OMEGA_DOT;

    double IDOT;
    double L2Codes;
    double QZSSWeek;
    double L2Pflag;

    double URA;
    double SV_health;
    double TGD;
    double IODC;

    long HOWtime;
    double fitInterval;

    virtual TimeSystem getTimeSystem() const override { return TimeSystem::QZS; }
    virtual std::string getSystemCode() const override { return "J"; }

private:
    static short getFitInterval(const short IODC, const short fitIntFlag);
};

#endif // NavEphQZSS_HPP