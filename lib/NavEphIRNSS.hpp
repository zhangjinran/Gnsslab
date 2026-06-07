#ifndef NavEphIRNSS_HPP
#define NavEphIRNSS_HPP

#include <string>
#include <cmath>
#include "TimeConvert.h"
#include "GnssStruct.h"
#include "NavEphBase.hpp"

class NavEphIRNSS : public NavEphBase {
public:
    NavEphIRNSS(void) {
        beginValid = END_OF_TIME;
        endValid = BEGINNING_OF_TIME;
        beginValid.m_timeSystem = TimeSystem::IRN;
        endValid.m_timeSystem = TimeSystem::IRN;
    }

    virtual ~NavEphIRNSS(void) {}

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
    double IRNSSWeek;

    double URA;
    double SV_health;
    double TGD;
    double IODC;

    long HOWtime;
    double fitInterval;

    virtual TimeSystem getTimeSystem() const override { return TimeSystem::IRN; }
    virtual std::string getSystemCode() const override { return "I"; }

private:
    static short getFitInterval(const short IODC, const short fitIntFlag);
};

#endif // NavEphIRNSS_HPP