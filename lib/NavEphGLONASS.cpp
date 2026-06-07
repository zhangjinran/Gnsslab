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

#include <string>
#include <iomanip>
#include "NavEphGLONASS.hpp"
#include "CoordStruct.h"
#define debug 0

using namespace std;

void NavEphGLONASS::printData() const {
    cout << "****************************************************************"
         << "************" << endl
         << "GLONASS Broadcast Ephemeris Data: " << endl;
    cout << "Satellite PRN: " << satPrn << endl;
    cout << "Frequency Number: " << freqNum << endl;
    cout << "Health Status: " << health << endl;
    
    cout << "Toc: " << this->CivilToc.year << " " << this->CivilToc.month << " "
         << this->CivilToc.day << " " << this->CivilToc.hour << " "
         << this->CivilToc.minute << " " << this->CivilToc.second << endl;
    cout << "Day Number: " << dayNumber << endl;
    cout << "Year: " << year << endl;

    cout << scientific << setprecision(8)
         << "tau_n: " << setw(16) << tau_n << endl
         << "gamma_n: " << setw(16) << gamma_n << endl
         << "dtau_n: " << setw(16) << dtau_n << endl;

    cout << "Position (ECEF): " << endl
         << "  X: " << setw(16) << X << endl
         << "  Y: " << setw(16) << Y << endl
         << "  Z: " << setw(16) << Z << endl;

    cout << "Velocity (ECEF): " << endl
         << "  Vx: " << setw(16) << Vx << endl
         << "  Vy: " << setw(16) << Vy << endl
         << "  Vz: " << setw(16) << Vz << endl;

    cout << "Acceleration (ECEF): " << endl
         << "  Ax: " << setw(16) << Ax << endl
         << "  Ay: " << setw(16) << Ay << endl
         << "  Az: " << setw(16) << Az << endl;

    cout << "Age of Data: " << setw(16) << ageOfData << endl;
    cout << "Frequency Bias: " << setw(16) << freqBias << endl;
    cout << "Satellite Type: " << svType << endl;

    cout << "ctToc: " << ctToc.toString() << endl;
    cout << "ctToe: " << ctToe.toString() << endl;
}

double NavEphGLONASS::svClockBias(const CommonTime &t) const {
    double elaptc = t - ctToc;
    double dtc = tau_n + elaptc * (dtau_n + elaptc * 0.0);
    return dtc;
}

double NavEphGLONASS::svClockDrift(const CommonTime &t) const {
    return dtau_n;
}

long double NavEphGLONASS::svRelativity(const CommonTime &t) const
{
    // 调用带位置速度参数的版本
    Xvt xvt = svXvt(t);
    Eigen::Vector3d r(xvt.x[0], xvt.x[1], xvt.x[2]);
    Eigen::Vector3d v(xvt.v[0], xvt.v[1], xvt.v[2]);
    return svRelativity(t, r, v);
}

long double NavEphGLONASS::svRelativity(const CommonTime& t, Eigen::Vector3d r, Eigen::Vector3d v) const
{
    double rv =
        r[0] * v[0] +
        r[1] * v[1] +
        r[2] * v[2];

    static int count = 0;
    if (count++ % 100 == 0&&debug) {
        double dtr = -rv / (C_MPS * C_MPS);
        double rangeCorr = -rv / C_MPS;
        cout << fixed << setprecision(6)
             << "GLONASS Relativity Debug: " << endl
             << "  r.norm() = " << r.norm() / 1000.0 << " km" << endl
             << "  v.norm() = " << v.norm() << " m/s" << endl
             << "  r.dot(v) = " << rv / 1000.0 << " km*m/s" << endl
             << "  dtr = " << dtr * 1e9 << " ns" << endl
             << "  rangeCorr = " << rangeCorr << " m" << endl;
    }

    return -2* rv / (C_MPS*C_MPS);
}

double NavEphGLONASS::svURA(const CommonTime &t) const {
    return 0.0;
}

Xvt NavEphGLONASS::svXvt(const CommonTime &t) const
{
    Xvt sv;

    //------------------------------------------------------------------
    // GLONASS uses PZ-90 Earth constants
    //------------------------------------------------------------------
    PZ90 ell;

    const double mu      = ell.getGM();      // [m^3/s^2]
    const double J2      = ell.getJ2();
    const double AE      = ell.getA();       // [m]
    const double OMEGA_E = ell.getOmega();   // [rad/s]

    //------------------------------------------------------------------
    // Time difference
    //------------------------------------------------------------------
    double tk = t - ctToe;

    if (debug)
    {
        cout << "ctToe:" << ctToe.toString() << endl;
        cout << "t:" << t.toString() << endl;
        cout << "tk:" << tk << endl;
    }

    //------------------------------------------------------------------
    // Clock
    //------------------------------------------------------------------
    sv.clkbias  = svClockBias(t);
    sv.clkdrift = svClockDrift(t);


    //------------------------------------------------------------------
    // Initial state vector
    //------------------------------------------------------------------
    Eigen::Vector3d r(X,  Y,  Z);
    Eigen::Vector3d v(Vx, Vy, Vz);
    Eigen::Vector3d a(Ax, Ay, Az);

    //------------------------------------------------------------------
    // Numerical integration
    // RK4 integration
    //------------------------------------------------------------------
    double step = 30.0; // 30 sec
    int nstep = static_cast<int>(fabs(tk) / step);

    double remain = fabs(tk) - nstep * step;

    if (tk < 0.0)
        step = -step;

    //------------------------------------------------------------------
    // Acceleration model
    //------------------------------------------------------------------
    auto accel = [&](const Eigen::Vector3d& rr,
                     const Eigen::Vector3d& vv)
    {
        double x = rr(0);
        double y = rr(1);
        double z = rr(2);

        double r2 = rr.squaredNorm();
        double r1 = sqrt(r2);

        double zx = z / r1;
        double factorJ2 =
            1.5 * J2 * mu * AE * AE / pow(r1, 5);

        double ax_j2 =
            factorJ2 * x * (5.0 * zx * zx - 1.0);

        double ay_j2 =
            factorJ2 * y * (5.0 * zx * zx - 1.0);

        double az_j2 =
            factorJ2 * z * (5.0 * zx * zx - 3.0);

        //------------------------------------------------------------------
        // Central gravity
        //------------------------------------------------------------------
        Eigen::Vector3d ag =
            -mu / pow(r1, 3) * rr;

        //------------------------------------------------------------------
        // J2 perturbation
        //------------------------------------------------------------------
        Eigen::Vector3d aj2(ax_j2, ay_j2, az_j2);

        //------------------------------------------------------------------
        // Earth rotation terms
        //------------------------------------------------------------------
        Eigen::Vector3d omega(0.0, 0.0, OMEGA_E);

        Eigen::Vector3d acor =
            -2.0 * omega.cross(vv)
            - omega.cross(omega.cross(rr));

        //------------------------------------------------------------------
        // Broadcast luni-solar acceleration
        //------------------------------------------------------------------
        Eigen::Vector3d atotal =
            ag + aj2 + acor + a;

        return atotal;
    };

    //------------------------------------------------------------------
    // RK4 propagate
    //------------------------------------------------------------------
    for (int i = 0; i < nstep; i++)
    {
        Eigen::Vector3d k1_r = v;
        Eigen::Vector3d k1_v = accel(r, v);

        Eigen::Vector3d k2_r =
            v + 0.5 * step * k1_v;

        Eigen::Vector3d k2_v =
            accel(r + 0.5 * step * k1_r,
                  v + 0.5 * step * k1_v);

        Eigen::Vector3d k3_r =
            v + 0.5 * step * k2_v;

        Eigen::Vector3d k3_v =
            accel(r + 0.5 * step * k2_r,
                  v + 0.5 * step * k2_v);

        Eigen::Vector3d k4_r =
            v + step * k3_v;

        Eigen::Vector3d k4_v =
            accel(r + step * k3_r,
                  v + step * k3_v);

        r += step / 6.0 *
             (k1_r + 2.0 * k2_r +
              2.0 * k3_r + k4_r);

        v += step / 6.0 *
             (k1_v + 2.0 * k2_v +
              2.0 * k3_v + k4_v);
    }

    //------------------------------------------------------------------
    // Remaining fractional step
    //------------------------------------------------------------------
    if (remain > 1e-6)
    {
        if (tk < 0.0)
            remain = -remain;

        Eigen::Vector3d k1_r = v;
        Eigen::Vector3d k1_v = accel(r, v);

        Eigen::Vector3d k2_r =
            v + 0.5 * remain * k1_v;

        Eigen::Vector3d k2_v =
            accel(r + 0.5 * remain * k1_r,
                  v + 0.5 * remain * k1_v);

        Eigen::Vector3d k3_r =
            v + 0.5 * remain * k2_v;

        Eigen::Vector3d k3_v =
            accel(r + 0.5 * remain * k2_r,
                  v + 0.5 * remain * k2_v);

        Eigen::Vector3d k4_r =
            v + remain * k3_v;

        Eigen::Vector3d k4_v =
            accel(r + remain * k3_r,
                  v + remain * k3_v);

        r += remain / 6.0 *
             (k1_r + 2.0 * k2_r +
              2.0 * k3_r + k4_r);

        v += remain / 6.0 *
             (k1_v + 2.0 * k2_v +
              2.0 * k3_v + k4_v);
    }

    //------------------------------------------------------------------
    // Output
    //------------------------------------------------------------------
    sv.x[0] = r(0);
    sv.x[1] = r(1);
    sv.x[2] = r(2);

    sv.v[0] = v(0);
    sv.v[1] = v(1);
    sv.v[2] = v(2);
    sv.relcorr  = svRelativity(t,r,v);

    return sv;
}
bool NavEphGLONASS::isValid(const CommonTime &ct) const {
    if (ct.m_timeSystem != TimeSystem::GLO) return false;
    if (ct < beginValid || ct > endValid) return false;
    return true;
}