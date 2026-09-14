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


#include <string>
#include "NavEphBDS.hpp"

using namespace std;

void NavEphBDS::printData() const {
    cout << "****************************************************************"
         << "************" << endl
         << "BDS Broadcast Ephemeris Data: " << endl;
    cout << "Toc: " << this->CivilToc.year << " " << this->CivilToc.month << " "
         << this->CivilToc.day << " " << this->CivilToc.hour << " "
         << this->CivilToc.minute << " " << this->CivilToc.second << endl;
    cout << scientific << setprecision(8)
         << "af0: " << setw(16) << af0 << endl
         << "af1: " << setw(16) << af1 << endl
         << "af2: " << setw(16) << af2 << endl;

    cout << "IODE: " << setw(16) << IODE << endl
         << "Crs:  " << setw(16) << Crs << endl
         << "Delta_n: " << setw(16) << Delta_n << endl
         << "M0: " << setw(16) << M0 << endl;

    cout << "Cuc: " << setw(16) << Cuc << endl
         << "ecc: " << setw(16) << ecc << endl
         << "Cus: " << setw(16) << Cus << endl
         << "sqrt_A: " << setw(16) << sqrt_A << endl;

    cout << "Toe: " << setw(16) << Toe << endl;
    cout << "Cic: " << setw(16) << Cic << endl;
    cout << "OMEGA_0: " << setw(16) << OMEGA_0 << endl;
    cout << "Cis: " << setw(16) << Cis << endl;

    cout << "i0: " << setw(16) << i0 << endl;
    cout << "Crc: " << setw(16) << Crc << endl;
    cout << "omega: " << setw(16) << omega << endl;
    cout << "OMEGA_DOT: " << setw(16) << OMEGA_DOT << endl;

    cout << "IDOT: " << setw(16) << IDOT << endl;
    //cout << "Codes_On_L2_Channel: " << setw(16) << L2Codes << endl;
    cout << "BDSWeek: " << setw(16) << BDSWeek << endl;
    //cout << "L2P_data_flag: " << setw(16) << L2Pflag << endl;

    cout << "URA: " << setw(16) << URA << endl;
    cout << "SV_health: " << setw(16) << SV_health << endl;
    cout << "TGD1: " << setw(16) << TGD1 << endl;
    cout << "TGD2: " << setw(16) << TGD2 << endl;
    cout << "IODC: " << setw(16) << IODC << endl;

    cout << "HOWtime: " << setw(16) << HOWtime << endl;
    cout << "fitInterval: " << setw(16) << fitInterval << endl;

    cout << "ctToc: " << ctToc.toString() << endl;
    cout << "ctToe: " << ctToe.toString() << endl;
}


double NavEphBDS::svClockBias(const CommonTime &t) const {
    if (t.m_timeSystem!=TimeSystem::BDT)
        cerr << "Timesystem should be BDT!!!" << endl;
    double dtc, elaptc;
    elaptc = t - ctToc;
    //cout << "elaptc:" <<  elaptc << endl;
    dtc = af0 + elaptc * (af1 + elaptc * af2);
    //cout << "af0:" << af0 << "af1:" << af1 << "af2:" << af2 << endl;
    //cout << "dtc:" << dtc << endl;
    return dtc;
}

double NavEphBDS::svClockDrift(const CommonTime &t) const {
    if (t.m_timeSystem!=TimeSystem::BDT)
        cerr << "Timesystem should be BDT!!!" << endl;
    double drift, elaptc;
    elaptc = t - ctToc;
    drift = af1 + elaptc * af2;
    return drift;
}

// Compute satellite relativity correction (sec) at the given time
// throw Invalid Request if the required data has not been stored.
long double NavEphBDS::svRelativity(const CommonTime &t) const {
    if (t.m_timeSystem!=TimeSystem::BDT)
        cerr << "Timesystem should be BDT!!!" << endl;
    BDSEllipsoid ell;
    ///Semi-major axis
    double A = sqrt_A * sqrt_A;

    ///Computed mean motion (rad/sec)
    double n0 = std::sqrt(ell.gm() / (A * A * A));

    ///Time from ephemeris reference epoch
    double tk = t - ctToe;
    if (tk > 302400) tk = tk - 604800;
    if (tk < -302400) tk = tk + 604800;

    ///Corrected mean motion
    double n = n0 + Delta_n;

    ///Mean anomaly
    double Mk = M0 + n * tk;

    ///Kepler's Equation for Eccentric Anomaly
    ///solved by iteration
    double twoPI = 2.0e0 * PI;
    Mk = fmod(Mk, twoPI);
    double Ek = Mk + ecc * ::sin(Mk);
    int loop_cnt = 1;
    double F, G, delea;
    do {
        F = Mk - (Ek - ecc * ::sin(Ek));
        G = 1.0 - ecc * ::cos(Ek);
        delea = F / G;
        Ek = Ek + delea;
        loop_cnt++;
    } while ((fabs(delea) > 1.0e-11) && (loop_cnt <= 20));

    return (REL_CONST_BDS * ecc * std::sqrt(A) * ::sin(Ek));
}

double NavEphBDS::svURA(const CommonTime &t) const {
    double ephURA = URA;
    return ephURA;
}

Xvt NavEphBDS::svXvt(const CommonTime &t) const {
    if (t.m_timeSystem!=TimeSystem::BDT)
        cerr << "Timesystem should be BDT!!!" << endl;
    Xvt sv;
    BDSEllipsoid ell;

    ///Semi-major axis
    double A = sqrt_A * sqrt_A;

    ///Computed mean motion (rad/sec)
    double n0 = std::sqrt(ell.gm() / (A * A * A));

    ///Time from ephemeris reference epoch
    double tk = t - ctToe;
    if (tk > 302400) tk = tk - 604800;
    if (tk < -302400) tk = tk + 604800;

    ///Corrected mean motion
    double n = n0 + Delta_n;

    ///Mean anomaly
    double Mk = M0 + n * tk;

    ///Kepler's Equation for Eccentric Anomaly
    ///solved by iteration
    double twoPI = 2.0e0 * PI;
    Mk = fmod(Mk, twoPI);
    double Ek = Mk + ecc * ::sin(Mk);
    int loop_cnt = 1;
    double F, G, delea;
    do {
        F = Mk - (Ek - ecc * ::sin(Ek));
        G = 1.0 - ecc * ::cos(Ek);
        delea = F / G;
        Ek = Ek + delea;
        loop_cnt++;
    } while ((fabs(delea) > 1.0e-11) && (loop_cnt <= 20));

    ///compute clock corrections
    sv.relcorr = svRelativity(t);
    sv.clkbias = svClockBias(t);
    sv.clkdrift = svClockDrift(t);

    ///True Anomaly
    double q = std::sqrt(1.0 - ecc * ecc);
    double sinEk = ::sin(Ek);
    double cosEk = ::cos(Ek);

    double GSTA = q * sinEk;
    double GCTA = cosEk - ecc;
    double vk = atan2(GSTA, GCTA);

    ///Eccentric Anomaly
    //Ek = std::acos((ecc+ ::cos(vk))/(1+ecc* ::cos(vk)));

    ///Argument of Latitude
    double phi_k = vk + omega;
    double cos2phi_k = ::cos(2.0 * phi_k);
    double sin2phi_k = ::sin(2.0 * phi_k);

    double duk = cos2phi_k * Cuc + sin2phi_k * Cus;
    double drk = cos2phi_k * Crc + sin2phi_k * Crs;
    double dik = cos2phi_k * Cic + sin2phi_k * Cis;

    double uk = phi_k + duk;
    double rk = A * (1.0 - ecc * cosEk) + drk;
    double ik = i0 + dik + IDOT * tk;

    ///Positions in orbital plane.
    double xip = rk * ::cos(uk);
    double yip = rk * ::sin(uk);

    ///Corrected longitude of ascending node.
    double OMEGA_k = OMEGA_0 + (OMEGA_DOT - ell.angVelocity()) * tk
                     - ell.angVelocity() * Toe;

    ///Earth-fixed coordinates.
    double sinOMG_k = ::sin(OMEGA_k);
    double cosOMG_k = ::cos(OMEGA_k);
    double cosik = ::cos(ik);
    double sinik = ::sin(ik);

    double xef = xip * cosOMG_k - yip * cosik * sinOMG_k;
    double yef = xip * sinOMG_k + yip * cosik * cosOMG_k;
    double zef = yip * sinik;
    sv.x[0] = xef;
    sv.x[1] = yef;
    sv.x[2] = zef;

    /// Compute velocity of rotation coordinates
    double dek, dlk, div, domk, duv, drv, dxp, dyp;
    dek = n * A / rk;

    //=====
    double dek2 = n/(1.0 - ecc * cosEk);

    //cout << "dek:" << dek << endl;
    //cout << "dek2:" << dek2 << endl;

    //======

    dlk = sqrt_A * q * std::sqrt(ell.gm()) / (rk * rk);

    //====
    double dlk2;
    dlk2 = q*dek2/(1.0 - ecc * cosEk);
    //cout << "dlk:" << dlk  << endl;
    //cout << "dlk2:" << dlk2  << endl;

    //=====

    div = IDOT - 2.0e0 * dlk * (Cic * sin2phi_k - Cis * cos2phi_k);
    domk = OMEGA_DOT - ell.angVelocity();
    duv = dlk * (1.e0 + 2.e0 * (Cus * cos2phi_k - Cuc * sin2phi_k));
    drv = A * ecc * dek * sinEk - 2.e0 * dlk * (Crc * sin2phi_k - Crs * cos2phi_k);
    dxp = drv * ::cos(uk) - rk * ::sin(uk) * duv;
    dyp = drv * ::sin(uk) + rk * ::cos(uk) * duv;

    /// Calculate velocities
    double vxef = dxp * cosOMG_k - xip * sinOMG_k * domk - dyp * cosik * sinOMG_k
                  + yip * (sinik * sinOMG_k * div - cosik * cosOMG_k * domk);
    double vyef = dxp * sinOMG_k + xip * cosOMG_k * domk + dyp * cosik * cosOMG_k
                  - yip * (sinik * cosOMG_k * div + cosik * sinOMG_k * domk);
    double vzef = dyp * sinik + yip * cosik * div;

    sv.v[0] = vxef;
    sv.v[1] = vyef;
    sv.v[2] = vzef;

    return sv;
}

Xvt NavEphBDS::svXvt(const CommonTime &t, const SatID& sat) const {
    // BDS GEO: PRN 1-5 (BDS-2) 及 59-62 (BDS-3)
    // 课本表 4-6：GEO 1-5, 59-62；MEO/IGSO 6-58
    bool isGEO = (sat.id >= 1 && sat.id <= 5) || (sat.id >= 59 && sat.id <= 62);
    if (!(sat.system == "C" && isGEO)) {
        // 非 GEO：直接返回标准结果
        return svXvt(t);
    }

    // === BDS-2 GEO 特殊处理 ===
    // 课本 4.2 节：GEO 卫星需要：
    // 1. Ωk = Ω0 + Ω_dot·tk - ωe·toe  (不同于 MEO 的 Ω_dot-ωe)
    // 2. [Xg,Yg,Zg] = RZ(ωe·tk) · RX(+5°) · [Xk,Yk,Zk]^T

    Xvt sv = svXvt(t);  // 先用标准 Keplerian 计算
    BDSEllipsoid ell;

    // 获取 tk
    double tk = t - ctToe;
    if (tk > 302400) tk -= 604800;
    if (tk < -302400) tk += 604800;

    double omega_e = ell.angVelocity();

    // 1. 用 GEO 公式重算 Ωk
    double OMEGA_k_GEO = OMEGA_0 + OMEGA_DOT * tk - omega_e * Toe;
    double sinOMG = sin(OMEGA_k_GEO);
    double cosOMG = cos(OMEGA_k_GEO);

    // 从 svXvt(t) 中提取轨道平面坐标 xip, yip, ik
    // 但实际上 svXvt(t) 已经做了 ECEF 变换，我们需要反推
    // 更可靠的方式：用 A, ecc, Ek, uk, ik 重算

    // 重新获取中间量（与 svXvt(t) 一致）
    double A = sqrt_A * sqrt_A;
    double n0 = sqrt(ell.gm() / (A * A * A));
    double n = n0 + Delta_n;
    double Mk = M0 + n * tk;
    double twoPI = 2.0 * PI;
    Mk = fmod(Mk, twoPI);
    double Ek = Mk + ecc * sin(Mk);
    for (int i = 0; i < 20; i++) {
        double F = Mk - (Ek - ecc * sin(Ek));
        double G = 1.0 - ecc * cos(Ek);
        double delea = F / G;
        Ek += delea;
        if (fabs(delea) < 1e-11) break;
    }

    double sinEk = sin(Ek), cosEk = cos(Ek);
    double q = sqrt(1.0 - ecc * ecc);
    double vk = atan2(q * sinEk, cosEk - ecc);

    double phi_k = vk + omega;
    double c2p = cos(2.0 * phi_k), s2p = sin(2.0 * phi_k);
    double duk = c2p * Cuc + s2p * Cus;
    double drk = c2p * Crc + s2p * Crs;
    double dik = c2p * Cic + s2p * Cis;

    double uk = phi_k + duk;
    double rk = A * (1.0 - ecc * cosEk) + drk;
    double ik = i0 + dik + IDOT * tk;

    // 轨道平面坐标（与 MEO 相同）
    double xip = rk * cos(uk);
    double yip = rk * sin(uk);

    // 用 GEO Ωk 计算 ECEF（中间坐标系）
    double ci = cos(ik), si = sin(ik);
    double Xk = xip * cosOMG - yip * ci * sinOMG;
    double Yk = xip * sinOMG + yip * ci * cosOMG;
    double Zk = yip * si;

    // 2. RX(+5°) 旋转
    double ang5 = 5.0 * DEG_TO_RAD;
    double c5 = cos(ang5), s5 = sin(ang5);
    double Y1 = Yk * c5 - Zk * s5;
    double Z1 = Yk * s5 + Zk * c5;

    // 3. RZ(ωe·tk) 地球自转改正
    double theta = omega_e * tk;
    double ct = cos(theta), st = sin(theta);
    sv.x[0] = Xk * ct + Y1 * st;
    sv.x[1] = -Xk * st + Y1 * ct;
    sv.x[2] = Z1;

    // === 速度处理（简版：RX + RZ 旋转已有速度）===
    // 对于 GEO，速度很小（相对地面静止），RX+RT 旋转已足够
    // 从 svXvt(t) 取原始速度进行旋转
    double vx0 = sv.v[0], vy0 = sv.v[1], vz0 = sv.v[2];

    // RX(+5°)
    double vy1 = vy0 * c5 - vz0 * s5;
    double vz1 = vy0 * s5 + vz0 * c5;

    // RZ(ωe·tk) + RZ_dot(ωe·tk) 项
    // 完整的 GEO 速度 = RZ·RX·V + RZ_dot·RX·X
    // 其中 RZ_dot = ωe * [-sin(θ), -cos(θ), 0; cos(θ), -sin(θ), 0; 0, 0, 0]
    sv.v[0] = vx0 * ct + vy1 * st + omega_e * (-Xk * st + Y1 * ct);
    sv.v[1] = -vx0 * st + vy1 * ct + omega_e * (-Xk * ct - Y1 * st);
    sv.v[2] = vz1;

    return sv;
}

bool NavEphBDS::isValid(const CommonTime &ct) const {
    if (ct.m_timeSystem != TimeSystem::BDT) return false;
    if (ct < beginValid || ct > endValid) return false;
    return true;
}