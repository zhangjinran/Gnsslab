#include <string>
#include <iomanip>
#include <gnsslab/NavEphQZSS.hpp>

using namespace std;

void NavEphQZSS::printData() const {
    cout << "****************************************************************"
         << "************" << endl
         << "QZSS Broadcast Ephemeris Data: " << endl;
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
    cout << "Codes_On_L2_Channel: " << setw(16) << L2Codes << endl;
    cout << "QZSSWeek: " << setw(16) << QZSSWeek << endl;
    cout << "L2P_data_flag: " << setw(16) << L2Pflag << endl;

    cout << "URA: " << setw(16) << URA << endl;
    cout << "SV_health: " << setw(16) << SV_health << endl;
    cout << "TGD: " << setw(16) << TGD << endl;
    cout << "IODC: " << setw(16) << IODC << endl;

    cout << "HOWtime: " << setw(16) << HOWtime << endl;
    cout << "fitInterval: " << setw(16) << fitInterval << endl;

    cout << "ctToc: " << ctToc.toString() << endl;
    cout << "ctToe: " << ctToe.toString() << endl;
}

double NavEphQZSS::svClockBias(const CommonTime &t) const {
    double dtc, elaptc;
    elaptc = t - ctToc;
    dtc = af0 + elaptc * (af1 + elaptc * af2);
    return dtc;
}

double NavEphQZSS::svClockDrift(const CommonTime &t) const {
    double drift, elaptc;
    elaptc = t - ctToc;
    drift = af1 + elaptc * af2;
    return drift;
}

long double NavEphQZSS::svRelativity(const CommonTime &t) const {
    GPSEllipsoid ell;
    double A = sqrt_A * sqrt_A;
    double n0 = std::sqrt(ell.gm() / (A * A * A));
    double tk = t - ctToe;
    if (tk > 302400) tk = tk - 604800;
    if (tk < -302400) tk = tk + 604800;
    double n = n0 + Delta_n;
    double Mk = M0 + n * tk;
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
    return (REL_CONST * ecc * std::sqrt(A) * ::sin(Ek));
}

double NavEphQZSS::svURA(const CommonTime &t) const {
    double ephURA = URA;
    return ephURA;
}

Xvt NavEphQZSS::svXvt(const CommonTime &t) const {
    Xvt sv;
    GPSEllipsoid ell;

    double A = sqrt_A * sqrt_A;
    double n0 = std::sqrt(ell.gm() / (A * A * A));
    double tk = t - ctToe;
    if (tk > 302400) tk = tk - 604800;
    if (tk < -302400) tk = tk + 604800;
    double n = n0 + Delta_n;
    double Mk = M0 + n * tk;

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

    sv.relcorr = svRelativity(t);
    sv.clkbias = svClockBias(t);
    sv.clkdrift = svClockDrift(t);

    double q = std::sqrt(1.0 - ecc * ecc);
    double sinEk = ::sin(Ek);
    double cosEk = ::cos(Ek);
    double GSTA = q * sinEk;
    double GCTA = cosEk - ecc;
    double vk = atan2(GSTA, GCTA);

    double phi_k = vk + omega;
    double cos2phi_k = ::cos(2.0 * phi_k);
    double sin2phi_k = ::sin(2.0 * phi_k);

    double duk = cos2phi_k * Cuc + sin2phi_k * Cus;
    double drk = cos2phi_k * Crc + sin2phi_k * Crs;
    double dik = cos2phi_k * Cic + sin2phi_k * Cis;

    double uk = phi_k + duk;
    double rk = A * (1.0 - ecc * cosEk) + drk;
    double ik = i0 + dik + IDOT * tk;

    double xip = rk * ::cos(uk);
    double yip = rk * ::sin(uk);

    double OMEGA_k = OMEGA_0 + (OMEGA_DOT - ell.angVelocity()) * tk
                     - ell.angVelocity() * Toe;

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

    double dek, dlk, div, domk, duv, drv, dxp, dyp;
    dek = n * A / rk;
    dlk = sqrt_A * q * std::sqrt(ell.gm()) / (rk * rk);
    div = IDOT - 2.0e0 * dlk * (Cic * sin2phi_k - Cis * cos2phi_k);
    domk = OMEGA_DOT - ell.angVelocity();
    duv = dlk * (1.e0 + 2.e0 * (Cus * cos2phi_k - Cuc * sin2phi_k));
    drv = A * ecc * dek * sinEk - 2.e0 * dlk * (Crc * sin2phi_k - Crs * cos2phi_k);
    dxp = drv * ::cos(uk) - rk * ::sin(uk) * duv;
    dyp = drv * ::sin(uk) + rk * ::cos(uk) * duv;

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

bool NavEphQZSS::isValid(const CommonTime &ct) const {
    if (ct.m_timeSystem != TimeSystem::QZS) return false;
    if (ct < beginValid || ct > endValid) return false;
    return true;
}