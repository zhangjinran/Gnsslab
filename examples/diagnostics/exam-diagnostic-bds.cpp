/**
 * Debug: BDS C01 GEO satellite position intermediate values
 *
 * Compares with textbook reference (表4-8, 4-9).
 * Target epoch: 2025-01-01 00:05:00 BDT
 * Reference epoch (Toe): 2025-01-01 00:00:00 BDT
 */
#include <iostream>
#include <iomanip>
#include <cmath>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/TimeConvert.h>
#include <gnsslab/RinexNavStore.hpp>

using namespace std;

int main() {
    cout << "=== BDS C01 GEO Debug ===" << endl;

    string dir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    string navFile = dir + "BRDC00IGS_R_20250010000_01D_MN.rnx";

    RinexNavStore nav;
    nav.loadFile(const_cast<string&>(navFile));

    BDSEllipsoid ell;
    SatID satC01("C01");

    // 课本用 GPST 00:05:00，BDT = GPST - 14s → tk = 286
    CivilTime civilTarget(2025, 1, 1, 0, 5, 0, TimeSystem::GPS);
    CommonTime epoch = convertTimeSystem(CivilTime2CommonTime(civilTarget), TimeSystem::BDT);

    CivilTime civilToe(2025, 1, 1, 0, 0, 0, TimeSystem::BDT);
    CommonTime ctToe = CivilTime2CommonTime(civilToe);

    try {
        NavEphBDS eph = nav.findBDSEph(satC01, epoch);

        // 打印星历参数 √
        cout << "\nsqrt_A=" << fixed << setprecision(12) << eph.sqrt_A
             << "  Delta_n=" << scientific << eph.Delta_n
             << "\nM0=" << fixed << eph.M0 << "  ecc=" << eph.ecc
             << "\nomega=" << eph.omega << "  OMEGA_0=" << eph.OMEGA_0
             << "\nOMEGA_DOT=" << scientific << eph.OMEGA_DOT
             << "\ni0=" << fixed << eph.i0 << "  IDOT=" << scientific << eph.IDOT << endl;

        double A = eph.sqrt_A * eph.sqrt_A;
        double n0 = sqrt(ell.gm() / (A * A * A));
        double tk = epoch - ctToe;

        cout << "\nA=" << fixed << setprecision(6) << A << " (ref: 42166175.565771)"
             << "\nn0=" << scientific << n0 << " (ref: 7.3e-5)"
             << "\ntk=" << fixed << setprecision(6) << tk << " (ref: 286.000000)" << endl;

        double n = n0 + eph.Delta_n;
        double twoPI = 2.0 * PI;
        double Mk = fmod(eph.M0 + n * tk, twoPI);
        cout << "Mk=" << fixed << Mk << " (ref: -0.682851)" << endl;

        // Ek iteration
        double Ek = Mk + eph.ecc * sin(Mk);
        for (int i = 0; i < 20; i++) {
            double F = Mk - (Ek - eph.ecc * sin(Ek));
            double G = 1.0 - eph.ecc * cos(Ek);
            double delea = F / G;
            Ek += delea;
            if (fabs(delea) < 1e-11) break;
        }
        cout << "Ek=" << fixed << Ek << " (ref: -0.683150)" << endl;

        double q = sqrt(1.0 - eph.ecc * eph.ecc);
        double vk = atan2(q * sin(Ek), cos(Ek) - eph.ecc);
        cout << "vk=" << fixed << vk << " (ref: -0.683449)" << endl;

        double phi_k = vk + eph.omega;
        double c2p = cos(2.0 * phi_k), s2p = sin(2.0 * phi_k);
        double duk = c2p * eph.Cuc + s2p * eph.Cus;
        double drk = c2p * eph.Crc + s2p * eph.Crs;
        double dik = c2p * eph.Cic + s2p * eph.Cis;
        double uk = phi_k + duk;
        double rk = A * (1.0 - eph.ecc * cos(Ek)) + drk;
        double ik = eph.i0 + dik + eph.IDOT * tk;

        cout << "Φk=" << fixed << phi_k << " (ref: -0.869285)"
             << "\nδik=" << scientific << dik << " (ref: -2.8e-5)"
             << "\nuk=" << fixed << uk << " (ref: -0.869314)"
             << "\nrk=" << fixed << rk << " (ref: 42150835.964427)"
             << "\nik=" << ik << " (ref: 0.058372)" << endl;

        double xip = rk * cos(uk), yip = rk * sin(uk);
        cout << "xip=" << fixed << xip << " (ref: 27202087.124328)"
             << "\nyip=" << yip << " (ref: 157.336631)" << endl;

        // Ωk: GEO vs MEO 公式对比
        double we = ell.angVelocity();
        double OMk_geo = eph.OMEGA_0 + eph.OMEGA_DOT * tk - we * eph.Toe;
        double OMk_meo = eph.OMEGA_0 + (eph.OMEGA_DOT - we) * tk - we * eph.Toe;
        cout << "\nΩk(GEO)=" << fixed << OMk_geo << " (ref: -21.721757)"
             << "\nΩk(MEO)=" << OMk_meo << endl;

        // 用 GEO 公式的 Ωk 算 ECEF（旋转前）
        double cOM = cos(OMk_geo), sOM = sin(OMk_geo);
        double ci = cos(ik), si = sin(ik);
        double Xk = xip * cOM - yip * ci * sOM;
        double Yk = xip * sOM + yip * ci * cOM;
        double Zk = yip * si;
        cout << "\nXk=" << fixed << Xk << " (ref: -34775856.583934)"
             << "\nYk=" << Yk << " (ref: 23744563.907316)"
             << "\nZk=" << Zk << " (ref: -1878418.538304)" << endl;

        // RX(-5°)
        double a5 = -5.0 * DEG_TO_RAD;
        double c5 = cos(a5), s5 = sin(a5);
        double Y5 = Yk * c5 - Zk * s5;
        double Z5 = Yk * s5 + Zk * c5;
        cout << "\nAfter RX(-5°):"
             << "\nX=" << fixed << Xk
             << "\nY=" << Y5
             << "\nZ=" << Z5 << endl;

        // RZ(ωe·tk)
        double th = we * tk;
        double ct = cos(th), st = sin(th);
        double Xg = Xk * ct + Y5 * st;
        double Yg = -Xk * st + Y5 * ct;
        double Zg = Z5;
        cout << "\nAfter RZ(ωe·tk) (final GEO):"
             << "\nXg=" << fixed << Xg << " (ref: -34271596.501952)"
             << "\nYg=" << Yg << " (ref: 24537957.549023)"
             << "\nZg=" << Zg << " (ref: 198204.514906)" << endl;

        // 当前代码结果对比
        Xvt sv_now = eph.svXvt(epoch, satC01);
        cout << "\n当前代码:"
             << "\nX=" << fixed << sv_now.x[0]
             << "\nY=" << sv_now.x[1]
             << "\nZ=" << sv_now.x[2] << endl;

    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}
