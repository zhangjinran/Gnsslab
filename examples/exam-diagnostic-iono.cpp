/**
 * Debug: Klobuchar 电离层模型中间量验证
 *
 * 对照课本例 5-2：
 *   历元: 2025-01-01 00:00:00 GPST
 *   测站: WUH2 (-2267749, 5009154, 3221290)
 *   卫星: G10 (方位角 265.659363°, 高度角 83.007251°)
 */
#include <iostream>
#include <iomanip>
#include <cmath>
#include "GnssStruct.h"
#include "TimeConvert.h"
#include "GnssFunc.h"
#include "CoordConvert.h"
#include "RinexNavStore.hpp"

using namespace std;

int main() {
    cout << "=== Klobuchar 电离层模型 Debug ===" << endl;

    string dir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    string navFile = dir + "BRDC00IGS_R_20250010000_01D_MN.rnx";

    RinexNavStore nav;
    nav.loadFile(const_cast<string&>(navFile));

    // WUH2 近似坐标
    Vector3d xyz(-2267749.0, 5009154.0, 3221290.0);

    // G10 卫星
    SatID satG10("G10");
    double elev = 83.007251;   // 度
    double azim = 265.659363;  // 度

    // 历元: 2025-01-01 00:00:00 GPST
    CivilTime civil(2025, 1, 1, 0, 0, 0, TimeSystem::GPS);
    CommonTime epoch = CivilTime2CommonTime(civil);

    // 转换到 GPS 周秒
    GPSWeekSecond ws;
    CommonTime2WeekSecond(epoch, ws);
    double tow = ws.getSOW();

    cout << "GPS 周秒: " << fixed << setprecision(6) << tow << endl;

    // 接收机 BLH
    auto frame = ReferenceFrameFactory::create("G");
    BLH blh = xyz2blh(xyz, *frame);
    double latUser = blh(0);  // rad
    double lonUser = blh(1);  // rad
    cout << "测站纬度: " << latUser*180/PI << "°  经度: " << lonUser*180/PI << "°" << endl;

    // === Klobuchar 逐步骤 ===
    double RE = frame->getA();
    double h_ion = 350000.0;  // GPS 电离层高度

    double E = elev * PI / 180.0;
    double A = azim * PI / 180.0;

    // 1. 地心角
    double psi = PI/2.0 - E - asin((RE * cos(E)) / (RE + h_ion));
    cout << "\n1. 地心角 ψ = " << psi*180/PI << "° (ref: 0.357594°)" << endl;

    // 2. IPP 纬度
    double latIPP = asin(sin(latUser)*cos(psi) + cos(latUser)*sin(psi)*cos(A));
    cout << "2. IPP 纬度 = " << latIPP*180/PI << "° (ref: 30.504584°)" << endl;

    // 3. IPP 经度
    double lonIPP = lonUser + (sin(psi)*sin(A))/cos(latIPP);
    cout << "3. IPP 经度 = " << lonIPP*180/PI << "° (ref: 113.943409°)" << endl;

    // 4. 地磁纬度 — 课本值
    const double phiP_book = 78.3 * PI/180.0;
    const double lamP_book = 291.0 * PI/180.0;
    double latMag_book = asin(sin(latIPP)*sin(phiP_book) + cos(latIPP)*cos(phiP_book)*cos(lonIPP - lamP_book));
    cout << "4a.地磁纬度(课本参数) = " << latMag_book*180/PI << "° (ref: 18.999169°)" << endl;

    // 地磁纬度 — 代码值（当前代码用的 79.5°, 288.0°）
    const double phiP_code = 79.5 * PI/180.0;
    const double lamP_code = 288.0 * PI/180.0;
    double latMag_code = asin(sin(latIPP)*sin(phiP_code) + cos(latIPP)*cos(phiP_code)*cos(lonIPP - lamP_code));
    cout << "4b.地磁纬度(代码参数) = " << latMag_code*180/PI << "°" << endl;

    // 5. 当地时间
    double t = 43200.0 * lonIPP / PI + tow;
    t = fmod(t, 86400.0);
    if (t < 0) t += 86400.0;
    cout << "5. 当地时间 t = " << fixed << setprecision(8) << t << " s (ref: 27346.41818396)" << endl;

    // === 用课本参数重算地磁纬度 ===
    double latMag = latMag_book;

    cout << "5a.使用课本地磁纬度: " << latMag*180/PI << "°" << endl;

    // 从导航文件读取 GPSA/GPSB 参数（按系统名查找，不用 begin()）
    double alpha[4] = {0}, beta[4] = {0};
    auto itA = nav.ionoCorrData.find("GPSA");
    if (itA != nav.ionoCorrData.end())
        for (int i = 0; i < 4 && i < (int)itA->second.size(); i++) alpha[i] = itA->second[i];
    auto itB = nav.ionoCorrData.find("GPSB");
    if (itB != nav.ionoCorrData.end())
        for (int i = 0; i < 4 && i < (int)itB->second.size(); i++) beta[i] = itB->second[i];

    cout << "\nalpha: " << scientific << alpha[0] << " " << alpha[1] << " " << alpha[2] << " " << alpha[3] << endl;
    cout << "beta:  " << beta[0] << " " << beta[1] << " " << beta[2] << " " << beta[3] << endl;

    double x = latMag_book / PI;
    // 6. 幅度
    double AI = alpha[0] + alpha[1]*x + alpha[2]*x*x + alpha[3]*x*x*x;
    if (AI < 0) AI = 0;
    cout << "\n6. 幅度 AI = " << scientific << setprecision(10) << AI << " s (ref: 3.07e-8)" << endl;

    // 7. 周期
    double Pi = beta[0] + beta[1]*x + beta[2]*x*x + beta[3]*x*x*x;
    if (Pi < 72000) Pi = 72000;
    cout << "7. 周期 PI = " << fixed << setprecision(6) << Pi << " s (ref: 133055.103903)" << endl;

    // 8. 相位
    double XI = 2*PI*(t - 50400)/Pi;
    cout << "8. 相位 XI = " << fixed << setprecision(8) << XI << " rad (ref: -1.0886461495)" << endl;

    // 9. 倾斜因子
    double F = 1.0 / sqrt(1.0 - pow((RE*cos(E))/(RE+h_ion), 2));
    cout << "9. 倾斜因子 F = " << fixed << setprecision(8) << F << " (ref: 1.0052216221)" << endl;

    // 10. L1延迟
    double I1;
    if (fabs(XI) < PI/2.0)
        I1 = (5e-9 + AI*cos(XI)) * F;
    else
        I1 = 5e-9 * F;
    cout << "10.L1延迟 I1 = " << scientific << setprecision(10) << I1 << " s (ref: 1.94e-8 ≈ 5.816m)" << endl;
    cout << "   I1 (米)  = " << fixed << setprecision(3) << I1 * C_MPS << " m" << endl;

    // 调用现有函数对比
    double result = klobucharIonosphericCorrection(xyz, elev, azim, alpha, beta, tow, satG10, L1_FREQ_GPS);
    cout << "\n函数返回值: " << scientific << setprecision(10) << result << " s"
         << " = " << fixed << setprecision(3) << result * C_MPS << " m" << endl;

    // === BDS 测试（C06 卫星）===
    cout << "\n=== BDS C06 Klobuchar 测试 ===" << endl;
    SatID satC06("C06");
    double elevC06 = 60.0, azimC06 = 180.0;  // 假设值

    // BDS 用 BDSA/BDSB 参数（按卫星查找）
    double alphaB[4] = {0}, betaB[4] = {0};
    auto itBDS_A = nav.ionoCorrDataBDS.find(satC06);
    if (itBDS_A != nav.ionoCorrDataBDS.end() && itBDS_A->second.hasAlpha)
        for (int i = 0; i < 4; i++) alphaB[i] = itBDS_A->second.alpha[i];
    auto itBDS_B = nav.ionoCorrDataBDS.find(satC06);
    if (itBDS_B != nav.ionoCorrDataBDS.end() && itBDS_B->second.hasBeta)
        for (int i = 0; i < 4; i++) betaB[i] = itBDS_B->second.beta[i];

    cout << "C06 alpha: " << scientific << alphaB[0] << " " << alphaB[1] << " " << alphaB[2] << " " << alphaB[3] << endl;
    cout << "C06 beta:  " << betaB[0] << " " << betaB[1] << " " << betaB[2] << " " << betaB[3] << endl;

    // BDS 参考频率 B1I = L2_FREQ_BDS = 1561.098 MHz
    double resultB = klobucharIonosphericCorrection(xyz, elevC06, azimC06, alphaB, betaB, tow, satC06, L2_FREQ_BDS);
    cout << "C06 电离层延迟: " << scientific << resultB << " s = " << fixed << setprecision(3) << resultB * C_MPS << " m" << endl;

    return 0;
}
