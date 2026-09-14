/**
 * Debug: Saastamoinen 对流层模型
 *
 * 对照课本例 5-3：
 *   历元: 2025-01-01 00:00:00 GPST
 *   测站: WUH2 (-2267749, 5009154, 3221290)
 *   卫星: G10 (仰角 83.007251°)
 */
#include <iostream>
#include <iomanip>
#include <cmath>
#include "GnssStruct.h"
#include "TimeConvert.h"
#include "CoordConvert.h"
#include "GnssFunc.h"

using namespace std;

int main() {
    cout << "=== Saastamoinen 对流层模型 Debug ===" << endl;

    // WUH2 近似坐标
    Vector3d xyz(-2267749.0, 5009154.0, 3221290.0);

    // G10 卫星
    SatID satG10("G10");
    double elev = 83.007251;   // 度

    // BLH
    GPSEllipsoid ell;
    BLH blh = xyz2blh(xyz, ell);
    double B = blh(0);   // rad
    double H_m = blh(2); // m
    double H_km = H_m / 1000.0;

    cout << fixed << setprecision(6);
    cout << "纬度: " << B << " rad (ref: 0.532878)" << endl;
    cout << "大地高: " << H_m << " m (ref: 25.126182)" << endl;

    // === 标准大气模型 ===
    double P = 1013.25 * pow(1.0 - 0.0000226 * H_m, 5.225);
    double T_c = 15.0 - 0.0065 * H_m;
    double T_k = T_c + 273.15;
    double RH = 0.5;

    cout << "\n大气压 P = " << P << " hPa (ref: 1010.247266)" << endl;
    cout << "温度 T = " << T_c << " C / " << T_k << " K (ref: 287.986680 K)" << endl;

    // 饱和水汽压: 代码公式 vs 课本公式
    double es_code = 6.1078 * exp(17.27 * T_c / (T_c + 237.3));
    double es_book = 6.108 * exp((17.15 * T_k - 4684.0) / (T_k - 38.45));
    double e_code = RH * es_code;
    double e_book = RH * es_book;

    cout << "\nes (代码 Magnus) = " << es_code << " hPa" << endl;
    cout << "es (课本 Tetens) = " << es_book << " hPa (→ e=" << e_book << ", ref: 8.484425)" << endl;

    // 用课本公式的 e
    double e = e_book;

    // Saastamoinen ZHD / ZWD
    double fBH = 1.0 - 0.00266 * cos(2.0 * B) - 0.00028 * H_km;
    double ZHD = 2.277e-3 * P / fBH;
    double ZWD = 0.002277 * (1255.0 / T_k + 0.05) * e;

    cout << "\nf(B,H) = " << fBH << endl;
    cout << "ZHD = " << ZHD << " m (ref: 2.303314)" << endl;
    cout << "ZWD = " << ZWD << " m (ref: 0.085155)" << endl;

    // 映射函数: 课本简单 1/sin(E)
    double E_rad = elev * PI / 180.0;
    double MF_simple = 1.0 / sin(E_rad);
    double tropo_simple = (ZHD + ZWD) * MF_simple;
    cout << "\nMF (1/sin) = " << MF_simple << " (ref: ~1.0073)" << endl;
    cout << "Tropo (简单映射) = " << tropo_simple << " m (ref: 2.406368)" << endl;

    // 调用现有函数对比
    double result = saastamoinenTroposphericCorrection(blh, elev, satG10, 0.5);
    cout << "\n函数返回值 = " << result << " m" << endl;

    return 0;
}
