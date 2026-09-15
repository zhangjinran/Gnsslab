/**
 * Exam 6.7: 多系统 SPP 单点定位
 *
 * 支持任意系统组合，以函数式封装复用
 * SPP 每系统只需一个观测码
 */
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <set>
#include <gnsslab/SPPCode.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/TimeConvert.h>

using namespace std;

void runMultiSystemSPP(const string& label,
                       const map<string, set<string>>& sysTypes,
                       const string& roverFile,
                       const string& navFile,
                       const string& outputDir) {
    cout << "\n--- " << label << " ---" << endl;

    RinexNavStore nav;
    nav.loadFile(const_cast<string&>(navFile));

    SPPCode spp;
    spp.setRinexNavStore(&nav);
    spp.setSysTypes(sysTypes);

    string outPath = outputDir + label + ".out";
    vector<SPPResult> results = spp.full_solve(&nav, roverFile, sysTypes, true, true, false);

    fstream out(outPath, ios::out);
    out << "# YDSTime X Y Z E N U PDOP NSAT Sigma0 MeanResidual RMSResidual MaxResidual" << endl;
    for (auto& r : results) {
        out << r.ydsTime
            << " " << fixed << setprecision(3) << r.xyz.transpose()
            << " " << fixed << setprecision(3) << r.enu.transpose()
            << " " << fixed << setprecision(2) << r.pdop
            << " " << r.nSat
            << " " << fixed << setprecision(3) << r.sigma0
            << " " << fixed << setprecision(3) << r.meanResidual
            << " " << fixed << setprecision(3) << r.rmsResidual
            << " " << fixed << setprecision(3) << r.maxResidual
            << endl;
    }
    out.close();
    cout << "  输出: " << outPath << " (" << results.size() << " 历元)" << endl;
}

int main() {
    string dir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    string navFile = dir + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    string obsFile = dir + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    string outDir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/spp_multi/";
    system(("mkdir -p " + outDir).c_str());

    cout << "=== Exam 6.7: 多系统 SPP 定位 ===" << endl;

    map<string, set<string>> gps, bds, gal, glo, qzs, irn;
    gps["G"].insert("C1W");
    bds["C"].insert("C2I");
    gal["E"].insert("C1X");
    glo["R"].insert("C1C");
    qzs["J"].insert("C1C");
    irn["I"].insert("C5A");

    // 双系统
    { auto t = gps; t["C"] = bds["C"]; runMultiSystemSPP("spp_GPS_BDS", t, obsFile, navFile, outDir); }
    { auto t = gps; t["E"] = gal["E"]; runMultiSystemSPP("spp_GPS_Galileo", t, obsFile, navFile, outDir); }
    { auto t = bds; t["E"] = gal["E"]; runMultiSystemSPP("spp_BDS_Galileo", t, obsFile, navFile, outDir); }

    // 三系统
    { auto t = gps; t["C"] = bds["C"]; t["E"] = gal["E"]; runMultiSystemSPP("spp_GPS_BDS_Galileo", t, obsFile, navFile, outDir); }

    // 四系统
    { auto t = gps; t["C"] = bds["C"]; t["E"] = gal["E"]; t["R"] = glo["R"]; runMultiSystemSPP("spp_GPS_BDS_Galileo_GLONASS", t, obsFile, navFile, outDir); }

    // 六系统
    { auto t = gps; t["C"] = bds["C"]; t["E"] = gal["E"]; t["R"] = glo["R"]; t["J"] = qzs["J"]; t["I"] = irn["I"];
      runMultiSystemSPP("spp_6systems", t, obsFile, navFile, outDir); }

    return 0;
}
