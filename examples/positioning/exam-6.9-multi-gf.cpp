/**
 * Exam 6.9: 多系统 GF 非组合定位（GPS + BDS + Galileo + GLONASS）
 *
 * 使用 SPPGFCode，sysCode 留空时遍历 sysTypes 中所有系统
 */
#include <string>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <gnsslab/SPPGFCode.h>
#include <gnsslab/RinexNavStore.hpp>

using namespace std;

void runMultiSystemGF(const string& label,
                      const map<string, set<string>>& sysTypes,
                      const string& roverFile,
                      const string& navFile,
                      const string& outputDir) {
    cout << "\n--- " << label << " ---" << endl;

    RinexNavStore nav;
    nav.loadFile(const_cast<string&>(navFile));

    SPPGFCode sppgf;
    sppgf.setRinexNavStore(&nav);
    sppgf.setSysTypes(sysTypes);

    string outPath = outputDir + label + ".out";
    // sysCode 留空 → full_solve 遍历 sysTypes 中所有系统
    sppgf.full_solve(&nav, outPath, roverFile, true, true);

    int n = 0; string line;
    ifstream in(outPath);
    while (getline(in, line)) if (line[0] != '#') n++;
    cout << "  输出: " << outPath << " (" << n << " 历元)" << endl;
}

int main() {
    string dir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    string navFile = dir + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    string obsFile = dir + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    string outDir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/gf_multi/";
    system(("mkdir -p " + outDir).c_str());

    cout << "=== Exam 6.9: 多系统 GF 非组合定位 ===" << endl;

    // 各系统双频观测类型
    map<string, set<string>> gpsT, bdsT, galT, gloT;
    gpsT["G"] = {"C1W", "C2W"};
    bdsT["C"] = {"C2I", "C7I"};
    galT["E"] = {"C1X", "C5X"};
    gloT["R"] = {"C1C", "C2C"};

    // 双系统
    { auto st = gpsT; st["C"] = bdsT["C"]; runMultiSystemGF("gf_GPS_BDS", st, obsFile, navFile, outDir); }
    { auto st = gpsT; st["E"] = galT["E"]; runMultiSystemGF("gf_GPS_Galileo", st, obsFile, navFile, outDir); }
    { auto st = gpsT; st["R"] = gloT["R"]; runMultiSystemGF("gf_GPS_GLONASS", st, obsFile, navFile, outDir); }

    // 四系统
    { auto st = gpsT; st["C"] = bdsT["C"]; st["E"] = galT["E"]; st["R"] = gloT["R"];
      runMultiSystemGF("gf_GPS_BDS_Galileo_GLONASS", st, obsFile, navFile, outDir); }

    return 0;
}
