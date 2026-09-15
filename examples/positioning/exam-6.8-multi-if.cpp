/**
 * Exam 6.8: 多系统 IF 组合定位（GPS + BDS + Galileo + GLONASS）
 */
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <set>
#include <gnsslab/SPPIFCode.h>
#include <gnsslab/RinexNavStore.hpp>

using namespace std;

void runMultiSystemIF(const string& label,
                      const map<string, pair<string, string>>& ifCodeTypes,
                      const map<string, set<string>>& selectedTypes,
                      const string& roverFile,
                      const string& navFile,
                      const string& outputDir) {
    cout << "\n--- " << label << " ---" << endl;

    RinexNavStore nav;
    nav.loadFile(const_cast<string&>(navFile));

    SPPIFCode sppif;
    sppif.setRinexNavStore(&nav);
    sppif.setIFCodeTypes(const_cast<map<string, pair<string, string>>&>(ifCodeTypes));
    sppif.setSelectedTypes(selectedTypes);

    string outPath = outputDir + label + ".out";
    vector<SPPIFResult> results = sppif.full_solve(&nav, const_cast<map<string, pair<string, string>>&>(ifCodeTypes), roverFile, true, true);

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
    string outDir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/if_multi/";
    system(("mkdir -p " + outDir).c_str());

    cout << "=== Exam 6.8: 多系统 IF 组合定位 ===" << endl;

    // ===== IF 组合类型（转换后） =====
    map<string, pair<string, string>> gpsIF, bdsIF, galIF, gloIF;
    gpsIF["G"] = {"C1", "C2"};          // L1 + L2
    bdsIF["C"] = {"C2", "C7"};          // B1I + B2I
    galIF["E"] = {"C1", "C5"};          // E1 + E5a
    gloIF["R"] = {"C1", "C2"};          // L1 + L2（FDMA，频率按 freqNum 算）

    // ===== 原始观测类型 =====
    map<string, set<string>> gpsT, bdsT, galT, gloT;
    gpsT["G"] = {"C1W", "C2W"};
    bdsT["C"] = {"C2I", "C7I"};
    galT["E"] = {"C1X", "C5X"};
    gloT["R"] = {"C1C", "C2C"};

    // ---- 双系统 ----
    { auto ift = gpsIF; auto st = gpsT; st["C"] = bdsT["C"]; ift["C"] = bdsIF["C"];
      runMultiSystemIF("if_GPS_BDS", ift, st, obsFile, navFile, outDir); }

    { auto ift = gpsIF; auto st = gpsT; st["E"] = galT["E"]; ift["E"] = galIF["E"];
      runMultiSystemIF("if_GPS_Galileo", ift, st, obsFile, navFile, outDir); }

    { auto ift = gpsIF; auto st = gpsT; st["R"] = gloT["R"]; ift["R"] = gloIF["R"];
      runMultiSystemIF("if_GPS_GLONASS", ift, st, obsFile, navFile, outDir); }

    // ---- 四系统 ----
    { auto ift = gpsIF; auto st = gpsT;
      st["C"] = bdsT["C"]; ift["C"] = bdsIF["C"];
      st["E"] = galT["E"]; ift["E"] = galIF["E"];
      st["R"] = gloT["R"]; ift["R"] = gloIF["R"];
      runMultiSystemIF("if_GPS_BDS_Galileo_GLONASS", ift, st, obsFile, navFile, outDir); }

    return 0;
}
