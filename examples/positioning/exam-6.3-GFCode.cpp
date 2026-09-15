/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *
 * Exam 6.3: 双频非组合单点定位 (GF Code)
 *
 * 模型：
 *   P1 = ρ + c·δt_r,IF + T + I + ε1
 *   P2 = ρ + c·δt_r,IF + T + γ·I + ε2
 *
 * 对 4 个卫星系统分别进行双频非组合定位。
 */

#include <string>
#include <fstream>
#include <iostream>
#include <set>
#include <map>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/TimeConvert.h>
#include <gnsslab/GnssFunc.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/RinexObsReader.h>
#include <gnsslab/SPPGFCode.h>
#include <gnsslab/CoordConvert.h>

#define debug 0

using namespace std;

map<string, string> sysNameMap = {
    {"GPS", "G"}, {"BDS", "C"}, {"Galileo", "E"}, {"GLONASS", "R"}
};

map<string, set<string>> getDualFreqTypes(const string& code) {
    map<string, set<string>> t;
    if (code == "G") { t["G"].insert("C1C"); t["G"].insert("C2W"); }
    else if (code == "C") { t["C"].insert("C2I"); t["C"].insert("C7I"); }
    else if (code == "E") { t["E"].insert("C1X"); t["E"].insert("C5X"); }
    else if (code == "R") { t["R"].insert("C1C"); t["R"].insert("C2C"); }
    return t;
}

void runSystem(const string& system, const string& roverFile,
               const string& outputPath, RinexNavStore* nav) {
    string code = sysNameMap[system];
    string solFile = outputPath + "sppUC_" + system + ".out";
    auto sysTypes = getDualFreqTypes(code);

    fstream obsStream(roverFile);
    if (!obsStream) { cerr << "Error opening " << roverFile << endl; return; }

    RinexObsReader reader;
    reader.setFileStream(&obsStream);
    reader.setSelectedTypes(sysTypes);

    // ENU 参考点（已知精确位置）
    Vector3d refXYZ(-2267750.275, 5009154.471, 3221294.345);
    unique_ptr<ReferenceFrame> frame = ReferenceFrameFactory::create("G");

    SPPGFCode spp;
    spp.setRinexNavStore(nav);
    spp.setSysTypes(sysTypes);

    fstream out(solFile, ios::out);
    out << "# YDSTime X Y Z E N U PDOP NSAT Sigma0 MeanResidual MaxResidual" << endl;

    int ep = 0, ok = 0;
    while (true) {
        ObsData d;
        try { d = reader.parseRinexObs(); }
        catch (EndOfFile&) { break; }
        reader.chooseObs(d);
        ep++;

        // 筛选有星历的卫星
        SatTypeValueMap keep;
        for (auto& s : d.satTypeValueData) {
            if (code == "G" && nav->gpsEphData.find(s.first)  != nav->gpsEphData.end())  keep.insert(s);
            if (code == "C" && nav->bdsEphData.find(s.first)  != nav->bdsEphData.end())  keep.insert(s);
            if (code == "E" && nav->galEphData.find(s.first)  != nav->galEphData.end())  keep.insert(s);
            if (code == "R" && nav->gloEphData.find(s.first)  != nav->gloEphData.end())  keep.insert(s);
        }
        d.satTypeValueData.swap(keep);

        try { spp.solve(d, true, true); }
        catch (...) { continue; }

        Vector3d xyz = spp.getXYZ();
        BLH blh = xyz2blh(xyz, *frame);
        XYZ enu = blh2ENU(blh, *frame, refXYZ);

        int nSat = d.satTypeValueData.size();

        out << CommonTime2YDSTime(d.epoch)
            << " " << fixed << setprecision(3) << xyz.transpose()
            << " " << fixed << setprecision(3) << enu.X() << " " << enu.Y() << " " << enu.Z()
            << " " << fixed << setprecision(2) << spp.getPDOP()
            << " " << nSat
            << " " << fixed << setprecision(3) << spp.getSigma0()
            << " " << fixed << setprecision(3) << spp.getMeanResidual()
            << " " << fixed << setprecision(3) << spp.getMaxResidual()
            << endl;
        ok++;
    }
    out.close();
    obsStream.close();
    // 记录总历元数用于跳过统计
    // 注意: const_cast 因为 gfEpochSkip 通过 getEpochSkipStats() 返回 const 引用
    // 但 totalEpochs 需要累加，这里直接通过 printEpochSkipStats 输出

    cout << system << " -> " << solFile << " (" << ok << "/" << ep << ")" << endl;
    spp.printEpochSkipStats(ep);
}

int main() {
    cout << "=== Exam 6.3: 双频非组合定位 (GF Code) ===" << endl;
    string dir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    string outDir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/spp_uc/";
    system(("mkdir -p " + outDir).c_str());

    RinexNavStore nav;
    string navFile = dir + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    nav.loadFile(const_cast<string&>(navFile));

    string obsFile = dir + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    vector<string> sysList = {"GPS", "BDS", "Galileo", "GLONASS"};
    for (const auto& s : sysList)
        runSystem(s, obsFile, outDir, &nav);

    cout << "\n完成！结果保存至: " << outDir << endl;
    return 0;
}
