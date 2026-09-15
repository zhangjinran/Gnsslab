/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *
 * Exam 6.6: 单点测速 (SPP Velocity)
 *
 * 基于多普勒观测值，利用最小二乘估计接收机速度。
 * 对 4 个卫星系统（GPS/BDS/Galileo/GLONASS）分别进行单点测速。
 * 公式参考教材第 6.3 节：
 *   l_rs = -λ·D - e·Ẋ^s + c·δṫ_s  =  -e·Ẋ_r + c·δṫ_r
 */

#include <string>
#include <fstream>
#include <iostream>
#include <set>
#include <map>
#include <vector>
#include <iomanip>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/TimeConvert.h>
#include <gnsslab/GnssFunc.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/RinexObsReader.h>
#include <gnsslab/SPPCode.h>
#include <gnsslab/CoordConvert.h>

#define debug 0

using namespace std;

map<string, string> sysNameMap = {
    {"GPS", "G"}, {"BDS", "C"}, {"Galileo", "E"}, {"GLONASS", "R"}
};

map<string, set<string>> getDopplerTypes(const string& code) {
    map<string, set<string>> t;
    if (code == "G") { t["G"].insert("C1C"); t["G"].insert("D1C"); }
    else if (code == "C") { t["C"].insert("C2I"); t["C"].insert("D2I"); }
    else if (code == "E") { t["E"].insert("C1X"); t["E"].insert("D1X"); }
    else if (code == "R") { t["R"].insert("C1C"); t["R"].insert("D1C"); }
    return t;
}

void runSystem(const string& system, const string& roverFile,
               const string& outputPath, RinexNavStore* nav) {
    string code = sysNameMap[system];
    string solFile = outputPath + "vel_" + system + ".out";
    auto sysTypes = getDopplerTypes(code);

    fstream obsStream(roverFile);
    if (!obsStream) { cerr << "Error opening " << roverFile << endl; return; }

    RinexObsReader reader;
    reader.setFileStream(&obsStream);
    reader.setSelectedTypes(sysTypes);

    // ENU 参考点（已知精确位置）
    Vector3d refXYZ(-2267750.275, 5009154.471, 3221294.345);
    unique_ptr<ReferenceFrame> frame = ReferenceFrameFactory::create("G");

    SPPCode spp;
    spp.setRinexNavStore(nav);
    spp.setSystemCode(code);
    spp.setSysTypes(sysTypes);

    fstream out(solFile, ios::out);
    out << "# YDSTime X Y Z E N U Vx Vy Vz cdt_dot VDOP NSAT" << endl;

    int ep = 0, ok = 0;
    while (true) {
        ObsData d;
        try { d = reader.parseRinexObs(); }
        catch (EndOfFile&) { break; }
        reader.chooseObs(d);
        ep++;

        SatTypeValueMap keep;
        for (auto& s : d.satTypeValueData) {
            if (code == "G" && nav->gpsEphData.find(s.first)  != nav->gpsEphData.end())  keep.insert(s);
            if (code == "C" && nav->bdsEphData.find(s.first)  != nav->bdsEphData.end())  keep.insert(s);
            if (code == "E" && nav->galEphData.find(s.first)  != nav->galEphData.end())  keep.insert(s);
            if (code == "R" && nav->gloEphData.find(s.first)  != nav->gloEphData.end())  keep.insert(s);
        }
        d.satTypeValueData.swap(keep);

        try { spp.solve(d, true, true, true); }
        catch (...) { continue; }

        auto vel = spp.solveVelocity(d);
        if (vel.nSat < 4) continue;

        Vector3d xyz = spp.getXYZ();
        BLH blh = xyz2blh(xyz, *frame);
        XYZ enu = blh2ENU(blh, *frame, refXYZ);

        out << CommonTime2YDSTime(d.epoch)
            << " " << fixed << setprecision(3) << xyz.transpose()
            << " " << fixed << setprecision(3) << enu.X() << " " << enu.Y() << " " << enu.Z()
            << " " << fixed << setprecision(3) << vel.vel.transpose()
            << " " << fixed << setprecision(3) << vel.cdt_dot
            << " " << fixed << setprecision(2) << vel.vdop
            << " " << vel.nSat << endl;
        ok++;
    }
    out.close();
    obsStream.close();
    cout << system << " -> " << solFile << " (" << ok << "/" << ep << ")" << endl;
}

int main() {
    cout << "=== Exam 6.6: 单点测速 ===" << endl;
    string dir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    string outDir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/spp_velocity/";
    system(("mkdir -p " + outDir).c_str());

    RinexNavStore nav;
    string navFile = dir + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    nav.loadFile(navFile);

    vector<string> systems = {"GPS", "BDS", "Galileo", "GLONASS"};
    string obsFile = dir + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    for (const auto& s : systems)
        runSystem(s, obsFile, outDir, &nav);

    cout << "\n完成！结果保存至: " << outDir << endl;
    return 0;
}
