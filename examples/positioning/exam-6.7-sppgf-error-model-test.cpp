/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 */

#include <string>
#include <fstream>
#include <iostream>
#include <set>
#include <map>
#include <vector>
#include <iomanip>
#include <memory>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/TimeConvert.h>
#include <gnsslab/GnssFunc.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/RinexObsReader.h>
#include <gnsslab/SPPGFCode.h>
#include <gnsslab/CoordConvert.h>
#include <gnsslab/CoordStruct.h>

#define debug 0

using namespace std;

void runSPPGFWithErrorModel(const string& system,
                            const string& roverFile,
                            const string& navFile,
                            const string& outputPath,
                            const string& modelFlag,
                            bool relativityEnable,
                            bool earthRotationEnable,
                            bool TGD_bool,
                            bool Trop_bool,
                            const std::map<string, std::set<string>>& sysTypes) {
    
    std::cout << "  " << system << " - " << modelFlag << std::endl;
    
    std::string solFile = outputPath + "sppgf_" + system + modelFlag + ".out";
    
    // 加载导航文件（静态变量只加载一次）
    static std::map<std::string, RinexNavStore> navStoreMap;
    RinexNavStore* pNavStore;
    
    auto navIt = navStoreMap.find(navFile);
    if (navIt == navStoreMap.end()) {
        RinexNavStore& newStore = navStoreMap[navFile];
        if (!newStore.loadFile(const_cast<string&>(navFile))) {
            std::cerr << "Error loading nav file for " << system << std::endl;
            return;
        }
        pNavStore = &newStore;
    } else {
        pNavStore = &(navIt->second);
    }
    
    // 手动控制流程（类似 exam-6.3）
    fstream obsStream(const_cast<string&>(roverFile));
    if (!obsStream) { cerr << "  Error opening " << roverFile << endl; return; }
    
    RinexObsReader reader;
    reader.setFileStream(&obsStream);
    reader.setSelectedTypes(const_cast<std::map<string, std::set<string>>&>(sysTypes));
    
    Vector3d refXYZ(-2267750.275, 5009154.471, 3221294.345);
    std::map<std::string, std::string> sysCodeMap2 = {
        {"GPS", "G"}, {"BDS", "C"}, {"Galileo", "E"}, {"GLONASS", "R"}
    };
    unique_ptr<ReferenceFrame> frame = ReferenceFrameFactory::create(sysCodeMap2[system]);
    
    SPPGFCode sppgf;
    sppgf.setRinexNavStore(pNavStore);
    sppgf.setSysTypes(sysTypes);
    sppgf.setRelativityEnable(relativityEnable);
    sppgf.setEarthRotationEnable(earthRotationEnable);
    
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
        string code = sysCodeMap2[system];
        SatTypeValueMap keep;
        for (auto& s : d.satTypeValueData) {
            bool has = false;
            if (code == "G" && pNavStore->gpsEphData.find(s.first) != pNavStore->gpsEphData.end()) has = true;
            if (code == "C" && pNavStore->bdsEphData.find(s.first) != pNavStore->bdsEphData.end()) has = true;
            if (code == "E" && pNavStore->galEphData.find(s.first) != pNavStore->galEphData.end()) has = true;
            if (code == "R" && pNavStore->gloEphData.find(s.first) != pNavStore->gloEphData.end()) has = true;
            if (has) keep.insert(s);
        }
        d.satTypeValueData.swap(keep);
        
        try { sppgf.solve(d, TGD_bool, Trop_bool); }
        catch (...) { continue; }
        
        double pdop = sppgf.getPDOP();
       
        
        Vector3d xyz = sppgf.getXYZ();
        BLH blh = xyz2blh(xyz, *frame);
        XYZ enu = blh2ENU(blh, *frame, refXYZ);
        int nSat = d.satTypeValueData.size();
        
        out << CommonTime2YDSTime(d.epoch)
            << " " << fixed << setprecision(3) << xyz.transpose()
            << " " << fixed << setprecision(3) << enu.X() << " " << enu.Y() << " " << enu.Z()
            << " " << fixed << setprecision(2) << sppgf.getPDOP()
            << " " << nSat
            << " " << fixed << setprecision(3) << sppgf.getSigma0()
            << " " << fixed << setprecision(3) << sppgf.getMeanResidual()
            << " " << fixed << setprecision(3) << sppgf.getMaxResidual()
            << endl;
        ok++;
    }
    out.close(); obsStream.close();
    std::cout << "  Output -> " << solFile << " (" << ok << "/" << ep << " epochs)" << std::endl;
}

int main() {
    std::cout << "=== SPPGF Error Model Test ===" << std::endl;
    
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    
    std::string outputPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/sppgf_error_model/";
    string cmd = "mkdir -p " + outputPath;
    system(cmd.c_str());
    
    struct SystemConfig {
        string name;
        std::map<string, std::set<string>> types;
    };
    
    vector<SystemConfig> systems = {
        {"GPS", {{"G", {"C1C", "C2W"}}}},
        {"BDS", {{"C", {"C2I", "C7I"}}}},
        {"Galileo", {{"E", {"C1X", "C5X"}}}},
        {"GLONASS", {{"R", {"C1C", "C2C"}}}}
    };
    
    for (const auto& sys : systems) {
        std::cout << "\n--- " << sys.name << " ---" << std::endl;
        
        runSPPGFWithErrorModel(sys.name, roverFile, navFile, outputPath,
                               "_full_model", true, true, true, true, sys.types);
        runSPPGFWithErrorModel(sys.name, roverFile, navFile, outputPath,
                               "_no_relativity", false, true, true, true, sys.types);
        runSPPGFWithErrorModel(sys.name, roverFile, navFile, outputPath,
                               "_no_earth_rotation", true, false, true, true, sys.types);
        runSPPGFWithErrorModel(sys.name, roverFile, navFile, outputPath,
                               "_no_trop", true, true, true, false, sys.types);
        runSPPGFWithErrorModel(sys.name, roverFile, navFile, outputPath,
                               "_no_tgd", true, true, false, true, sys.types);
    }
    
    std::cout << "\n=== All SPPGF error model tests completed ===" << std::endl;
    return 0;
}
