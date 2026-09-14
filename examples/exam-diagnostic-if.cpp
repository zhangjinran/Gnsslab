/**
 * SPP IF 定位逐项验证（对照课本例 6-1）
 * 使用 SPPIFCode，仅处理 GPS 第一历元
 */
#include <iostream>
#include <iomanip>
#include "SPPIFCode.h"
#include "RinexNavStore.hpp"
#include "RinexObsReader.h"

using namespace std;

int main() {
    string dir = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";

    RinexNavStore nav;
    string navFile = dir + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    nav.loadFile(const_cast<string&>(navFile));

    string obsFile = dir + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    fstream obsStream(obsFile);
    RinexObsReader reader;
    reader.setFileStream(&obsStream);

    // 仅选 C1W + C2W
    map<string, set<string>> sel;
    sel["G"].insert("C1W");
    sel["G"].insert("C2W");
    reader.setSelectedTypes(sel);

    // 找到第一历元
    ObsData data;
    while (true) {
        try { data = reader.parseRinexObs(); }
        catch (EndOfFile&) { return -1; }
        reader.chooseObs(data);
        break;
    }

    // 只保留课本中 8 颗卫星
    vector<string> targets = {"G10","G12","G23","G24","G25","G28","G31","G32"};
    SatTypeValueMap keep;
    for (auto& st : data.satTypeValueData)
        for (auto& t : targets)
            if (st.first.toString() == t) keep.insert(st);
    data.satTypeValueData = keep;

    cout << "历元: " << CommonTime2CivilTime(data.epoch) << endl;
    cout << "卫星数: " << data.satTypeValueData.size() << endl;

    // IF 组合类型：GPS L1=C1, L2=C2
    map<string, pair<string,string>> ifTypes;
    ifTypes["G"] = {"C1", "C2"};

    SPPIFCode sppif;
    sppif.setRinexNavStore(&nav);
    sppif.setSystemCode("G");
    sppif.setIFCodeTypes(ifTypes);
    sppif.setSelectedTypes(sel);

    // 执行定位
    try {
        sppif.solve(data, false, true);  // TGD=false, Trop=true
    } catch (const exception& e) {
        cerr << "solve 异常: " << e.what() << endl;
        return -1;
    }

    Vector3d xyz = sppif.getXYZ();
    cout << "\n最终位置: " << fixed << setprecision(3) << xyz.transpose() << endl;

    return 0;
}
