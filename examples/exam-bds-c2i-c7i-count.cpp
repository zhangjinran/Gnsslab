#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "GnssStruct.h"
#include "RinexObsReader.h"

using namespace std;

int main() {
    string obsFile = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/WUH200CHN_R_20250010000_01D_30S_MO.rnx";

    RinexObsReader obsReader;
    obsReader.loadFile(obsFile);
    
    int totalEpoch = 0;
    int totalSatellites = 0;
    int maxSatellites = 0;
    int minSatellites = 999;
    
    cout << "===== BDS C2I + C7I 卫星统计 =====" << endl;
    cout << "正在处理观测文件..." << endl;
    
    try {
        while (true) {
            ObsData obsData;
            try {
                obsData = obsReader.parseRinexObs();
            } catch (const std::exception& e) {
                // 遇到 EOF 错误时正常结束循环
                std::string errMsg = e.what();
                if (errMsg.find("EOF") != std::string::npos) {
                    cout << "\n观测文件读取完毕（EOF）" << endl;
                    break;
                }
                // 其他错误则报错退出
                std::cerr << "Error: " << e.what() << std::endl;
                return 1;
            }

            if (obsData.satTypeValueData.empty()) {
                break;
            }

            // 统计北斗系统中同时有 C2I 和 C7I 的卫星数量
            int count = obsReader.countDualCodeSatellites(obsData, "C", "C2I", "C7I");

            totalEpoch++;
            totalSatellites += count;
            maxSatellites = max(maxSatellites, count);
            minSatellites = min(minSatellites, count);

            // 每100个历元输出进度
            if (totalEpoch % 100 == 0) {
                cout << "已处理 " << totalEpoch << " 个历元..." << endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    cout << "-----------------------------------------" << endl;
    cout << "统计汇总:" << endl;
    cout << "总历元数: " << totalEpoch << endl;
    cout << "平均每历元C2I+C7I卫星数: " << fixed << setprecision(2) 
         << (double)totalSatellites / totalEpoch << endl;
    cout << "最大C2I+C7I卫星数: " << maxSatellites << endl;
    cout << "最小C2I+C7I卫星数: " << minSatellites << endl;
    
    return 0;
}