/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *  As stipulated by the MulanPSL-2.0, you are granted the following freedoms:
 *      To copy, use, and modify the software;
 *      To use the software for commercial purposes;
 *      To redistribute the software.
 *
 * GNSS 观测数据质量分析示例程序
 *
 * 用法:
 *   quality_analysis <obsFile> <navFile> [outputDir]
 *
 * 示例:
 *   quality_analysis ../shixi/data/obs.rnx ../data/nav.rnx ../gnss_draw/data/quality/
 */

#include <iostream>
#include <string>
#include <gnsslab/QualityAnalyzer.h>

using namespace std;

int main(int argc, char* argv[])
{
    if (argc < 4) {
        cerr << "Usage: " << argv[0] << " <obsFile> <navFile> <outputDir>" << endl;
        cerr << "Example:" << endl;
        cerr << "  " << argv[0] << " ../data/WUH200CHN_R_20250010000_01D_30S_MO.rnx \\" << endl;
        cerr << "      ../data/BRDC00IGS_R_20250010000_01D_MN.rnx \\" << endl;
        cerr << "      ../../gnss_draw/data/quality/" << endl;
        return 1;
    }

    string obsFile = argv[1];
    string navFile = argv[2];
    string outDir  = argv[3];

    cout << "========================================" << endl;
    cout << "  GNSS 观测数据质量分析" << endl;
    cout << "  观测文件: " << obsFile << endl;
    cout << "  导航文件: " << navFile << endl;
    cout << "  输出目录: " << outDir << endl;
    cout << "========================================" << endl;

    // 1. 创建分析器
    QualityAnalyzer analyzer;
    analyzer.setCutOffElevation(10);
    analyzer.setMWSigmaThreshold(4.0);

    // 2. 配置双频组合（用频点号）
    analyzer.setDualFreqPair("G", 1, 2);   // GPS: L1/L2
    analyzer.setDualFreqPair("C", 2, 6);   // BDS: B1I/B3I
    analyzer.setDualFreqPair("E", 1, 5);   // Galileo: E1/E5a
    analyzer.setDualFreqPair("R", 1, 2);   // GLONASS: G1/G2（基准频率 k=0）
    analyzer.setDualFreqPair("J", 1, 2);   // QZSS: L1/L2
    // IRNSS: 仅 L5 单频，无法构成双频组合，跳过 MW 周跳探测

    // 3. 全流程运行（自动完成所有分析）
    analyzer.processFile(obsFile, navFile, outDir);

    return 0;
}
