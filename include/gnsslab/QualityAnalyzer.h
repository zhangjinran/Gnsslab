/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *  As stipulated by the MulanPSL-2.0, you are granted the following freedoms:
 *      To copy, use, and modify the software;
 *      To use the software for commercial purposes;
 *      To redistribute the software.
 *
 * Author: Shoujian Zhang, shjzhang@sgg.whu.edu.cn, 2024-10-10
 *
 * Reference: BD 420022-2019
 *  北斗/全球卫星导航系统（GNSS）测量型接收机观测数据质量评估方法
 */

#ifndef GNSSLAB_QUALITYANALYZER_H
#define GNSSLAB_QUALITYANALYZER_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <fstream>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/TimeConvert.h>
#include <gnsslab/CoordConvert.h>
#include <gnsslab/Const.h>
#include <gnsslab/RinexNavStore.hpp>

using namespace std;

// ============================================================
// —— 一个频点的观测值和衍生分析值 ——
// ============================================================
struct FreqData {
    double C = 0.0;  // 伪距（米）
    double L = 0.0;  // 载波相位（米）
    double D = 0.0;  // 多普勒

    double MP           = 0.0;  // 多路径
    double tripleDiffC  = 0.0;  // 伪距三次差分
    double tripleDiffL  = 0.0;  // 载波相位三次差分
};

// ============================================================
// —— 每颗卫星每个历元的完整观测记录 ——
// ============================================================
struct EpochRecord {
    CommonTime time;
    int seqIdx = 0;

    // 按频点号索引，由 feed() 自动构建
    // key: 频点号（RINEX 观测码第二个字符的数字）
    // 例: GPS L1/L2 → freqObs[1]={C1,L1}, freqObs[2]={C2,L2}
    map<int, FreqData> freqObs;

    // —— 双频组合衍生值（由 setDualFreqPair 指定频点对计算） ——
    double L_MW = 0.0;  // MW 组合值
    double L_GF = 0.0;  // GF 组合值
    double P_GF = 0.0;  // 伪距电离残差
    double I1   = 0.0;  // 电离层计算量（pair 的频点1）
    double I2   = 0.0;  // 电离层计算量（pair 的频点2）

    // —— 辅助数据 ——
    double elevation = 0.0;  // 高度角（度）
    double azimuth   = 0.0;  // 方位角（度）

    // —— 质量标记 ——
    bool isOutlier   = false;  // MW 三点法判为粗差
    bool csByMW      = false;  // MW 组合检测到周跳
    bool csByGF      = false;  // GF 组合检测到周跳
    bool hasClockJump = false; // 该历元存在接收机钟跳
    double clockJumpDelta = 0.0; // 钟跳探测量 ΔL（m）
    bool excluded    = false;  // 各种原因排除

    bool hasCycleSlip() const { return csByMW || csByGF; }
};

// ============================================================
// —— 一颗卫星的完整弧段数据 ——
// ============================================================
struct MPAccum {
    double sum   = 0.0;  // 多路径值累加
    int    count = 0;    // 有效历元数
};

struct SatRecord {
    string prn;                     // "G01", "C11"...
    string system;                  // "G", "C", "E"...
    vector<EpochRecord> epochs;     // 所有历元
    set<int> observedFreqs;         // 出现过的频点号集合 {1,2,5}...

    // MW 滑动窗口状态
    double mwMean  = 0.0;
    double mwSigma = 0.0;
    int    mwCount = 0;

    // 三点法暂态：前一历元是否被标记为"疑似"（等待下一历元确认）
    bool   hasSuspicious = false;
    double suspiciousMW  = 0.0;  // 疑似历元的 MW 值
    double suspiciousMean = 0.0; // 疑似时的窗口均值
    double suspiciousSigma = 0.0;// 疑似时的窗口方差
    int    suspiciousCount = 0;  // 疑似时的窗口大小

    // 多路径累加器（逐频点）
    map<int, MPAccum> mpAccum;     // freqNum → MPAccum
    map<int, double> mpMean;       // 去均值后的频点均值
};

// ============================================================
// —— 双频组合配置 ——
// ============================================================
struct DualFreqPair {
    int f1 = 0;  // 频点号1
    int f2 = 0;  // 频点号2
};

// ============================================================
// —— 质量分析主类 ——
// ============================================================
class QualityAnalyzer {
public:
    QualityAnalyzer();
    ~QualityAnalyzer() = default;

    // ========== 配置：双频组合（周跳/电离层） ==========
    void setDualFreqPair(const string& sys, int freqNum1, int freqNum2);

    // ========== 通用配置 ==========
    void setCutOffElevation(double deg)        { cutoffElev = deg; }
    void setMWSigmaThreshold(double sigma)     { mwSigmaThreshold = sigma; }
    void setGFPolyOrder(int order)             { gfPolyOrder = order; }
    void setGFSlipThreshold(double threshold)  { gfSlipThreshold = threshold; }
    void setMultipathWindow(int n)             { mpWindow = n; }
    void setMPMaxValid(double m)               { mpMaxValid = m; }

    // ========== 导航星历 ==========
    void setNavStore(RinexNavStore* navStore)  { navStorePtr = navStore; }

    // ========== Phase 1：逐历元处理 ==========
    void feed(const ObsData& obsData);

    // ========== Phase 2：GF 组合全局拟合 ==========
    void detectGFSlips();

    // ========== Phase 3：后处理分析 ==========
    void analyzeMultipath();
    void analyzeNoise();
    void analyzeIonoResidual();
    void detectClockJumps();

    // ========== 导出 ==========
    void exportIntegrity(const string& outDir);
    void exportCSResult(const string& outDir);
    void exportMultipath(const string& outDir);
    void exportNoise(const string& outDir);
    void exportIono(const string& outDir);
    void exportSummary(const string& outDir);

    // ========== 全流程 ==========
    /// 全流程运行：读取观测文件→分析→导出到指定目录
    /// @param obsFile  RINEX 观测文件路径
    /// @param navFile  RINEX 导航文件路径
    /// @param outputDir 输出目录（必须指定绝对路径或运行时 CWD 相对路径）
    void processFile(const string& obsFile,
                     const string& navFile,
                     const string& outputDir);
    void printReport(ostream& os = cout) const;

    // ========== 弧段结构 ==========
    struct ArcSegment {
        size_t startIdx = 0;  // epochs 中的起始索引
        size_t endIdx   = 0;  // 结束索引（不包含）
        int length() const { return static_cast<int>(endIdx - startIdx); }
    };

    /// 根据 csByMW/csByGF 将卫星的 epochs 分割为连续弧段
    static vector<ArcSegment> splitArcs(const vector<EpochRecord>& epochs);

private:
    // —— 内部方法 ——
    int extractFreqNum(const string& obsType) const;
    void parseSatObs(const SatID& sat, const TypeValueMap& tvm,
                     EpochRecord& epoch);
    void computeElevAzimForSat(const SatID& sat, EpochRecord& epoch,
                               const CommonTime& epochTime,
                               const XYZ& refPos);
    void detectCSMW(SatRecord* sat, EpochRecord& epoch);

    // —— 配置数据 ——
    map<string, DualFreqPair> dualFreqPairs;  // 系统→双频对
    double cutoffElev    = 10.0;
    double mwSigmaThreshold = 4.0;
    int    gfPolyOrder   = 5;
    double gfSlipThreshold = 0.05;  // 米
    int    mpWindow      = 50;
    double mpMaxValid    = 100.0;   // MP 有效范围 ±100m（去均值后过滤）

    // —— 运行时数据 ——
    map<string, unique_ptr<SatRecord>> satRecords;  // prn → SatRecord
    int totalEpochs = 0;           // 理论历元总数
    int intervalSec = 30;          // 采样间隔（秒），从 RINEX header 获取

    // 完整率统计（feed() 中实时更新）
    map<string, map<int, int>> freqActualEpoch;  // sys→freqNum→所有卫星累计历元数
    map<string, int> systemSatCount;             // sys→该系统的卫星数量
    map<string, set<int>> systemFreqs;           // sys→该系统共有的频点集合
    map<string, int> systemCompleteEpoch;        // sys→所有频点均有数据的历元数(ΣCj)

    // 周跳统计
    map<string, int> csMWCount;   // 系统→MW检测周跳数
    map<string, int> csGFCount;   // 系统→GF检测周跳数

    // 噪声统计（三差法中间量）
    struct NoiseStats {
        double sumTripleSq = 0.0;  // Σ(ΔΔΔ)²
        int    count = 0;           // 三差个数 N
    };
    map<string, map<int, NoiseStats>> noisePR;  // sys→freq→伪距噪声
    map<string, map<int, NoiseStats>> noiseCP;  // sys→freq→载波噪声

    // 电离层残差统计
    map<string, map<int, int>> ionoJumpCount;   // sys→freq→IOD>0.07 跳变次数

    // 钟跳统计
    map<string, int> clockJumpCount;            // sys→检测到钟跳的历元数

    // 测站坐标（从 RINEX header 获取）
    XYZ refPos;
    bool hasRefPos = false;

    // 导航星历
    RinexNavStore* navStorePtr = nullptr;
};

#endif //GNSSLAB_QUALITYANALYZER_H
