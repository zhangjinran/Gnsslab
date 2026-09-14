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

#include "QualityAnalyzer.h"
#include "GnssFunc.h"
#include "RinexObsReader.h"
#include <cmath>
#include <iomanip>
#include <filesystem>

#define DEBUG_QA 0

// ============================================================
// 构造函数
// ============================================================
QualityAnalyzer::QualityAnalyzer() {}

// ============================================================
// 配置接口
// ============================================================
void QualityAnalyzer::setDualFreqPair(const string& sys, int freqNum1, int freqNum2)
{
    dualFreqPairs[sys] = {freqNum1, freqNum2};
}

// ============================================================
// 工具方法：从观测类型名提取频点号
// 例如 "C1"→1, "L2"→2, "C5"→5, "L6"→6
// ============================================================
int QualityAnalyzer::extractFreqNum(const string& obsType) const
{
    if (obsType.length() < 2) return 0;
    // 第二个字符是频点号
    char c = obsType[1];
    if (c >= '0' && c <= '9') return c - '0';
    return 0;
}

// ============================================================
// 解析一颗卫星的观测值 → 填入 EpochRecord::freqObs
// ============================================================
void QualityAnalyzer::parseSatObs(const SatID& sat, const TypeValueMap& tvm,
                                  EpochRecord& epoch)
{
    for (const auto& [obsType, value] : tvm) {
        if (obsType.empty()) continue;
        char prefix = obsType[0];   // C/L/D/S
        int freqNum = extractFreqNum(obsType);

        if (freqNum <= 0) continue;

        if (prefix == 'C') {
            epoch.freqObs[freqNum].C = value;
        } else if (prefix == 'L') {
            epoch.freqObs[freqNum].L = value;
        } else if (prefix == 'D') {
            epoch.freqObs[freqNum].D = value;
        }
        // 'S' (信噪比) 跳过
    }
}

// ============================================================
// 计算单颗卫星的高度角/方位角
// ============================================================
void QualityAnalyzer::computeElevAzimForSat(const SatID& sat, EpochRecord& epoch,
                                             const CommonTime& epochTime,
                                             const XYZ& refPosXYZ)
{
    if (!navStorePtr) return;

    try {
        Xvt satXvt = navStorePtr->getXvt(sat, epochTime);
        XYZ satXYZ = satXvt.x;
        epoch.elevation = elevation(refPosXYZ, satXYZ);
        epoch.azimuth   = azimuth(refPosXYZ, satXYZ);
    } catch (...) {
        // 无法计算高度角时不处理
    }
}

// ============================================================
// MW 组合周跳探测（三点法：延迟一历元区分粗差与周跳）
// ============================================================
void QualityAnalyzer::detectCSMW(SatRecord* sat, EpochRecord& epoch)
{
    auto it = dualFreqPairs.find(sat->system);
    if (it == dualFreqPairs.end()) return;

    int p1 = it->second.f1;
    int p2 = it->second.f2;

    auto& freq = epoch.freqObs;
    if (freq.count(p1) == 0 || freq.count(p2) == 0) return;
    if (freq[p1].L == 0.0 || freq[p2].L == 0.0) return;
    if (freq[p1].C == 0.0 || freq[p2].C == 0.0) return;

    // GLONASS FDMA 固有多路径/电离层残差，使用更宽松的阈值
    double sysSigmaMult = (sat->system == "R") ? mwSigmaThreshold * 2.5 : mwSigmaThreshold;
    double f1, f2;
    if (sat->system == "R" && navStorePtr) {
        try {
            SatID satId(sat->prn);
            NavEphGLONASS gloEph = navStorePtr->findGLOEph(satId, epoch.time);
            f1 = gloEph.getFreq("L1");
            f2 = gloEph.getFreq("L2");
        } catch (std::exception& e) {
            cerr << "[QA] GLO freq ERROR " << sat->prn
                 << ": " << e.what() << endl;
            f1 = getFreq(sat->system, p1);
            f2 = getFreq(sat->system, p2);
        } catch (...) {
            cerr << "[QA] GLO freq UNKNOWN ERROR " << sat->prn << endl;
            f1 = getFreq(sat->system, p1);
            f2 = getFreq(sat->system, p2);
        }
    } else {
        f1 = getFreq(sat->system, p1);
        f2 = getFreq(sat->system, p2);
    }
    if (f1 == 0.0 || f2 == 0.0) return;

    // ---- 计算 MW 组合值 ----
    double L1 = freq[p1].L, L2 = freq[p2].L;
    double C1 = freq[p1].C, C2 = freq[p2].C;
    double mwValue = (f1 * L1 - f2 * L2) / (f1 - f2)
                   - (f1 * C1 + f2 * C2) / (f1 + f2);
    epoch.L_MW = mwValue;

    // ---- 三点法：先处理前一历元的疑似状态 ----
    if (sat->hasSuspicious) {
        // 正在等待确认：检查当前历元的 MW 值
        double oldMean = sat->suspiciousMean;
        double biasToOld = fabs(mwValue - oldMean);
        double biasToSusp = fabs(mwValue - sat->suspiciousMW);
        double threshold = sysSigmaMult * sqrt(sat->suspiciousSigma);

        if (biasToOld < threshold) {
            // 当前历元回到旧均值 → 前一历元是粗差，不是周跳
            // 回退前一历元的标记
            if (!sat->epochs.empty()) {
                sat->epochs.back().csByMW = false;
                sat->epochs.back().isOutlier = true;
            }
            // 恢复窗口状态（等同于未发生过疑似）
            sat->mwMean  = sat->suspiciousMean;
            sat->mwSigma = sat->suspiciousSigma;
            sat->mwCount = sat->suspiciousCount;
        } else if (biasToSusp < threshold * 0.5) {
            // 当前历元与前一疑似历元接近（均远离均值）→ 周跳成立
            // 前一历元已经是 csByMW=true，无需修改
            // 重置窗口：均值随新弧段更新，方差保留（噪声特性不变）
            csMWCount[sat->system]++;
            sat->mwMean  = mwValue;
            // 方差不重置！保留历史收敛值作为新弧段初始方差
            sat->mwCount = 1;
        } else {
            // 不确定：前一历元标记为 CS，当前也重新开始
            csMWCount[sat->system]++;
            sat->mwMean  = mwValue;
            // 方差不重置
            sat->mwCount = 1;
        }
        sat->hasSuspicious = false;
        // 当前历元的 MW 已作为新窗口起点，直接返回
        return;
    }

    // ---- 正常滑动窗口更新 ----
    if (sat->mwCount == 0) {
        // 第一个历元：初始化（初始方差按系统噪声水平设置，避免σ太小时首个历元误触发）
        sat->mwMean  = mwValue;
        sat->mwSigma = (sat->system == "R") ? 9.0 : 0.25;
        sat->mwCount = 1;
        return;
    }

    double bias = fabs(mwValue - sat->mwMean);
    double sigLimit = sysSigmaMult * sqrt(sat->mwSigma);

    if (bias > sigLimit) {
        // ---- 超限：进入"疑似"状态，等待下一历元确认 ----
        epoch.csByMW = true;  // 先标记，三点法确认后可能取消
        // 保存当前窗口状态（用于可能的回退）
        sat->hasSuspicious  = true;
        sat->suspiciousMW   = mwValue;
        sat->suspiciousMean = sat->mwMean;
        sat->suspiciousSigma = sat->mwSigma;
        sat->suspiciousCount = sat->mwCount;
        // 此时不更新窗口，等待下一历元判断
    } else {
        // ---- 正常：更新滑动窗口 ----
        sat->mwCount++;
        double n = static_cast<double>(sat->mwCount);
        double mwBias = mwValue - sat->mwMean;
        sat->mwMean += mwBias / n;
        // 递推方差: Var(n) = Var(n-1) + (bias²/n - Var(n-1))/n
        sat->mwSigma += (mwBias * mwBias - sat->mwSigma) / n;
    }
}

// ============================================================
// Phase 1：逐历元处理（核心：频点自动发现 + MW 周跳）
// ============================================================
void QualityAnalyzer::feed(const ObsData& obsData)
{
    totalEpochs++;
    CommonTime epochTime = obsData.epoch;
    XYZ refPosXYZ = obsData.antennaPosition;
    if (!hasRefPos && refPosXYZ.norm() > 0) {
        refPos = refPosXYZ;
        hasRefPos = true;
    }

    // 遍历每颗卫星
    for (const auto& [sat, tvm] : obsData.satTypeValueData) {
        string prn = sat.toString();

        // 获取或创建 SatRecord
        SatRecord* satRec;
        auto it = satRecords.find(prn);
        if (it == satRecords.end()) {
            auto newRec = make_unique<SatRecord>();
            newRec->prn = prn;
            newRec->system = sat.system;
            satRec = newRec.get();
            satRecords[prn] = move(newRec);
        } else {
            satRec = it->second.get();
        }

        // 构建 EpochRecord
        EpochRecord epoch;
        epoch.time = epochTime;
        epoch.seqIdx = satRec->epochs.size();

        // 解析观测值（自动发现频点）
        parseSatObs(sat, tvm, epoch);


        // 如果该卫星没有任何有效频点，跳过
        if (epoch.freqObs.empty()) continue;

        // 记录该卫星出现过的频点 + 系统频点
        for (const auto& [freqNum, _] : epoch.freqObs) {
            satRec->observedFreqs.insert(freqNum);
            systemFreqs[sat.system].insert(freqNum);
        }

        // 计算高度角
        if (hasRefPos && navStorePtr) {
            computeElevAzimForSat(sat, epoch, epochTime, refPos);
        }

        // 高度角筛选
        if (epoch.elevation > 0.0 && epoch.elevation < cutoffElev) {
            continue;  // 低于截止角，跳过该卫星
        }

        // 首次确认纳入分析的卫星：计数（必须在所有 continue 之后）
        if (satRec->epochs.empty()) {
            systemSatCount[sat.system]++;
        }

        // 完整率统计：每个频点计数
        for (const auto& [freqNum, fd] : epoch.freqObs) {
            freqActualEpoch[sat.system][freqNum]++;
        }

        // MW 组合周跳探测（三点法）
        detectCSMW(satRec, epoch);

        // 存入弧段
        satRec->epochs.push_back(move(epoch));
    }
}

// ============================================================
// GF 组合全局拟合：多项式拟合 P_GF → 检测 L_GF 跳变
// ============================================================
void QualityAnalyzer::detectGFSlips()
{
    for (auto& [prn, satRec] : satRecords) {
        auto it = dualFreqPairs.find(satRec->system);
        if (it == dualFreqPairs.end()) continue;
        int p1 = it->second.f1;
        int p2 = it->second.f2;
        int n = satRec->epochs.size();
        if (n < 20) continue;  // 太少历元无法拟合

        // 收集有效历元的 P_GF 和 L_GF
        vector<double> times, p_gf, l_gf;
        vector<int> idxMap;  // 有效历元→原始epoch索引
        double t0 = CommonTime2YDSTime(satRec->epochs[0].time).sod;

        for (int i = 0; i < n; i++) {
            auto& e = satRec->epochs[i];
            auto f1 = e.freqObs.find(p1);
            auto f2 = e.freqObs.find(p2);
            if (f1 == e.freqObs.end() || f2 == e.freqObs.end()) continue;
            if (f1->second.C == 0 || f2->second.C == 0) continue;
            if (f1->second.L == 0 || f2->second.L == 0) continue;
            // 跳过有 MW 周跳的历元（MW 已检测到的跳变）
            if (e.csByMW) continue;

            double t = CommonTime2YDSTime(e.time).sod - t0;
            double P_GF = f2->second.C - f1->second.C;
            double L_GF = f2->second.L - f1->second.L;
            times.push_back(t);
            p_gf.push_back(P_GF);
            l_gf.push_back(L_GF);
            idxMap.push_back(i);
        }

        int m = times.size();
        if (m < 20) continue;

        // ---- k 阶多项式拟合 P_GF(t) = a₀ + a₁·t + ... + aₖ·tᵏ ----
        int order = min(gfPolyOrder, m / 3);
        Eigen::MatrixXd A(m, order + 1);
        Eigen::VectorXd b(m);
        // 将时间归一化到 [-1, 1] 改善数值稳定性
        double tScale = (times.back() - times.front()) / 2.0;
        double tMid   = (times.back() + times.front()) / 2.0;

        for (int i = 0; i < m; i++) {
            double tn = (times[i] - tMid) / tScale;  // ≈[-1, 1]
            double tp = 1.0;
            for (int j = 0; j <= order; j++) {
                A(i, j) = tp;
                tp *= tn;
            }
            b(i) = p_gf[i];
        }

        // QR 最小二乘求解
        Eigen::VectorXd x = A.householderQr().solve(b);

        // ---- 计算残差 L_GF - Q_GF ----
        vector<double> residual(m);
        double resSum = 0, resSumSq = 0;
        for (int i = 0; i < m; i++) {
            double tn = (times[i] - tMid) / tScale;
            double tp = 1.0, q_gf = 0;
            for (int j = 0; j <= order; j++) {
                q_gf += x(j) * tp;
                tp *= tn;
            }
            residual[i] = l_gf[i] - q_gf;
            resSum += residual[i];
            resSumSq += residual[i] * residual[i];
        }
        double resMean = resSum / m;
        double resStd  = sqrt(resSumSq / m - resMean * resMean);

        // ---- 跳变检测：|Δ(residual)| > threshold ----
        double threshold = gfSlipThreshold;
        if (resStd > 0.01) threshold = max(gfSlipThreshold, 3.0 * resStd);

        for (int i = 1; i < m; i++) {
            double delta = fabs((residual[i] - resMean) - (residual[i-1] - resMean));
            if (delta > threshold) {
                int epochIdx = idxMap[i];
                satRec->epochs[epochIdx].csByGF = true;
                csGFCount[satRec->system]++;
            }
        }
    }
}

// ============================================================
// 多路径误差分析（BD 420022-2019）
// 对所有观测频点计算 MP，逐频点选择合适的参考频率：
//   - f₁ → 参考 f₂
//   - f₂ → 参考 f₁
//   - 其他频点 fₖ → 参考 f₁
// MP_i = P_i - (1 + 2/(α-1))·L_i + (2/(α-1))·L_ref
// α = (f_i / f_ref)²
// ============================================================
void QualityAnalyzer::analyzeMultipath()
{
    const size_t MIN_ARC_LEN = 2;  // 最短弧段

    for (auto& [prn, satRec] : satRecords) {
        auto it = dualFreqPairs.find(satRec->system);
        if (it == dualFreqPairs.end()) continue;

        int f1 = it->second.f1;
        int f2 = it->second.f2;

        // GLONASS FDMA 频率
        double freq1, freq2;
        if (satRec->system == "R" && navStorePtr && !satRec->epochs.empty()) {
            try {
                SatID satId(satRec->prn);
                CommonTime t0 = satRec->epochs[0].time;
                NavEphGLONASS gloEph = navStorePtr->findGLOEph(satId, t0);
                freq1 = gloEph.getFreq("L1");
                freq2 = gloEph.getFreq("L2");
            } catch (...) {
                freq1 = getFreq(satRec->system, f1);
                freq2 = getFreq(satRec->system, f2);
            }
        } else {
            freq1 = getFreq(satRec->system, f1);
            freq2 = getFreq(satRec->system, f2);
        }
        if (freq1 == 0.0 || freq2 == 0.0) continue;

        double alpha12 = (freq1 / freq2) * (freq1 / freq2);
        double coeff12 = (fabs(alpha12 - 1.0) < 1e-10) ? 0.0 : 2.0 / (alpha12 - 1.0);
        double alpha21 = (freq2 / freq1) * (freq2 / freq1);
        double coeff21 = (fabs(alpha21 - 1.0) < 1e-10) ? 0.0 : 2.0 / (alpha21 - 1.0);

        // 逐弧段处理：遇到 CS 就切弧段
        vector<size_t> arcIdx;              // 当前弧段的历元索引
        map<int, vector<double>> arcMP;     // 当前弧段: 频点 → MP值列表

        // 弧段结束时调用的处理函数
        auto flushArc = [&]() {
            if (arcIdx.size() >= MIN_ARC_LEN) {
                for (auto& [fn, mpVals] : arcMP) {
                    double sum = 0.0;
                    for (double v : mpVals) sum += v;
                    double mean = sum / mpVals.size();
                    for (size_t j = 0; j < mpVals.size(); j++) {
                        satRec->epochs[arcIdx[j]].freqObs[fn].MP = mpVals[j] - mean;
                    }
                }
            } else {
                for (size_t idx : arcIdx) {
                    for (auto& [fn, fd] : satRec->epochs[idx].freqObs) {
                        fd.MP = 0.0;
                    }
                }
            }
            arcIdx.clear();
            arcMP.clear();
        };

        for (size_t i = 0; i < satRec->epochs.size(); i++) {
            auto& epoch = satRec->epochs[i];

            // 遇到周跳 → 结束当前弧段
            if (epoch.hasCycleSlip()) {
                flushArc();
                continue;
            }

            // 正常历元：计算各频点 MP
            auto f1it = epoch.freqObs.find(f1);
            auto f2it = epoch.freqObs.find(f2);
            bool hasF1 = (f1it != epoch.freqObs.end() && f1it->second.L != 0.0);
            bool hasF2 = (f2it != epoch.freqObs.end() && f2it->second.L != 0.0);

            bool epochHasMP = false;
            for (auto& [freqNum, fd] : epoch.freqObs) {
                if (fd.C == 0.0 || fd.L == 0.0) continue;

                double MP = 0.0;
                if (freqNum == f1 && hasF2) {
                    MP = fd.C - (1.0 + coeff12) * fd.L + coeff12 * f2it->second.L;
                } else if (freqNum == f2 && hasF1) {
                    MP = fd.C - (1.0 + coeff21) * fd.L + coeff21 * f1it->second.L;
                } else if (freqNum != f1 && freqNum != f2 && hasF1) {
                    double f_i = getFreq(satRec->system, freqNum);
                    if (f_i == 0.0) continue;
                    double alpha = (f_i / freq1) * (f_i / freq1);
                    if (fabs(alpha - 1.0) < 1e-10) continue;
                    double coeff = 2.0 / (alpha - 1.0);
                    MP = fd.C - (1.0 + coeff) * fd.L + coeff * f1it->second.L;
                } else {
                    continue;
                }

                // 所有 MP 值参与弧段计算（导出时再过滤异常值）
                arcMP[freqNum].push_back(MP);
                epochHasMP = true;
            }

            if (epochHasMP) {
                arcIdx.push_back(i);
            }
        }

        // 处理最后一个弧段
        flushArc();
    }
}

// ============================================================
// ============================================================
// 观测值噪声分析（BD 420022-2019 6.5 & 6.6）
// 伪距噪声: (C-L) 三差 (公式21)
// 载波噪声: GF组合 (L1-L2) 历元间二次差
// 分弧段: 周跳切 + 数据间隙切
// ============================================================
void QualityAnalyzer::analyzeNoise()
{
    const size_t MIN_ARC_LEN = 4;
    const size_t MIN_ARC_GF  = 3;
    const int    MAX_GAP_MULT = 2;

    struct TriAccum {
        double sumSqPR = 0.0;
        double sumSqCP = 0.0;
        int    count   = 0;
        int    cpCount = 0;
    };

    for (auto& [prn, satRec] : satRecords) {
        map<int, TriAccum> freqAccum;
        map<int, vector<double>> arcC, arcL;
        map<int, vector<double>> arcL1G, arcL2G;
        vector<double> arcSod;
        int expInterval = 30;

        auto flushArc = [&]() {
            // (C-L)三差 → 伪距噪声
            for (auto& [fn, cv] : arcC) {
                if (cv.size() < MIN_ARC_LEN) continue;
                auto& lv = arcL[fn];
                if (lv.size() < MIN_ARC_LEN) continue;
                for (size_t j = 3; j < cv.size(); j++) {
                    if (cv[j]==0||cv[j-1]==0||cv[j-2]==0||cv[j-3]==0) continue;
                    if (lv[j]==0||lv[j-1]==0||lv[j-2]==0||lv[j-3]==0) continue;
                    double d3 = (cv[j]-lv[j]) - 3*(cv[j-1]-lv[j-1])
                              + 3*(cv[j-2]-lv[j-2]) - (cv[j-3]-lv[j-3]);
                    freqAccum[fn].sumSqPR += d3*d3;
                    freqAccum[fn].count++;
                }
            }
            // GF(L1-L2)二次差 → 载波噪声
            auto it = dualFreqPairs.find(satRec->system);
            if (it != dualFreqPairs.end()) {
                int f1 = it->second.f1, f2 = it->second.f2;
                auto& l1v = arcL1G[f1];
                auto& l2v = arcL2G[f2];
                size_t n = min(l1v.size(), l2v.size());
                if (n >= MIN_ARC_GF) {
                    for (size_t j = 2; j < n; j++) {
                        if (l1v[j]==0||l1v[j-1]==0||l1v[j-2]==0) continue;
                        if (l2v[j]==0||l2v[j-1]==0||l2v[j-2]==0) continue;
                        double gf0=l1v[j]-l2v[j], gf1=l1v[j-1]-l2v[j-1], gf2=l1v[j-2]-l2v[j-2];
                        double d2 = (gf0-gf1)-(gf1-gf2);
                        freqAccum[f1].sumSqCP += d2*d2;
                        freqAccum[f1].cpCount++;
                    }
                }
            }
            arcC.clear(); arcL.clear(); arcL1G.clear(); arcL2G.clear(); arcSod.clear();
        };

        for (size_t i = 0; i < satRec->epochs.size(); i++) {
            auto& ep = satRec->epochs[i];
            if (ep.hasCycleSlip()) { flushArc(); continue; }

            YDSTime yd = CommonTime2YDSTime(ep.time);
            double sod = yd.sod;

            // 数据间隙 → 切弧段
            if (!arcSod.empty()) {
                int gap = static_cast<int>(sod - arcSod.back());
                if (gap > expInterval * MAX_GAP_MULT) { flushArc(); }
                if (expInterval == 30 && gap > 1 && gap < 30) expInterval = gap;
            } else if (i + 1 < satRec->epochs.size()) {
                YDSTime yd1 = CommonTime2YDSTime(satRec->epochs[i+1].time);
                int est = static_cast<int>(yd1.sod - sod);
                if (est > 0 && est < 300) expInterval = est;
            }

            bool hasAny = false;
            for (auto& [fn, fd] : ep.freqObs) {
                if (fd.C == 0.0 && fd.L == 0.0) continue;
                arcC[fn].push_back(fd.C != 0.0 ? fd.C : 0.0);
                arcL[fn].push_back(fd.L != 0.0 ? fd.L : 0.0);
                arcL1G[fn].push_back(fd.L != 0.0 ? fd.L : 0.0);
                arcL2G[fn].push_back(fd.L != 0.0 ? fd.L : 0.0);
                hasAny = true;
            }
            if (hasAny) arcSod.push_back(sod);
        }
        flushArc();

        string sys = satRec->system;
        for (auto& [fn, acc] : freqAccum) {
            if (acc.count >= 2) {
                noisePR[sys][fn].sumTripleSq += sqrt(acc.sumSqPR / (8.0 * (acc.count - 1)));
                noisePR[sys][fn].count++;
            }
            if (acc.cpCount >= 2) {
                noiseCP[sys][fn].sumTripleSq += sqrt(acc.sumSqCP / (8.0 * (acc.cpCount - 1)));
                noiseCP[sys][fn].count++;
            }
        }
    }
}

// ============================================================
// 占位：电离层残差分析（下一阶段实现）
// ============================================================
// 电离层残差分析（BD 420022-2019 6.4，公式 18-19）
// 分弧段计算 IOD，统计 >0.07m/s 的跳变次数
// ============================================================
void QualityAnalyzer::analyzeIonoResidual()
{
    const double IOD_THRESHOLD = 0.07;  // m/s

    for (auto& [prn, satRec] : satRecords) {
        auto it = dualFreqPairs.find(satRec->system);
        if (it == dualFreqPairs.end()) continue;
        int f1 = it->second.f1;
        int f2 = it->second.f2;

        // GLONASS FDMA 频率
        double freq1, freq2;
        if (satRec->system == "R" && navStorePtr && !satRec->epochs.empty()) {
            try {
                SatID satId(satRec->prn);
                CommonTime t0 = satRec->epochs[0].time;
                NavEphGLONASS gloEph = navStorePtr->findGLOEph(satId, t0);
                freq1 = gloEph.getFreq("L1");
                freq2 = gloEph.getFreq("L2");
            } catch (...) {
                freq1 = getFreq(satRec->system, f1);
                freq2 = getFreq(satRec->system, f2);
            }
        } else {
            freq1 = getFreq(satRec->system, f1);
            freq2 = getFreq(satRec->system, f2);
        }
        if (freq1 == 0.0 || freq2 == 0.0) continue;

        // 公式 18 系数
        double f1sq = freq1 * freq1;
        double f2sq = freq2 * freq2;
        double denom = f1sq - f2sq;
        if (fabs(denom) < 1e-10) continue;
        double coeff1 = f2sq / denom;   // I1 = coeff1 * (L1-L2)
        double coeff2 = f1sq / denom;   // I2 = coeff2 * (L1-L2)

        // 逐弧段计算 IOD
        int    prevIdx = -1;
        double prevI1 = 0.0, prevI2 = 0.0;
        double prevSod = 0.0;

        for (size_t i = 0; i < satRec->epochs.size(); i++) {
            auto& epoch = satRec->epochs[i];
            if (epoch.hasCycleSlip()) {
                prevIdx = -1;  // 弧段结束，重置
                continue;
            }

            auto f1it = epoch.freqObs.find(f1);
            auto f2it = epoch.freqObs.find(f2);
            if (f1it == epoch.freqObs.end() || f2it == epoch.freqObs.end()) {
                prevIdx = -1; continue;
            }
            double L1 = f1it->second.L;
            double L2 = f2it->second.L;
            if (L1 == 0.0 || L2 == 0.0) { prevIdx = -1; continue; }

            double Ldiff = L1 - L2;
            double I1 = coeff1 * Ldiff;
            double I2 = coeff2 * Ldiff;
            epoch.I1 = I1;
            epoch.I2 = I2;

            YDSTime yd = CommonTime2YDSTime(epoch.time);
            double sod = yd.sod;

            if (prevIdx >= 0) {
                double dt = sod - prevSod;
                if (dt > 0) {
                    double iod1 = fabs(I1 - prevI1) / dt;
                    double iod2 = fabs(I2 - prevI2) / dt;
                    if (iod1 > IOD_THRESHOLD) ionoJumpCount[satRec->system][f1]++;
                    if (iod2 > IOD_THRESHOLD) ionoJumpCount[satRec->system][f2]++;
                }
            }

            prevIdx = i;
            prevI1 = I1;
            prevI2 = I2;
            prevSod = sod;
        }
    }
}

// ============================================================
// ============================================================
// 接收机钟跳探测（BD 420022-2019 6.2.4，公式 12-15）
// 需在周跳探测完成后运行（排除有周跳的卫星）
// ============================================================
void QualityAnalyzer::detectClockJumps()
{
    const double XI = 4.0;  // 观测噪声经验值（m）
    const double C = 299792458.0;  // 光速（m/s）

    // 公式 13: 毫秒级钟跳 (10⁻⁷·c - 3ξ) < ΔL < (10⁻⁵·c + 3ξ)
    double msLow  = 1e-7 * C - 3 * XI;   // ~18m
    double msHigh = 1e-5 * C + 3 * XI;   // ~3012m
    // 公式 14: 微秒级钟跳 ΔL > (10⁻³·c - 3ξ)
    double usLow  = 1e-3 * C - 3 * XI;   // ~299988m

    // 逐历元检查所有卫星
    // 需要知道每个历元有哪些卫星
    // 策略：对每个卫星，找连续的"无CS"历元，计算 ΔL
    for (auto& [prn, satRec] : satRecords) {
        string sys = satRec->system;
        for (size_t i = 1; i < satRec->epochs.size(); i++) {
            auto& ep = satRec->epochs[i];
            auto& epPrev = satRec->epochs[i-1];

            // 跳过有周跳的历元
            if (ep.hasCycleSlip() || epPrev.hasCycleSlip()) continue;

            // 取第一个有效频点的C/L值
            for (auto& [fn, fd] : ep.freqObs) {
                auto pIt = epPrev.freqObs.find(fn);
                if (pIt == epPrev.freqObs.end()) continue;
                if (fd.C == 0.0 || fd.L == 0.0) continue;
                if (pIt->second.C == 0.0 || pIt->second.L == 0.0) continue;

                // 公式 12: ΔL = (C(ti)-C(ti-1)) - (L(ti)-L(ti-1))
                double dC = fd.C - pIt->second.C;
                double dL = fd.L - pIt->second.L;
                double delta = fabs(dC - dL);
                ep.clockJumpDelta = delta;

                if (delta > msLow && delta < msHigh) {
                    ep.hasClockJump = true;
                    clockJumpCount[sys]++;
                } else if (delta > usLow) {
                    ep.hasClockJump = true;
                    clockJumpCount[sys]++;
                }
                break;  // 一个频点就够了
            }
        }
    }
}

// ============================================================
// 导出：观测数据完整率
// ============================================================
void QualityAnalyzer::exportIntegrity(const string& outDir)
{
    // 确保输出目录存在
    filesystem::create_directories(outDir);

    string filePath = outDir + "/integrity.txt";
    ofstream ofs(filePath);
    if (!ofs) {
        cerr << "[QA] Error: cannot write " << filePath << endl;
        return;
    }

    ofs << "# Format: system, freq_num, num_satellites, theoretical_epochs, actual_epochs, rate_percent" << endl;
    ofs << "# freq_num: RINEX 观测码第二个字符代表的频点号" << endl;
    ofs << "# Formula: DI_f = sum(actual) / (num_satellites * total_epochs) * 100" << endl;

    for (const auto& [sys, freqMap] : freqActualEpoch) {
        int nSat = systemSatCount[sys];
        if (nSat == 0) continue;
        double denominator = nSat * max(totalEpochs, 1);
        for (const auto& [freqNum, actual] : freqMap) {
            double rate = 100.0 * actual / denominator;
            ofs << sys << ","
                << freqNum << ","
                << nSat << ","
                << totalEpochs << ","
                << actual << ","
                << fixed << setprecision(2) << rate << endl;
        }
    }

    // DI_s：单系统完整率（用系统共有的全部频点逐历元检查）
    ofs << endl << "# DI_s: system_complete_rate" << endl;
    for (auto& [sys, freqs] : systemFreqs) {
        if (freqs.empty()) continue;
        int nSat = systemSatCount[sys];
        if (nSat == 0) continue;
        double denominator = nSat * max(totalEpochs, 1);

        int completeCount = 0;
        for (auto& [prn, satRec] : satRecords) {
            if (satRec->system != sys) continue;
            for (auto& epoch : satRec->epochs) {
                bool allOK = true;
                for (int fn : freqs) {
                    auto fit = epoch.freqObs.find(fn);
                    if (fit == epoch.freqObs.end() || fit->second.C == 0.0) {
                        allOK = false;
                        break;
                    }
                }
                if (allOK) completeCount++;
            }
        }

        double rate = 100.0 * completeCount / denominator;
        ofs << sys << ","
            << completeCount << ","
            << nSat << ","
            << totalEpochs << ","
            << fixed << setprecision(2) << rate << endl;
    }

    ofs.close();
    cout << "[QA] Exported: " << filePath << endl;
}

// ============================================================
// 导出：周跳结果（占位）
// ============================================================
void QualityAnalyzer::exportCSResult(const string& outDir)
{
    // TODO: Step 3/4 完成实现
    filesystem::create_directories(outDir);
    string filePath = outDir + "/cs_result.txt";
    ofstream ofs(filePath);
    if (!ofs) return;
    ofs << "# Format: ydoy, sod, sat, mw_value, mw_mean, mw_sigma, cs_by_mw(0/1), cs_by_gf(0/1), is_outlier(0/1), has_clock_jump(0/1)" << endl;
    // 遍历所有卫星的 EpochRecord 输出
    for (const auto& [prn, satRec] : satRecords) {
        for (const auto& epoch : satRec->epochs) {
            YDSTime yd = CommonTime2YDSTime(epoch.time);
            ofs << yd.year << ","
                << yd.doy << ","
                << fixed << setprecision(3) << yd.sod << ","
                << satRec->prn << ","
                << setprecision(3) << epoch.L_MW << ","
                << setprecision(3) << satRec->mwMean << ","
                << setprecision(3) << sqrt(satRec->mwSigma) << ","
                << (epoch.csByMW ? 1 : 0) << ","
                << (epoch.csByGF ? 1 : 0) << ","
                << (epoch.isOutlier ? 1 : 0) << ","
                << (epoch.hasClockJump ? 1 : 0)
                << endl;
        }
    }
    ofs.close();
    cout << "[QA] Exported: " << filePath << endl;
}

// ============================================================
// 导出：多路径结果
// ============================================================
void QualityAnalyzer::exportMultipath(const string& outDir)
{
    filesystem::create_directories(outDir);
    string filePath = outDir + "/multipath.txt";
    ofstream ofs(filePath);
    if (!ofs) {
        cerr << "[QA] Error: cannot write " << filePath << endl;
        return;
    }

    ofs << "# Format: ydoy, sod, prn, freq, mp(m), elevation" << endl;
    ofs << "# MP computed per satellite per frequency, demeaned" << endl;

    for (const auto& [prn, satRec] : satRecords) {
        for (const auto& epoch : satRec->epochs) {
            YDSTime yd = CommonTime2YDSTime(epoch.time);
            for (const auto& [freqNum, fd] : epoch.freqObs) {
                if (fd.MP == 0.0) continue;
                if (fabs(fd.MP) > mpMaxValid) continue;  // 过滤异常值
                ofs << yd.year << ","
                    << yd.doy << ","
                    << fixed << setprecision(3) << yd.sod << ","
                    << satRec->prn << ","
                    << freqNum << ","
                    << setprecision(3) << fd.MP << ","
                    << setprecision(1) << epoch.elevation
                    << endl;
            }
        }
    }
    ofs.close();
    cout << "[QA] Exported: " << filePath << endl;
}

// ============================================================
// ============================================================
// 导出：电离层残差结果
// ============================================================
void QualityAnalyzer::exportIono(const string& outDir)
{
    filesystem::create_directories(outDir);
    string filePath = outDir + "/iono_summary.txt";
    ofstream ofs(filePath);
    if (!ofs) {
        cerr << "[QA] Error: cannot write " << filePath << endl;
        return;
    }

    ofs << "# Format: system, freq, iono_jump_count" << endl;
    ofs << "# IOD > 0.07 m/s counted as ionospheric jump (formula 19)" << endl;

    for (auto& [sys, freqMap] : ionoJumpCount) {
        for (auto& [fn, count] : freqMap) {
            ofs << sys << ","
                << fn << ","
                << count << endl;
        }
    }
    ofs.close();
    cout << "[QA] Exported: " << filePath << endl;

    // 导出每历元电离层详情（用于时间序列绘图）
    string detailPath = outDir + "/iono_detail.txt";
    ofstream detOfs(detailPath);
    if (detOfs) {
        detOfs << "# Format: ydoy, sod, prn, I1(m), I2(m), elevation(deg)" << endl;
        for (const auto& [prn, satRec] : satRecords) {
            for (const auto& epoch : satRec->epochs) {
                if (epoch.I1 == 0.0 && epoch.I2 == 0.0) continue;
                YDSTime yd = CommonTime2YDSTime(epoch.time);
                detOfs << yd.year << ","
                       << yd.doy << ","
                       << fixed << setprecision(3) << yd.sod << ","
                       << satRec->prn << ","
                       << setprecision(3) << epoch.I1 << ","
                       << setprecision(3) << epoch.I2 << ","
                       << setprecision(1) << epoch.elevation
                       << endl;
            }
        }
        detOfs.close();
        cout << "[QA] Exported: " << detailPath << endl;
    }
}
// ============================================================
void QualityAnalyzer::exportNoise(const string& outDir)
{
    filesystem::create_directories(outDir);
    string filePath = outDir + "/noise_summary.txt";
    ofstream ofs(filePath);
    if (!ofs) {
        cerr << "[QA] Error: cannot write " << filePath << endl;
        return;
    }

    ofs << "# Format: system, freq, num_sats_pseudorange, pr_noise(m), num_sats_carrier, cp_noise(m)" << endl;
    ofs << "# Pseudorange noise (σρ) from formula 21; Carrier phase noise (σφ) from formula 23" << endl;

    for (auto& [sys, freqMap] : noisePR) {
        for (auto& [fn, stat] : freqMap) {
            double prNoise = (stat.count > 0) ? stat.sumTripleSq / stat.count : 0.0;
            int cpCount = 0;
            double cpNoise = 0.0;
            auto cpIt = noiseCP.find(sys);
            if (cpIt != noiseCP.end()) {
                auto cpFreq = cpIt->second.find(fn);
                if (cpFreq != cpIt->second.end()) {
                    cpCount = cpFreq->second.count;
                    cpNoise = (cpCount > 0) ? cpFreq->second.sumTripleSq / cpCount : 0.0;
                }
            }
            ofs << sys << ","
                << fn << ","
                << stat.count << ","
                << fixed << setprecision(4) << prNoise << ","
                << cpCount << ","
                << setprecision(6) << cpNoise
                << endl;
        }
    }
    ofs.close();
    cout << "[QA] Exported: " << filePath << endl;

    // 导出每历元噪声详情（(C-L)三差 + GF二次差，用于时间序列绘图）
    string detPath = outDir + "/noise_detail.txt";
    ofstream detOfs(detPath);
    if (!detOfs) return;
    detOfs << "# Format: ydoy, sod, prn, freq, d3CL(m), d2GF(m), elevation" << endl;
    detOfs << "# d3CL = (C-L) triple diff → pseudorange noise" << endl;
    detOfs << "# d2GF = GF (L1-L2) double diff → carrier phase noise" << endl;

    for (const auto& [prn, satRec] : satRecords) {
        auto& ep = satRec->epochs;
        auto dpIt = dualFreqPairs.find(satRec->system);
        int gf_f1 = (dpIt != dualFreqPairs.end()) ? dpIt->second.f1 : 0;
        int gf_f2 = (dpIt != dualFreqPairs.end()) ? dpIt->second.f2 : 0;
        double prevSod = 0;
        int expInterval = 30;

        for (size_t i = 3; i < ep.size(); i++) {
            // 周跳 → 跳过
            if (ep[i].hasCycleSlip() || ep[i-1].hasCycleSlip() ||
                ep[i-2].hasCycleSlip() || ep[i-3].hasCycleSlip())
                { prevSod = 0; continue; }

            YDSTime yd = CommonTime2YDSTime(ep[i].time);
            double sod = yd.sod;

            // 数据间隙 → 跳过
            if (prevSod > 0) {
                int gap = static_cast<int>(sod - prevSod);
                if (gap > 2 * expInterval) { prevSod = 0; continue; }
            }
            if (expInterval == 30) {
                int est = static_cast<int>(sod - CommonTime2YDSTime(ep[i-1].time).sod);
                if (est > 0 && est < 300) expInterval = est;
            }
            prevSod = sod;

            for (auto& [fn, fd] : ep[i].freqObs) {
                if (fd.C == 0.0 && fd.L == 0.0) continue;

                // (C-L) 三差
                double d3CL = 0.0;
                auto f1 = ep[i-1].freqObs.find(fn);
                auto f2 = ep[i-2].freqObs.find(fn);
                auto f3 = ep[i-3].freqObs.find(fn);
                if (f1 != ep[i-1].freqObs.end() && f2 != ep[i-2].freqObs.end() &&
                    f3 != ep[i-3].freqObs.end()) {
                    if (fd.C != 0 && f1->second.C != 0 && f2->second.C != 0 && f3->second.C != 0 &&
                        fd.L != 0 && f1->second.L != 0 && f2->second.L != 0 && f3->second.L != 0) {
                        d3CL = (fd.C - fd.L) - 3.0*(f1->second.C - f1->second.L)
                              + 3.0*(f2->second.C - f2->second.L) - (f3->second.C - f3->second.L);
                    }
                }

                // GF (L1-L2) 历元间二次差
                double d2GF = 0.0;
                if (gf_f1 > 0 && gf_f2 > 0 && i >= 2) {
                    auto l1_0 = ep[i].freqObs.find(gf_f1);
                    auto l1_1 = ep[i-1].freqObs.find(gf_f1);
                    auto l1_2 = ep[i-2].freqObs.find(gf_f1);
                    auto l2_0 = ep[i].freqObs.find(gf_f2);
                    auto l2_1 = ep[i-1].freqObs.find(gf_f2);
                    auto l2_2 = ep[i-2].freqObs.find(gf_f2);
                    if (l1_0 != ep[i].freqObs.end() && l1_1 != ep[i-1].freqObs.end() &&
                        l1_2 != ep[i-2].freqObs.end() && l2_0 != ep[i].freqObs.end() &&
                        l2_1 != ep[i-1].freqObs.end() && l2_2 != ep[i-2].freqObs.end()) {
                        auto& L1 = l1_0->second.L, L1m1 = l1_1->second.L, L1m2 = l1_2->second.L;
                        auto& L2 = l2_0->second.L, L2m1 = l2_1->second.L, L2m2 = l2_2->second.L;
                        if (L1 != 0 && L1m1 != 0 && L1m2 != 0 &&
                            L2 != 0 && L2m1 != 0 && L2m2 != 0) {
                            double gf0 = L1 - L2, gf1 = L1m1 - L2m1, gf2 = L1m2 - L2m2;
                            d2GF = (gf0 - gf1) - (gf1 - gf2);
                        }
                    }
                }

                detOfs << yd.year << "," << yd.doy << ","
                       << fixed << setprecision(3) << yd.sod << ","
                       << satRec->prn << "," << fn << ","
                       << setprecision(4) << d3CL << ","
                       << setprecision(6) << d2GF << ","
                       << setprecision(1) << ep[i].elevation << endl;
            }
        }
    }
    detOfs.close();
    cout << "[QA] Exported: " << detPath << endl;
}

// ============================================================
// 导出：汇总（占位）
// ============================================================
void QualityAnalyzer::exportSummary(const string& outDir)
{
    filesystem::create_directories(outDir);

    // 钟跳导出
    string cjPath = outDir + "/clock_jump.txt";
    ofstream cjOfs(cjPath);
    if (cjOfs) {
        cjOfs << "# Format: system, clock_jump_count" << endl;
        for (const auto& [sys, cnt] : clockJumpCount) {
            cjOfs << sys << "," << cnt << endl;
        }
        cjOfs.close();
        cout << "[QA] Exported: " << cjPath << endl;
    }

    // 导出每历元钟跳标记（用于绘图）
    string cjdPath = outDir + "/clock_jump_detail.txt";
    ofstream cjdOfs(cjdPath);
    if (cjdOfs) {
        cjdOfs << "# Format: ydoy, sod, prn, has_clock_jump(0/1), delta_L(m)" << endl;
        for (const auto& [prn, satRec] : satRecords) {
            for (const auto& epoch : satRec->epochs) {
                YDSTime yd = CommonTime2YDSTime(epoch.time);
                cjdOfs << yd.year << ","
                       << yd.doy << ","
                       << fixed << setprecision(3) << yd.sod << ","
                       << satRec->prn << ","
                       << (epoch.hasClockJump ? 1 : 0) << ","
                       << setprecision(2) << epoch.clockJumpDelta
                       << endl;
            }
        }
        cjdOfs.close();
        cout << "[QA] Exported: " << cjdPath << endl;
    }
}

// ============================================================
// 输出报告
// ============================================================
void QualityAnalyzer::printReport(ostream& os) const
{
    os << "\n========== GNSS 数据质量分析报告 ==========\n";
    os << "总历元数: " << totalEpochs << endl;
    os << "卫星数: " << satRecords.size() << endl;

    os << "\n--- 观测数据完整率 ---\n";
    for (const auto& [sys, freqMap] : freqActualEpoch) {
        int nSat = systemSatCount.at(sys);
        if (nSat == 0) continue;
        double denom = nSat * max(totalEpochs, 1);
        for (const auto& [freqNum, actual] : freqMap) {
            double rate = 100.0 * actual / denom;
            os << sys << " 频点" << freqNum << ": "
               << actual << "/" << totalEpochs
               << " x " << nSat << "sat"
               << " (" << fixed << setprecision(1) << rate << "%)" << endl;
        }
    }

    os << "\n--- 周跳统计 ---\n";
    for (const auto& [sys, cnt] : csMWCount) {
        os << sys << ": MW=" << cnt;
        auto gf = csGFCount.find(sys);
        if (gf != csGFCount.end() && gf->second > 0)
            os << " GF=" << gf->second;
        os << endl;
    }
    for (const auto& [sys, cnt] : csGFCount) {
        if (csMWCount.find(sys) == csMWCount.end())
            os << sys << ": GF=" << cnt << endl;
    }

    os << "\n--- 电离层跳变 (IOD > 0.07m/s) ---\n";
    for (const auto& [sys, freqMap] : ionoJumpCount) {
        for (const auto& [fn, cnt] : freqMap) {
            os << sys << " 频点" << fn << ": " << cnt << " 次" << endl;
        }
    }

    os << "\n--- 接收机钟跳 ---\n";
    if (clockJumpCount.empty()) {
        os << "  未检测到钟跳" << endl;
    } else {
        for (const auto& [sys, cnt] : clockJumpCount) {
            os << sys << ": " << cnt << " 次" << endl;
        }
    }

    os << "\n==========================================\n";
}

// ============================================================
// 全流程
// ============================================================
void QualityAnalyzer::processFile(const string& obsFile,
                                   const string& navFile,
                                   const string& outputDir)
{
    // 加载导航星历
    RinexNavStore navStore;
    try {
        string navFileCopy = navFile;
        navStore.loadFile(navFileCopy);
        setNavStore(&navStore);
    } catch (...) {
        cerr << "[QA] Warning: failed to load nav file: " << navFile << endl;
    }

    // 打开观测文件（parseRinexObs 首次调用时自动解析头部，不要显式调 parseRinexHeader）
    RinexObsReader reader;
    if (!reader.loadFile(obsFile)) {
        cerr << "[QA] Error: cannot open obs file: " << obsFile << endl;
        return;
    }

    // 逐历元处理
    int epochCount = 0;
    while (true) {
        ObsData obsData;
        try {
            obsData = reader.parseRinexObs();
        } catch (EndOfFile&) {
            break;
        } catch (...) {
            break;
        }
        convertObsType(obsData);
        feed(obsData);
        epochCount++;
    }

    cout << "[QA] Processed " << epochCount << " epochs." << endl;

    // 后处理
    detectGFSlips();
    analyzeMultipath();
    analyzeNoise();
    analyzeIonoResidual();
    detectClockJumps();

    // 导出
    exportIntegrity(outputDir);
    exportCSResult(outputDir);
    exportMultipath(outputDir);
    exportNoise(outputDir);
    exportIono(outputDir);
    exportSummary(outputDir);

    // 报告
    printReport(cout);
}
