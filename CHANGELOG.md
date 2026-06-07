# gnssLab 项目日志

---

### 2026-06-04

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | 创建 exam5.6 - SPP误差模型检测测试 | 测试程序 | ✅ 完成 |
| 2 | SPPCode添加误差模型控制开关（相对论效应、地球自转改正） | SPPCode | ✅ 完成 |
| 3 | SPPIFCode粗差检测添加硬阈值 | SPPIFCode | ✅ 完成 |
| 4 | 创建 exam6.4 - BDS SPPIF观测码组合测试 | 测试程序 | ✅ 完成 |
| 5 | 更新 claude.md 文档 | 项目文档 | ✅ 完成 |

**新增测试程序**:

1. **SPP误差模型检测工具** (`examples/exam-5.6-spp-error-model-test.cpp`):
   - **测试目标**: 检测相对论效应和地球自转改正两个误差模型的正确性
   - **测试方法**: 控制变量法，通过布尔开关控制误差模型的启用/禁用
   - **四种测试模式**:
     | 模式 | 相对论效应 | 地球自转改正 | 输出标识 |
     |------|-----------|-------------|---------|
     | 完整模型 | ✅ ON | ✅ ON | `_full_model` |
     | 无相对论 | ❌ OFF | ✅ ON | `_no_relativity` |
     | 无地球自转 | ✅ ON | ❌ OFF | `_no_earth_rotation` |
     | 无任何改正 | ❌ OFF | ❌ OFF | `_no_correction` |
   - **支持系统**: GPS、BDS、GLONASS、Galileo
   - **输出路径**: `gnss_draw/data/spp_error_model/`
   - **预期效果**: 相对论效应关闭后定位误差约数米；地球自转改正关闭后最大影响约数十米

2. **BDS SPPIF观测码组合测试** (`examples/exam-6.4-sppif-bds-test.cpp`):
   - **测试目标**: 测试北斗系统不同观测码组合的定位性能
   - **测试组合**（使用C1X、C2I、C5X、C6I、C7I）:
     - C1XC2I (B1C+B2I)
     - C1XC5X (B1C+B5A) ✅ 成功筛选
     - C1XC6I (B1C+B6I)
     - C1XC7I (B1C+B7I)
     - C2IC5X (B2I+B5A)
     - C2IC6I (B2I+B6I)
     - C2IC7I (B2I+B7I)
     - C5XC6I (B5A+B6I)
     - C5XC7I (B5A+B7I)
   - **大气校正模式**: 仅全改正模式（TGD + 对流层）
   - **输出路径**: `gnss_draw/data/sppif_bds/`
   - **测试结果**: ✅ C1X+C5X 组合成功筛选并运行

**代码修改**:

1. **SPPCode误差模型控制开关** (`lib/SPPCode.h/cpp`):
   - 添加 `relativityEnable` 和 `earthRotationEnable` 成员变量（默认值均为true）
   - 添加 `setRelativityEnable(bool)` 和 `setEarthRotationEnable(bool)` 方法
   - 修改 `computeAtTransmitTime()`: 根据开关控制是否应用相对论效应改正
   - 修改 `solve()`: 根据开关控制是否应用地球自转改正
   - **设计原则**: 使用默认参数，不修改原有函数签名，保持向后兼容性

2. **SPPIFCode粗差检测硬阈值改进** (`lib/SPPIFCode.cpp:921-940`):
   - **问题**: 原代码仅使用自适应阈值（`sigma0 * parameter`），当sigma0过大时阈值失效
   - **解决方案**: 添加30米硬阈值上限
   ```cpp
   double adaptiveThreshold = parameter * sigma0;
   const double MAX_THRESHOLD = 30.0;  // 最大阈值不超过30米
   double threshold = std::min(adaptiveThreshold, MAX_THRESHOLD);
   ```
   - **效果**: 确保即使sigma0很大，也能有效剔除粗差观测值

**文档更新**:

1. **claude.md**:
   - 添加 exam5.6 和 exam6.4 的条目到示例程序列表
   - 时间输出格式已为硬性规则（YDSTime格式）

**接口兼容性**:
- ✅ 所有修改均使用默认参数，不影响现有接口
- ✅ 不修改原有函数签名
- ✅ 保持向后兼容性

**测试数据分析方法**:
1. **相对论效应影响**: 对比 `_full_model` 和 `_no_relativity`，定位误差差异应在数米级别
2. **地球自转影响**: 对比 `_full_model` 和 `_no_earth_rotation`，随卫星位置变化，最大影响约数十米
3. **观测码组合对比**: 对比不同组合的定位精度、卫星数量、PDOP、收敛稳定性

---

### 2026-06-03

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | BDS卫星类型区分优化（基于轨道参数） | SPPCode | ✅ 完成 |
| 2 | 创建观测星数量统计工具 | exam-3.3 | ✅ 完成 |
| 3 | 粗差探测动态阈值优化 | SPPCode | ✅ 完成 |
| 4 | PDOP异常值检查 | SPPCode | ✅ 完成 |
| 5 | 修复 correctTGD 类型匹配问题 | SPPCode | ✅ 完成 |

**新增**:

1. **观测星数量统计工具** (`examples/exam-3.3-obs-statistics.cpp`):
   - 自动从RINEX头文件获取系统和观测类型信息
   - 按历元统计各观测类型的卫星数量
   - 每个系统生成独立的统计文件（MJD时间格式）
   - **统计结果**: C2I观测值数量最多，选择作为解算目标

2. **BDS卫星类型权重测试工具** (`examples/exam-5.5-bds-satweight-test.cpp`):
   - 支持配置不同的 MEO/IGSO/GEO 权重组合
   - 测试用例包括：(1.0, 0.3, 0.25) 和 GEO权重为0（彻底排除GEO）
   - 输出到独立目录 `gnss_draw/data/spp_bds_weight/`
   - 便于分析不同卫星类型权重对定位精度的影响

**优化**:

1. **BDS卫星类型区分** (`lib/SPPCode.cpp:850-935`):
   - 基于轨道参数（半长轴a、倾角i0）区分MEO/IGSO/GEO
   - 添加详细调试信息输出
   - **结论**: 对于C1X观测值，卫星类型区分对定位结果影响较小

2. **粗差探测动态阈值** (`lib/SPPCode.cpp:201-212`):
   - 根据sigma0自动调整粗差探测阈值：
     - sigma0 < 3: 不探测
     - 3 ~ 8: 5×sigma0
     - 8 ~ 15: 4×sigma0
     - 15 ~ 20: 3×sigma0
   - **原因**: 定位结果显示RMS极端值很大但中位数仅约10米，表明存在少量异常历元

3. **PDOP异常值检查** (`lib/SPPCode.cpp:175-188`):
   - 添加NaN、无穷大、负数检查
   - 跳过异常历元，避免无效定位结果

**修复**:

1. **correctTGD函数类型匹配** (`lib/SPPCode.cpp:585-592`):
   - 修复 `sysTypes` 类型不匹配问题（map<string,set<string>> → string）
   - 正确获取系统对应的观测类型

**分析结论**:
- BDS卫星类型区分对C1X结果影响小
- C2I观测值数量最多，但结果较差，怀疑存在异常历元
- 通过动态粗差探测阈值优化，可剔除误差较大的历元

**新增筛选机制**:

1. **sigma0 后验筛选** (`lib/SPPCode.cpp:810`):
   - 在解算完成后对 sigma0 进行判断
   - sigma0 > 10.0 时认为历元解算失败，直接剔除
   - **效果**: 2880个历元中删除约**300个**异常历元
   - **精度提升**: 误差从之前水平降低到 **13.53 m**

**后续优化方向**:
| 序号 | 优化方向 | 说明 | 优先级 |
|------|---------|------|--------|
| 1 | BDS GEO轨道模型修正 | GEO卫星需要特殊的轨道计算方式（不同于GPS广播星历公式） | **高** |
| 2 | 电离层模型升级 | 当前使用Klobuchar模型，可升级为BDGIM（北斗全球电离层模型） | **高** |
| 3 | TGD改正优化 | 确保正确读取和应用各系统的TGD参数 | **中** |

---

### 2026-06-02

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | 修改 TGD 输出格式为每个历元一行 | SPPCode | ✅ 完成 |
| 2 | 修复 CMakeLists.txt 引用不存在文件 | 构建系统 | ✅ 完成 |
| 3 | 更新 TGD 测试程序输出路径 | exam-5.4 | ✅ 完成 |
| 4 | BDS 观测值类型改为 C2（B1I） | SPPCode | ✅ 完成 |
| 5 | 修复 Galileo 星历 BGD 参数解析错误 | NavEphGalileo | ✅ 完成 |
| 6 | 修复 BDS GEO/IGSO/MEO 卫星类型判断错误 | SPPCode | ✅ 完成 |

**修复**:

1. **BDS 卫星类型判断修复** (`lib/SPPCode.cpp:855-868`):
   - **问题**: 原代码错误地将 BDS 卫星 PRN 分类为：MEO(C01-C16)、IGSO(C19-C30)、GEO(C31-C35)
   - **正确分类**: GEO(C01-C05)、IGSO(C06-C10, C19-C30)、MEO(C11-C16, C31-C63)
   - **影响**: 错误的分类导致 GEO 卫星被赋予较高权重，其轨道误差污染解算结果
   - **改进效果**: BDS SPP 定位精度从 18 m RMS 提升至 15 m RMS，U 方向误差明显下降

2. **TGD 输出格式调整** (`lib/SPPCode.cpp:830-835`):
   - 将原每个卫星一行的格式改为每个历元一行
   - 输出格式：`YDSTime SatID1=TGD1 SatID2=TGD2 ...`
   - 便于后续 Python 绘图分析

2. **CMakeLists.txt 文件引用修复** (`CMakeLists.txt:129`):
   - 原引用不存在文件 `examples/exam-6.4-spp_tgd.cpp`
   - 修改为实际存在的 `examples/exam-5.4-tgd_test.cpp`

3. **输出路径规范化** (`examples/exam-5.4-tgd_test.cpp:94`):
   - 按照项目规范将输出路径改为 `gnss_draw/data/spp_tgd/`
   - 自动创建子目录

4. **BDS 观测值类型修改** (`lib/SPPCode.cpp:241`):
   - 将 BDS 观测值类型从 C1（B1C）改为 C2（B1I）
   - 符合 BDS 单频定位工程实践

5. **Galileo BGD 参数解析修复** (`lib/RinexNavStore.cpp`, `lib/NavEphGalileo.hpp`):
   - 修复 RINEX 导航文件中 Galileo BGD 参数解析错误
   - 将原误读为 TGD/IODC 的字段修正为 BGD_E5aE1/BGD_E5bE1
   - 在 SPP 中正确应用 BGD 改正

---

### 2026-05-31

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | 修复 Galileo SPP 计算钟差错误 | NavEphGalileo | ✅ 完成 |
| 2 | SPP 输出改为 ENU 坐标 + XYZ | SPPCode | ✅ 完成 |
| 3 | 添加 PDOP/NSAT/残差统计输出 | SPPCode | ✅ 完成 |

**修复**:

1. **Galileo 钟差计算错误** (`lib/NavEphGalileo.cpp`):
   - **问题**: Galileo SPP 单点定位时误差达到几十万米
   - **问题表现**: `cttoc` 和 `cttoe` 之间差了约 1024 周，导致钟差计算严重错误
   
   **问题排查步骤**:
   1. **发现异常**: 运行 Galileo SPP 测试时，定位结果误差达几十万米，明显异常
   2. **初步定位**: 检查观测值、卫星位置计算，发现钟差参数异常偏大
   3. **时间分析**: 对比 `cttoc` 和 `cttoe` 的值，发现两者相差约 1024 周
   4. **时间系统对比**: 意识到 1024 周正好是 GPS 起始点（1980-01-06）和 Galileo 起始点（1999-08-22）的差距
   5. **时间转换检查**: 排查 `convertTimeSystem()` 函数，确认时间转换逻辑正确
   6. **星历读取定位**: 最终发现问题出在星历读取阶段，`cttoc` 被错误地以 GPS 周为参考计算
   
   - **根本原因**: 星历读取时，`cttoc` 使用 GPS 周秒格式计算，但 Galileo 时间系统的起始纪元与 GPS 不同（相差 1024 周）
   - **解决方案**: 在读取 Galileo 星历时，使用 `GALWeekSecond` 而非 `GPSWeekSecond` 创建周秒对象，正确计算 `cttoc` 和 `cttoe`
   - **代码修改**: 调整 `RinexNavStore.cpp` 中 Galileo 星历读取逻辑，使用 `TimeSystem::GAL` 创建周秒对象
   - **改进效果**: Galileo SPP 定位精度恢复正常，与其他系统一致

2. **SPP 输出增强** (`lib/SPPCode.cpp`):
   - 添加固定参考点 `(-2267750.275, 5009154.471, 3221294.345)` 用于 ENU 坐标转换
   - 使用 `ReferenceFrameFactory::create("G")` 创建 GPS 参考框架
   - 同时输出 XYZ 和 ENU 坐标
   - 添加文件头注释说明字段含义
   - 添加 PDOP、卫星数（NSAT）、平均残差、RMS残差、最大残差统计信息

**输出格式更新**:
```
# YDSTime X Y Z E N U PDOP NSAT Sigma0 MeanResidual RMSResidual MaxResidual
2025/001 00:00:00.000 -2267750.275 5009154.471 3221294.345 0.000 0.000 0.000 1.50 10 0.150 0.005 0.012 0.035
```

3. **粗差探测增强** (`lib/SPPCode.cpp`):
   - 添加100米固定阈值
   - 修改判定逻辑：残差 > 自适应阈值(5σ) **或** 残差 > 100米，满足任一条件即移除
   - 防止观测质量过好时阈值过小导致误删正常观测值

4. **PDOP 质量控制** (`lib/SPPCode.cpp`):
   - 在最小二乘求解后添加 PDOP 判定
   - PDOP > 10 时抛出异常跳过该历元
   - 避免几何条件差的历元影响解算结果

5. **BDS 卫星类型权重处理** (`lib/SPPCode.cpp`):
   - 添加 `getBDSSatType()` 函数：根据PRN号判断BDS卫星类型（MEO/IGSO/GEO）
   - 添加 `getTypeWeight()` 函数：为不同类型卫星分配权重
   - 权重策略：MEO(1.0)、IGSO(0.5)、GEO(0.25)
   - 最终权重 = 仰角权重 × 类型权重
   - 非BDS卫星不受影响（权重=1.0）

**BDS卫星PRN分类**:
| 类型 | PRN范围 | 权重 |
|------|---------|------|
| MEO | C01-C16 | 1.0 |
| IGSO | C19-C30 | 0.5 |
| GEO | C31-C35 | 0.25 |

**预期效果**:
- BDS定位精度改善：RMS从15m → 10-12m
- U方向误差明显下降
- 多系统融合时PDOP下降，解更稳定
- 保留BDS几何优势，避免IGSO/GEO误差污染

---

### 2026-05-26

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | 更新 CHANGELOG，添加第5章"观测方程与系统误差"待办清单 | 项目文档 | ✅ 完成 |

**第5章 观测方程与系统误差 —— 待办清单**

**【优先处理：大气传播系统误差】**

**5.4.2 大气传播系统误差**
| 任务 | 状态 | 说明 |
|------|------|------|
| Klobuchar 电离层模型 | ✅ 已完成 | `klobucharIonosphericCorrection()` |
| Saastamoinen 对流层模型 | ✅ 已完成 | `saastamoinenTroposphericCorrection()` |
| 电离层延迟 vs 时间数据 | ⏳ 待办 | 需生成数据 |
| 电离层延迟 vs 高度角数据 | ⏳ 待办 | 需生成数据 |
| 对流层延迟 vs 时间数据 | ⏳ 待办 | 需生成数据 |
| 对流层延迟 vs 高度角数据 | ⏳ 待办 | 需生成数据 |
| 改正前后误差对比图数据 | ⏳ 待办 | 需生成数据 |

---

**【基础准备：观测方程与几何模型】**

**5.1 非组合观测方程**
| 任务 | 状态 | 说明 |
|------|------|------|
| 整理伪距/载波观测方程 | ✅ 已完成 | `GnssFunc.cpp`, `SPPIFCode.cpp` |
| 绘制观测方程组成结构图 | ⏳ 待办 | 需生成数据 |
| 总结各误差项量级 | ⏳ 待办 | 需生成统计数据 |

**5.2 消电离层组合观测方程**
| 任务 | 状态 | 说明 |
|------|------|------|
| 完成 IF 组合公式推导 | ✅ 已完成 | `computeIF()` |
| 计算双频消电离层组合结果 | ✅ 已完成 | `computeIF()` |
| 电离层延迟时间序列图数据 | ⏳ 待办 | 需生成数据 |
| 改正前后伪距误差对比图数据 | ⏳ 待办 | 需生成数据 |
| L1/L2 电离层延迟对比图数据 | ⏳ 待办 | 需生成数据 |

**5.3 几何距离模型**
| 任务 | 状态 | 说明 |
|------|------|------|
| 卫星-接收机几何距离计算 | ✅ 已完成 | `linearize()` |
| 卫星天空图（Skyplot）数据 | ⏳ 待办 | 需生成数据 |
| DOP（PDOP/HDOP）时间变化图数据 | ⏳ 待办 | 需生成数据 |
| 几何关系示意图 | ⏳ 待办 | 需生成数据 |

---

**【卫星端与接收端误差】**

**5.4.1 卫星端系统误差**
| 任务 | 状态 | 说明 |
|------|------|------|
| 卫星钟差改正 | ✅ 已完成 | `computeAtTransmitTime()` |
| TGD 改正 | ✅ 已完成 | `correctTGD()` |
| 相对论效应公式整理 | ✅ 已完成 | `svRelativity()` |
| 精密星历轨道误差分析与绘图 | ✅ 已完成 | `OrbitExporter` |
| 卫星钟差时间序列图数据 | ⏳ 待办 | 需生成数据 |
| 轨道误差对比图数据 | ✅ 已完成 | `exportCombinedOrbitData()` |

**5.4.3 接收端系统误差**
| 任务 | 状态 | 说明 |
|------|------|------|
| 接收机钟差估计 | ✅ 已完成 | `Parameter::cdt` |
| 多路径与随机噪声影响分析 | ⏳ 待办 | 需分析 |
| 接收机钟差时间序列图数据 | ⏳ 待办 | 需生成数据 |
| 残差时间序列图数据 | ⏳ 待办 | 需生成数据 |
| 残差直方图（Histogram）数据 | ⏳ 待办 | 需生成数据 |

**5.4.4 其他系统误差**
| 任务 | 状态 | 说明 |
|------|------|------|
| 汇总未建模误差来源 | ⏳ 待办 | 需分析 |
| 制作误差预算统计表 | ⏳ 待办 | 需生成数据 |
| 误差贡献饼图数据（可选） | ⏳ 待办 | 需生成数据 |

---

**【程序设计与最终展示】**

**5.5 程序设计**
| 任务 | 状态 | 说明 |
|------|------|------|
| 整理程序整体架构 | ✅ 已完成 | 代码规范文档 |
| 绘制程序流程图 | ⏳ 待办 | 需生成文档 |
| 整理模块划分 | ✅ 已完成 | 代码规范文档 |

**最终结果展示**
| 任务 | 状态 | 说明 |
|------|------|------|
| 改正前后定位误差对比数据 | ⏳ 待办 | 需生成数据 |
| ENU误差散点图数据 | ⏳ 待办 | 需生成数据 |
| RMS/STD统计结果数据 | ⏳ 待办 | 需生成数据 |
| DOP 与定位精度关系分析数据 | ⏳ 待办 | 需生成数据 |
| 残差分析与误差来源总结 | ⏳ 待办 | 需生成报告 |

---

### 2026-05-25

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | 创建代码规范文档 `claude.md` | 项目文档 | ✅ 完成 |
| 2 | 更新代码规范文档，添加 CHANGELOG 和参考文档说明 | 项目文档 | ✅ 完成 |

**新增**:

1. **代码规范文档** (`gnssLab-2.4/claude.md`):
   - 创建完整的 gnssLab-2.4 代码规范文档
   - 包含项目概述、命名规范、代码风格、类设计、函数设计等章节
   - 定义支持的7种GNSS系统（GPS、BDS、GLONASS、Galileo、QZSS、IRNSS、SBAS）
   - 规范文件命名、类/函数/变量/常量命名规则
   - 制定代码风格标准（缩进、大括号、空格、行长度）
   - 定义注释规范和异常处理规范
   - 包含常用GNSS领域缩写对照表

2. **文档内容更新**:
   - 在项目结构中添加 `CHANGELOG.md`、`LICENSE`、`README.md` 文件说明
   - 添加 `doc/` 目录重要参考文档清单（ICD-GPS-200C、RINEX格式规范、SP3格式规范、OEM7手册等）

---

### 2026-05-24

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | 广播星历验证与问题排查 | RinexNavStore | ✅ 完成 |
| 2 | 添加 svURA 输出功能 | OrbitExporter | ✅ 完成 |

**广播星历验证阶段问题总结**:

#### 1. 初始问题：广播星历与精密星历存在明显系统偏差
在完成广播星历读取与卫星位置计算后，将广播星历计算结果与 SP3 精密星历进行对比，发现：

| 系统 | 误差特征 |
|------|----------|
| GPS (G) | 误差最大，可达约 10~11 m |
| GLONASS (R) | 误差表现为较明显的正弦变化 |
| Galileo (E) | 表现最好，大多数卫星误差在 1 m 以内 |
| BDS/QZSS/IRNSS | 一般在 3~5 m 量级 |

**说明**: 星历读取并未完全错误，但某些时间参数、坐标参考或广播模型实现存在偏差。

#### 2. 首先怀疑：Toe / Toc 时间处理错误
重点检查了 `Toe`、`Toc`、`ctToe`、`ctToc` 之间的关系。发现原代码中：
```cpp
gpsEph.ctToe = CivilTime2CommonTime(cvt);
```
这里实际上读取的是 RINEX 第一行 epoch（即 Toc），但错误赋值给了 `ctToe`，属于语义错误。

#### 3. Toe 与 ctToe 的本质区别
| 变量 | 含义 |
|------|------|
| `Toe` | 星历参考时刻（周内秒） |
| `ctToe` | Toe 对应的绝对 CommonTime |
| `Toc` | 钟差参考时刻（周内秒） |
| `ctToc` | Toc 对应的绝对 CommonTime |

原代码实际上执行了 `ctToe ← Toc`，导致 `tk = t - ctToe` 使用了错误参考时间。

#### 4. 修复 Toe 时间构造
修改为正确的构造方式：
```cpp
GPSWeekSecond ws(gpsEph.GPSWeek, gpsEph.Toe, TimeSystem::GPS);
WeekSecond2CommonTime(ws, gpsEph.ctToe);
```
即 `ctToe = GPSWeek + Toe`，而不是 `ctToe = Toc`。

#### 5. 修改后结果：误差几乎没有变化
这是一个非常关键的现象，说明：
- 原系统并不是完全依赖错误 ctToe 工作
- 或者 Toe/Toc 在广播星历中本身接近
- 或者误差来源并不主要来自 Toe

#### 6. 检查星历选择逻辑（findGPSEph）
发现广播星历选择逻辑存在问题：
- **原逻辑**: 选择与目标时刻最近的星历
- **问题**: 没有限制星历是否在目标时刻之前、是否已经"生效"
- **结果**: 使用未来星历或跨 fit interval 使用星历

**修复方案**: 仅允许选择 `Toe <= epoch` 的星历，即只选过去最近星历。

#### 7. 发现 GPS 只有 G01 明显异常
| 卫星 | 误差情况 |
|------|----------|
| G01 | 约 10~11 m |
| G02 等 | 约 3~5 m |

这说明系统整体实现大概率已经基本正确，否则所有 GPS 卫星都会同时出现大误差。问题开始转向单颗卫星广播星历质量。

#### 8. 检查 URA 后确认：广播星历质量确实下降
输出 `eph.svURA(epoch)` 结果：
- 前半天：URA = 2 m
- 后半天：URA = 2.8 m  
- 最后阶段：URA = 4 m

说明广播星历自身精度在下降，因此广播与精密星历出现数米级偏差属于正常现象。

#### 9. 当前最可能结论
代码主体已经基本正确，包括：
- ✅ 开普勒求解
- ✅ 轨道改正
- ✅ 地球自转改正
- ✅ 坐标转换
- ✅ Toe/Toc 处理
- ✅ 星历选择

都没有明显致命错误。

#### 10. 当前剩余误差来源（按可能性排序）
1. **广播星历本身误差** - GPS 广播星历本来只有米级到十米级精度，URA 已明确显示精度下降
2. **SP3 与广播星历参考点不同** - SP3 通常提供卫星质心（COM），广播星历有时对应 APC（天线相位中心），会带来数米级系统偏差且具有周期性变化
3. **G01 特殊卫星状态** - 可能轨道机动、广播质量差、fit interval 较长、轨道外推误差大

#### 11. 补充分析：其余卫星系统误差来源
进一步分析发现，除 G01 外的其余卫星也存在不同程度的系统误差（3~5 m 量级），推测主要原因是：
- **卫星天线相位中心（APC）与质心（COM）偏差**：广播星历计算的是卫星天线相位中心位置，而 SP3 精密星历提供的是卫星质心位置
- **不同卫星的 APC 偏差不同**：各卫星的天线安装位置和相位中心偏移参数存在差异
- **周期性变化特征**：误差呈现与轨道周期相关的周期性变化，符合 APC 偏差的特征

#### 12. 当前整体结论
| 项目 | 状态 |
|------|------|
| 广播星历读取 | ✅ 基本正确 |
| GPS/Galileo/BDS/QZSS 轨道计算流程 | ✅ 基本正确 |
| Toe/Toc 处理 | ✅ 已修正 |
| 星历选择逻辑 | ✅ 已修正 |
| 系统误差主要来源 | ✅ 广播星历自身精度与参考点差异 |

**排除的错误来源**:
- ❌ 时间系统炸裂（否则误差会达到 km 级）
- ❌ 坐标系错误
- ❌ 周跳错误
- ❌ 开普勒求解错误
- ❌ 地球自转漏改正

**新增**:

1. **OrbitExporter 添加 svURA 输出** (`lib/OrbitExporter.hpp/cpp`):
   - 在 `OrbitPoint` 结构体中添加 `svURA` 字段
   - 在 `computeOrbitPoints()` 中通过 `navStore.findEph()` 获取星历对象，调用 `eph->svURA(currentTime)` 获取卫星用户距离精度
   - 在 `exportOrbitData()` 输出的 `orbit_data.txt` 中添加 `svURA(m)` 字段

---

## 项目概述

**项目名称**: gnssLab  
**版本**: 1.2  
**作者**: Shoujian Zhang, Wuhan University  
**用途**: GNSS算法实验平台

---

## 时间线变更记录

### 2026-05-23

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | 添加 QZSS 系统支持 | NavEphQZSS | ✅ 完成 |
| 2 | 添加 IRNSS 系统支持 | NavEphIRNSS | ✅ 完成 |
| 3 | 添加 IRNSS 自洽验证功能 | exam-3.2 | ✅ 完成 |
| 4 | 创建 OrbitExporter 轨道数据导出模块 | OrbitExporter | ✅ 完成 |
| 5 | 修复时间计算方式（使用 CommonTime operator） | OrbitExporter | ✅ 完成 |

**新增**:

1. **QZSS 系统支持** (`lib/NavEphQZSS.hpp/cpp`):
   - 创建 `NavEphQZSS` 类，继承自 `NavEphBase`
   - 实现完整的 QZSS 广播星历数据结构
   - 实现 `svXvt()`、`svClockBias()`、`svRelativity()` 等方法
   - 使用 GPS 时间系统（QZS），与 GPS 相同的开普勒轨道解析
   - 更新 `NavEphRegistry.cpp` 注册 QZSS 工厂函数（系统码 'J'）
   - 更新 `RinexNavStore.hpp/cpp` 添加 QZSS 星历加载和查找方法

2. **IRNSS 系统支持** (`lib/NavEphIRNSS.hpp/cpp`):
   - 创建 `NavEphIRNSS` 类，继承自 `NavEphBase`
   - 实现完整的 IRNSS 广播星历数据结构
   - 实现 `svXvt()`、`svClockBias()`、`svRelativity()` 等方法
   - 使用 IRNSS 时间系统（IRN），支持 GEO/IGSO 轨道
   - 更新 `NavEphRegistry.cpp` 注册 IRNSS 工厂函数（系统码 'I'）
   - 更新 `RinexNavStore.hpp/cpp` 添加 IRNSS 星历加载和查找方法

3. **IRNSS 自洽验证功能** (`examples/exam-3.2-read_rinex_data.cpp`):
   - 针对 IRNSS 无 SP3 精密星历的情况，实现广播星历自洽验证
   - 在 toe 前后各 30 分钟，以 300 秒间隔计算卫星位置
   - 检查轨道连续性（相邻历元位置变化）
   - 检查轨道半径合理性（GEO/IGSO 约 42164 km）
   - 检查速度量级（约 2-4 km/s）
   - 检查 toe 前后对称性（检测 tk 周跳问题）
   - 输出验证报告到 `irnss_self_consistency_check.txt`

4. **OrbitExporter 轨道数据导出模块** (`lib/OrbitExporter.hpp/cpp`):
   - 创建 `OrbitPoint` 结构体，存储卫星轨道点数据（时间、卫星ID、X/Y/Z坐标、半径、经纬度）
   - 创建 `OrbitDataMap` 类型，用于管理多卫星轨道数据
   - 实现 `computeOrbitPoints()` 方法：基于广播星历计算卫星24小时轨道（每300秒一个点）
   - 实现 `exportOrbitData()` 方法：导出所有卫星综合轨道数据
   - 实现 `exportSingleSatOrbit()` 方法：导出单颗卫星轨道数据
   - 实现 `exportRadiusAnalysis()` 方法：导出轨道半径随时间变化分析数据
   - 实现 `exportVelocityAnalysis()` 方法：导出速度变化分析数据
   - 实现 `exportGroundTrack()` 方法：导出地面轨迹数据（经纬度）
   - 数据输出路径：`~/Documents/GNSS_Lab/gnss_draw/data/orbit/`
   - 更新 `examples/exam-3.2-read_rinex_data.cpp`：添加星历验证实验流程，处理 G01、E01、C01、C06、I01 五颗典型卫星

**修复**:

1. **构造函数初始化错误修复** (`lib/NavEphQZSS.hpp`, `lib/NavEphIRNSS.hpp`):
   - **问题**: 构造函数错误地在初始化列表中初始化父类成员 `beginValid` 和 `endValid`
   - **解决方案**: 改为在构造函数体内赋值

2. **时间计算方式修复** (`lib/OrbitExporter.cpp`) ⚠️ **重要修复**:
   - **问题**: 原代码使用手动计算方式设置时间（`currentTime.set(currentTime.m_day, currentTime.m_sod + offset)`），直接操作 `m_sod` 字段进行秒数累加
   - **风险**: ⚠️ **严重缺陷** - 手动计算无法正确处理跳秒（leap second）和跨天边界情况
   - **具体影响**: 
     - 午夜时分：`m_sod + offset` 可能超过 86400，导致日期未正确递增
     - 闰秒时刻：无法处理 UTC 跳秒（±1秒），导致时间不连续
     - 轨道计算：在边界时刻产生位置跳跃或速度异常
   - **解决方案**: ✅ **推荐实践** - 使用 `CommonTime` 类内置的重载运算符
   - **修改内容**:
     - `computeOrbitPoints()`: 将 `currentTime.set(currentTime.m_day, currentTime.m_sod + offset)` 修改为 `currentTime = startTime + i * intervalSeconds`
     - `exportVelocityAnalysis()`: 将硬编码时间间隔 `300.0` 修改为 `double dt = curr.time - prev.time` 动态计算实际时间差
   - **改进效果**: 
     - ✅ 自动处理跨天日期递增
     - ✅ 自动处理闰秒（依赖 `CommonTime` 内部实现）
     - ✅ 速度计算更加精确（使用实际时间差而非假设的固定间隔）
     - ✅ 提高轨道计算在边界时刻的准确性和鲁棒性

3. **OrbitExporter 数值鲁棒性修复** (`lib/OrbitExporter.cpp`) ⚠️ **重要修复**:
   - **问题1**: 使用 `double` 作为 map key 导致浮点误差匹配失败
   - **解决方案**: 改为使用 `long long`（毫秒时间戳）作为键
   - **代码**: `inline long long timeToMillis(const CommonTime& time)`
   
   - **问题2**: `computeLatLon()` 使用球坐标而非大地坐标
   - **解决方案**: 使用 `xyz2blh(xyz, wgs84)` 进行 WGS84 大地坐标转换
   - **改进效果**: GEO/IGSO 地面轨迹计算准确，高纬度误差消除
   
   - **问题3**: SP3 单位不确定（可能是 km 或 m）
   - **解决方案**: 通过 `pos.norm()` 判断单位，自动乘以适当的缩放因子
   - **代码**: `if (posNorm < 50000.0) scaleFactor = 1000.0;`
   
   - **问题4**: 缺少 SP3 coverage check
   - **解决方案**: 使用 `sp3Store.getSatSet()` 提前检查卫星是否在 SP3 文件中
   - **改进效果**: 避免不必要的异常捕获，代码更优雅
   
   - **问题5**: IRNSS 卫星跳过逻辑不合理
   - **问题**: 原代码 `if (sat.system == "I") continue;` 导致 IRNSS 不生成文件
   - **解决方案**: 修改为条件性获取 SP3 数据，但始终调用 `exportCombinedSingleSatOrbit()`
   - **改进效果**: IRNSS 卫星正常生成文件（精密星历列为 NaN）
   
   - **问题6**: 速度计算可能除零
   - **解决方案**: 添加 `if (fabs(dt) < 1e-6) continue;` 检查
   - **改进效果**: 避免时间重复或错乱导致的除零错误
   
   - **问题7**: 注释不准确
   - **修复**: 将 `SP3 precise ephemeris is interpolated` 改为 `SP3 precise ephemeris sampled at broadcast epochs`
   
   - **整体改进效果**:
     - ✅ 消除浮点时间精度问题
     - ✅ 正确处理大地坐标（WGS84）
     - ✅ 自动检测 SP3 单位系统
     - ✅ 完善卫星覆盖检测
     - ✅ IRNSS 卫星正常处理
     - ✅ 避免除零错误
     - ✅ 提高代码整体鲁棒性

**更新**:

1. **CMakeLists.txt**:
   - 添加 `lib/NavEphQZSS.hpp`、`lib/NavEphQZSS.cpp`、`lib/NavEphIRNSS.hpp`、`lib/NavEphIRNSS.cpp` 到编译配置
   - 添加 `lib/OrbitExporter.hpp`、`lib/OrbitExporter.cpp` 到编译配置

**技术细节**:

| 系统 | 轨道计算方法 | 时间系统 | 参考框架 |
|------|-------------|----------|----------|
| QZSS | 开普勒轨道解析 | QZS（与GPS相同） | WGS84 |
| IRNSS | 开普勒轨道解析 | IRN | WGS84 |

**当前支持的 GNSS 系统**:
- GPS (G) ✅
- BDS (C) ✅
- GLONASS (R) ✅
- Galileo (E) ✅
- **QZSS (J)** ✅ 新增
- **IRNSS (I)** ✅ 新增

---

### 2026-05-20

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | 利用SP3精密星历完成广播星历调试 | 星历模块 | ✅ 完成 |
| 2 | 纠正GLONASS轨道计算错误 | NavEphGLONASS | ✅ 完成 |
| 3 | 添加Galileo系统支持 | NavEphGalileo | ✅ 完成 |

**修复**:

1. **GLONASS轨道计算方法修正** (`lib/NavEphGLONASS.cpp:85`):
   - **问题**: 之前错误地使用了与GPS相同的开普勒轨道解析计算方法
   - **影响**: 导致GLONASS卫星位置计算误差过大
   - **解决方案**: 改为使用RK4（四阶龙格-库塔）数值积分方法
   - **技术实现**:
     - GLONASS广播星历提供卫星在参考时刻的状态向量（位置X,Y,Z、速度Vx,Vy,Vz、加速度Ax,Ay,Az）
     - 使用RK4数值积分进行轨道外推
     - 考虑的力模型包括：中心引力、J2摄动、地球自转影响
     - 使用PZ-90参考框架参数（不同于GPS的WGS84）
   - **改进效果**: 计算精度显著提高，与SP3精密星历的一致性验证通过

2. **SP3精密星历辅助调试** (`lib/SP3Store.cpp`):
   - 使用SP3精密星历作为参考标准，验证广播星历计算结果的准确性
   - 完成GPS和GLONASS广播星历与SP3星历的对比验证

**新增**:

3. **Galileo系统支持** (`lib/NavEphGalileo.hpp/cpp`):
   - 创建 `NavEphGalileo` 类，继承自 `NavEphBase`
   - 实现完整的Galileo广播星历数据结构（包含SISA等Galileo特有参数）
   - 实现 `svXvt()` 方法，使用开普勒轨道解析计算卫星位置和速度
   - 使用 `Galileo` 参考框架（与GPS相同的WGS84参数）
   - 更新 `RinexNavStore` 添加Galileo星历加载和查找方法
   - 在 `NavEphRegistry` 中注册Galileo工厂函数

4. **writeFile方法重构** (`lib/RinexNavStore.cpp:1200`):
   - 提取公共的 `writeContrastData()` 辅助函数，消除代码重复
   - 使用 `std::map` 实现系统名称到数据集的映射，支持工厂模式思想
   - 同时支持系统代码（G/C/R/E）和系统全称（GPS/BDS/GLONASS/Galileo）
   - 添加GLONASS和Galileo对比数据集支持

**修复**:

3. **Galileo编译错误修复** (`CMakeLists.txt:55`):
   - **问题**: `undefined reference to vtable for NavEphGalileo` 链接错误
   - **原因**: `NavEphGalileo.cpp` 文件未添加到CMake编译配置中
   - **解决方案**: 在 `CMakeLists.txt` 中添加 `lib/NavEphGalileo.hpp` 和 `lib/NavEphGalileo.cpp`

**技术细节**:

| 系统 | 轨道计算方法 | 星历参数类型 | 参考框架 |
|------|-------------|-------------|----------|
| GPS | 开普勒轨道解析 | 轨道根数（半长轴、偏心率等） | WGS84 |
| BDS | 开普勒轨道解析 | 轨道根数（半长轴、偏心率等） | CGCS2000 |
| GLONASS | RK4数值积分 | 状态向量（位置、速度、加速度） | PZ90 |
| Galileo | 开普勒轨道解析 | 轨道根数（半长轴、偏心率等） | GTRF/WGS84 |

**测试结果**:
- GLONASS广播星历轨道计算结果与SP3精密星历对比验证通过
- 位置精度达到预期要求

---

### 2026-05-19

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | 添加 `loadFile()` 方法 | RinexObsReader | ✅ 完成 |
| 2 | 添加析构函数和移动语义 | RinexObsReader | ✅ 完成 |
| 3 | 增强异常错误上下文（行号追踪） | RinexObsReader | ✅ 完成 |
| 4 | `static_Obs()` 添加数据有效性检查 | RinexObsReader | ✅ 完成 |
| 5 | `loadFile()` 返回 `bool` | RinexNavStore | ✅ 完成 |
| 6 | 添加 `hasEphData()` 方法 | RinexNavStore | ✅ 完成 |
| 7 | 添加 `getSystems()` 方法 | RinexNavStore | ✅ 完成 |
| 8 | 优化 `ObsDataStaticSum` 输出格式 | GnssStruct | ✅ 完成 |
| 9 | `ObsDataStaticSum` 添加多系统统计方法 | GnssStruct | ✅ 完成 |
| 10 | `codeSelectFrequency()` 异常处理 | GnssFunc | ✅ 完成 |
| 11 | `writefileSatPos()` 路径参数化 | GnssFunc | ✅ 完成 |
| 12 | 添加系统过滤配置 | RinexObsReader | ✅ 完成 |
| 13 | 修复 `safeStoi` 返回值歧义 | StringUtils | ✅ 完成 |
14 | **修复星历查找异常处理** | RinexNavStore | ✅ 完成 |
| 15 | **修复GLONASS数据读取解析** | RinexNavStore | ✅ 完成 |

**新增功能**:
- 在 `lib/StringUtils.h` 中添加 `safeStoiOpt()` 函数，使用 `std::optional<int>` 返回值，区分无效输入和实际值

**修复**:
1. **星历查找异常处理** (`lib/RinexNavStore.cpp`):
   - 在 `findGPSEph()`、`findBDSEph()`、`findGLOEph()` 函数中添加卫星存在性检查
   - 当卫星不存在时抛出 `InvalidRequest` 异常，确保 `try-catch` 能正确捕获
   - 解决了查询不存在卫星时程序崩溃的问题

2. **GLONASS数据读取解析** (`lib/RinexNavStore.cpp`):
   - 修复 `loadGLOEph()` 函数中 GLONASS 导航电文读取错误
   - RINEX 3.x 格式中 GLONASS 每行有4个字段（每个19字符），之前只读取了前3个
   - 添加 `n += 19` 跳过第4个字段，确保后续数据正确对齐

**更新**:
- 更新 `examples/exam-3.2-read_rinex_data.cpp`，添加新功能测试用例和异常处理测试
- 更新测试时间为导航文件对应日期（2022-06-20），确保星历查询结果准确

**测试结果**:
- 所有测试用例均已通过
- GLONASS 卫星位置计算结果正确（约26,000-43,000 km）
- 不存在卫星查询正确抛出 `InvalidRequest` 异常

**下一步计划（多卫星系统支持完善）**:

| 序号 | 任务 | 模块 | 优先级 | 状态 |
|------|------|------|--------|------|
| 1 | 添加 GLONASS 星历数据结构和加载方法 | RinexNavStore | P2 | ✅ 完成 |
| 2 | **重构：统一星历接口架构（工厂模式）** | NavEphBase | **P1** | 🔄 进行中 |
| 3 | 添加 Galileo 星历数据结构和加载方法 | RinexNavStore | P2 | ⏳ 待办 |
| 4 | 更新 `getXvt()` 方法支持多系统 | RinexNavStore | P2 | ✅ 完成 |
| 5 | 添加 QZSS 星历支持 | RinexNavStore | P3 | ⏳ 待办 |
| 6 | 添加 IRNSS 星历支持 | RinexNavStore | P3 | ⏳ 待办 |
| 7 | 添加 SBAS 星历支持 | RinexNavStore | P3 | ⏳ 待办 |
| 8 | 添加 `getSystems()` 方法返回已加载系统列表 | RinexNavStore | P3 | ✅ 完成 |
| 9 | 更新 exam-3.2 测试多系统星历加载 | 测试代码 | P3 | ✅ 完成 |

**已完成的多系统支持改进**:
- 创建了 `lib/NavEphGLONASS.hpp` - GLONASS 星历数据结构
- 创建了 `lib/NavEphGLONASS.cpp` - GLONASS 星历实现（包含位置速度计算）
- 更新了 `RinexNavStore.hpp` - 添加 GLONASS 相关声明
- 更新了 `RinexNavStore.cpp` - 添加 GLONASS 星历加载和查找方法
- 更新了 `getXvt()` - 支持 GLONASS 卫星位置计算
- 更新了 `hasEphData()` - 支持 GLONASS 数据检查
- 更新了 `CMakeLists.txt` - 添加 GLONASS 文件到构建系统

**统一星历接口架构重构（进行中）**:
- 创建了 `lib/NavEphBase.hpp` - 统一星历接口基类
- 更新了 `NavEphGPS.hpp` - 继承自 `NavEphBase`，实现接口方法
- 更新了 `NavEphBDS.hpp` - 继承自 `NavEphBase`，实现接口方法
- 更新了 `NavEphGLONASS.hpp` - 继承自 `NavEphBase`，实现接口方法
- 创建了 `lib/NavEphBase.cpp` - 工厂类实现

---

### 2026-05-18

**新增**:
- 在 `lib/GnssStruct.h` 的 `SatID` 结构体中添加 `generation` 字段：
  - 支持区分北斗2代（BDS-2，C01-C14）和北斗3代（BDS-3，C19+）卫星
  - 构造函数自动根据卫星ID判断世代
  - 非北斗系统世代值为0
**改进**:
- 增强 `SatID` 字符串构造函数的输入验证：
  - 验证字符串长度（至少3个字符）
  - 验证系统标识有效性（仅允许 G/C/R/E/J/I/S）
  - 验证卫星ID为有效数字
  - 验证卫星ID范围（1-64）
- 修复 `lib/GnssStruct.cpp` 中 `Parameter::paraNameStrings` 数组：
  - 添加缺失的 `"cdt_BDS"` 和 `"bias"` 元素
  - 现在与 `ParameterName` 枚举完全对应（10个元素）
- 修复 `lib/GnssStruct.h` 和 `lib/GnssStruct.cpp` 中 `Variable` 类：
  - `operator==` 和 `operator!=` 声明为 `const` 成员函数
- 为 `SatID` 添加赋值运算符 `operator=`：
  - 正确复制 system、id 和 generation 字段
  - 包含自赋值检查

**更新**:
- 更新 `examples/exam-3.1-satid.cpp` 测试代码：
  - 添加异常处理测试（无效格式、无效系统、无效数字、越界ID）
  - 添加北斗世代字段测试（BDS-2、BDS-3、预留、非北斗）
  - 添加赋值运算符测试
  - 添加整数构造函数测试

**测试结果**:
- 所有测试用例均已通过

---

### 2026-05-17

**新增**:
- 在 `lib/CoordConvert.h` 中添加弧度/角度转换函数：
  - `rad2deg(double rad)`: 将弧度转换为角度
  - `deg2rad(double deg)`: 将角度转换为弧度
  - 定义常量 `RAD_TO_DEG` 和 `DEG_TO_RAD`
- 在 `lib/CoordStruct.h` 中扩展多GNSS系统参考框架支持：
  - 添加 `Galileo` 类（继承 `GPSEllipsoid`）：使用与GPS相同的参数
  - 添加 `IRNSS` 类（继承 `WGS84`）：使用标准WGS84参数
  - QZSS可直接使用 `GPSEllipsoid`（与GPS参数相同）
- 在 `lib/CoordStruct.h` 中实现参考框架工厂模式：
  - 添加 `ReferenceFrameFactory` 类，支持动态创建不同系统的参考框架对象
  - `create(const std::string& system)`: 根据系统名称创建对象
  - `getSupportedSystems()`: 获取支持的系统列表
  - `isSupported(const std::string& system)`: 检查系统是否支持
  - 支持大小写不敏感（如 "GPS" 和 "gps" 均可）
  - 未知系统自动返回默认WGS84
- 更新 `examples/exam-2.3-myself.cpp`：
  - 添加完整的多系统参考框架测试
  - 验证XYZ↔BLH转换可逆性
  - 测试ENU坐标转换功能
  - 添加工厂模式测试（动态创建、大小写不敏感、容错处理）

**技术讨论**:

**参考框架 vs 椭球的区别**:
- **椭球 (Ellipsoid)**: 纯几何模型，包含长半轴(a)、扁率(f)等参数，仅用于大地坐标与直角坐标转换
- **参考框架 (Reference Frame)**: 完整的坐标系定义，除椭球参数外，还包含地球物理参数（自转角速度ω、引力常数GM）、定向参数、原点定义等
- 当前实现中，定向参数（极移、GMST等）未包含，因为主要用于同一框架内的坐标转换，不同框架间的七参数转换需要定向参数

**各系统参考框架特性**:
| 系统 | 参考框架 | 椭球 | 长半轴 | GM值 |
|------|----------|------|--------|------|
| GPS | WGS84/G873 | WGS84 | 6378137m | 3.986005e14 |
| BDS | CGCS2000 | GRS80/WGS84 | 6378137m | 3.986004418e14 |
| GLONASS | PZ90-11 | PZ90 | 6378136m | 3.9860044e14 |
| Galileo | GTRF | WGS84 | 6378137m | 3.986005e14 |
| QZSS | JGD2000/WGS84 | WGS84 | 6378137m | 3.986005e14 |
| IRNSS | ITRF/WGS84 | WGS84 | 6378137m | 3.986004418e14 |

**测试结果**:
- 所有七种参考框架的坐标转换测试均已通过
- XYZ↔BLH转换可逆性验证通过（差异为0）
- GLONASS的PZ90椭球因长半轴小1米，相同XYZ坐标转换后高度相差约1米

**修复**:
- 修复 `CMakeLists.txt` 中 `coord_myself` 目标的错误配置（移除了错误链接的源文件）

**下一步计划**:
1. **P2**: 添加坐标验证功能（输入参数有效性检查）
2. **P3**: 实现七参数框架转换（布尔莎模型）
3. **P3**: 添加投影坐标支持（UTM、高斯-克吕格）
4. **P3**: 性能优化（缓存计算结果，减少重复计算）
5. **P3**: 配置化支持（从配置文件读取椭球参数）

---

### 2026-05-14

**新增**:
- 扩展 `WeekSecond` 类支持多GNSS系统：
  - 在 `lib/Const.h` 中添加 Galileo、GLONASS、QZSS、IRNSS 的纪元常量
  - 在 `lib/TimeStruct.h` 中添加 `IRN`（IRNSS）时间系统枚举
  - 添加四个新的派生类：`GALWeekSecond`、`GLOWeekSecond`、`QZSWeekSecond`、`IRNWeekSecond`
  - 声明工厂函数 `createWeekSecond()` 用于统一创建不同系统的时间对象
  - 在 `lib/TimeStruct.cpp` 中实现工厂函数，支持GPS、BDS、Galileo、GLONASS、QZSS、IRNSS六种系统
- 更新 `examples/exam-2.2-time_convert.cpp`：
  - 添加GPS周秒转换测试
  - 添加BDS周秒转换测试
  - 添加Galileo周秒转换测试
  - 添加GLONASS周秒转换测试（特殊处理，GLONASS无周数概念）
  - 添加QZSS周秒转换测试
  - 添加IRNSS周秒转换测试

**技术细节**:
- **GPS**: 纪元MJD 44244（1980-01-06），10位周数（0-1023）
- **BDS**: 纪元MJD 53736（2006-01-01），13位周数（0-8191）
- **Galileo**: 纪元MJD 51300（1999-08-22），13位周数（0-8191）
- **GLONASS**: 纪元MJD 37300（1996-01-01），无周数概念（Nbits=0）
- **QZSS**: 纪元MJD 44244（与GPS相同），10位周数（0-1023）
- **IRNSS**: 纪元MJD 51300（1999-08-22），10位周数（0-1023）

**测试结果**:
- 所有六种GNSS系统的工厂函数测试均已通过
- 时间转换精度验证通过，转换可逆性验证通过
- 多系统时间转换测试验证（以GPS时间43200秒为基准）：
  - BDS: 43186秒（BDT = GPS - 14s，验证通过）
  - Galileo: 43200秒（与GPS相同，验证通过）
  - GLONASS: 43219秒（GLO = GPS + 19s，验证通过）
  - QZSS: 43200秒（与GPS相同，验证通过）
  - IRNSS: 43200秒（与GPS相同，验证通过）

**时间系统转换原理**:
- GPS = TAI - 19s
- BDS = TAI - 33s → BDS = GPS - 14s
- Galileo/QZSS/IRNSS = TAI - 19s → 与GPS相同
- GLONASS = TAI（简化实现）→ GLO = GPS + 19s

---

### 2026-05-13

**新增**:
- 创建项目日志文件 `CHANGELOG.md`

**重构**:
- 重构 `examples/exam-1.1-parse_opt.cpp`：
  - 将全局变量替换为 `CalculatorConfig` 结构体封装
  - 使用 `getValueAsDouble()` 替代 `getValueAsInt()` 读取浮点数
  - 添加 `validate()` 方法验证配置有效性
  - 引入 `DEBUG_MODE` 宏控制调试输出
  - 添加完善的异常处理和错误提示
  - 优化帮助信息格式
- 创建 `examples/config.ini` 示例配置文件
- 重构 `examples/exam-1.2-parse_config.cpp`：
  - 使用 `GNSSConfig` 结构体封装所有配置参数
  - 添加 `validate()` 方法验证配置有效性
  - 添加 `print()` 方法格式化显示配置信息
  - 支持读取 int、string、double 多种类型配置值
  - 引入 `DEBUG_MODE` 宏控制调试输出
  - 添加完善的异常处理和帮助信息
- 创建 `examples/gnss_config.ini` 示例配置文件
- 重构 `examples/exam-2.1-eigen.cpp`（Eigen 矩阵库入门演示）：
  - 展示矩阵定义与初始化（动态矩阵、静态矩阵、特殊矩阵）
  - 演示矩阵基本运算（加法、减法、乘法、数乘）
  - 展示矩阵常用方法（转置、行列式、求逆、迹）
  - 演示向量运算（点积、范数、标准化）
  - 展示线性方程组求解（QR分解）
  - 演示矩阵块操作（提取/修改子矩阵）
  - 添加 `DEBUG_MODE` 宏控制调试输出
  - 添加辅助函数 `printMatrixInfo()` 和 `printVectorInfo()`
- 重构 `examples/exam-2.2-time_convert.cpp`（时间格式转换演示）：
  - 添加必要头文件（`iostream`, `iomanip`, `Exception.h`）
  - 添加 `DEBUG_MODE` 宏和异常处理机制
  - 优化输出格式，添加清晰的标题和分隔
  - 扩展演示内容至11种时间转换类型：
    - YMD↔JD、CivilTime↔CommonTime、CommonTime↔JulianDate
    - GPS↔UTC时间系统转换、BDT周秒格式转换
    - HMS↔SOD、YDSTime↔CommonTime、MJD↔CommonTime
    - CommonTime2020系列转换
- 重构 `lib/TimeConvert.cpp`：
  - 优化跳秒数据存储：将每次调用重建的 `map` 改为静态 `vector`
  - 添加 `LeapSecondData` 结构体和 `getLeapSecondData()` 辅助函数
  - 添加 `convertToTAI()` 和 `convertFromTAI()` 分离转换逻辑
  - 重构 `convertTimeSystem()`：采用"TAI作为中间系统"的转换策略
- 修复 `lib/TimeStruct.h`：
  - 修复 `BDTWeekSecond` 类末尾的注释错误

**问题记录**:
- **相对路径问题**：运行 `./parse_opt config.ini` 或 `./parse_config gnss_config.ini` 时，程序在当前工作目录查找配置文件，若配置文件不在当前目录则报错 `Unable to open file: xxx.ini`
  - **解决方案**：使用绝对路径或切换到配置文件所在目录运行
- **编译问题**：修改源代码后需要重新编译才能生效（运行 `make clean && make <target>`）

---

### 2026-05-08

**新增**:
- 创建 `examples/zuiyouguji/` 目录
- 创建 `examples/zuiyouguji/test.cpp` 主程序
- 创建 `lib/zuiyouguji/` 目录及基础框架

**修改**:
- 更新 `CMakeLists.txt`，添加 `coord_myself` 可执行程序

---

### 2020年（原始版本）

**初始创建**:
- 项目框架搭建
- 基础GNSS数据处理模块
- SPP单点定位实现
- RINEX文件读取
- 时间/坐标转换工具
- Lambda模糊度固定算法
- 卡尔曼滤波求解器

---

## 项目结构

```
gnssLab-2.4/
├── CMakeLists.txt
├── LICENSE
├── README.md
├── CHANGELOG.md
├── cmake-build-debug/
├── lib/                    # 核心库
│   ├── GnssFunc.cpp/h
│   ├── GnssStruct.cpp/h
│   ├── SPPCode.cpp/h
│   ├── SPPGFCode.cpp/h
│   ├── SPPIFCode.cpp/h
│   ├── SolverLSQ.cpp/h
│   ├── SolverKalman.cpp/h
│   ├── TimeConvert.cpp/h
│   ├── RinexObsReader.cpp/h
│   ├── RinexNavStore.cpp/h
│   ├── SP3Store.cpp/h
│   ├── CSDetector.cpp/h
│   ├── ARLambda.cpp/hpp
│   └── zuiyouguji/
│       ├── gnssfunc.cpp
│       └── gnssfunc.h
├── examples/               # 示例程序
│   ├── exam-*.cpp          # 第1-8章示例
│   ├── zuiyouguji/
│   │   ├── test.cpp
│   │   ├── CUSV_2026001_1.txt
│   │   └── result/
│   └── spp.ini
└── thirdparty/
    └── eigen-3.4.0/
```

---

## 功能模块清单

| 模块 | 文件 | 功能 |
|------|------|------|
| 基础工具 | TimeConvert, CoordConvert | 时间/坐标转换 |
| 数据结构 | GnssStruct, CoordStruct | 数据结构定义 |
| 数据读取 | RinexObsReader, RinexNavStore, SP3Store | 文件读取 |
| 定位算法 | SPPCode, SPPIFCode, SPPGFCode | SPP定位 |
| 求解器 | SolverLSQ, SolverKalman | 最小二乘/卡尔曼 |
| 高级功能 | CSDetector, ARLambda | 周跳检测/模糊度固定 |

---

## 可执行程序

| 程序名 | 源文件 |
|--------|--------|
| parse_opt | exam-1.1-parse_opt.cpp |
| parse_config | exam-1.2-parse_config.cpp |
| eigen | exam-2.1-eigen.cpp |
| time_convert | exam-2.2-time_convert.cpp |
| coord_myself | exam-2.3-myself.cpp |
| satid | exam-3.1-satid.cpp |
| read_rinex_data | exam-3.2-read_rinex_data.cpp |
| gps_eph | exam-4.1-gps_eph.cpp |
| system_bias | exam-5.1-system_bias.cpp |
| sppif | exam-6.1-sppif.cpp |
| spp | exam-6.2-spp.cpp |
| sppUC | exam-6.3-GFCode.cpp |
| test | zuiyouguji/test.cpp |

---

## 编译命令

```bash
cd gnssLab-2.4/cmake-build-debug
make -j4
```

---

*最后更新: 2026-05-24*