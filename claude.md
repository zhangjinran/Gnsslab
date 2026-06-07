# gnssLab-2.4 代码规范

## 1. 项目概述

gnssLab-2.4 是一个 GNSS（全球导航卫星系统）数据处理库，主要用于卫星定位、导航和授时相关的算法实现。项目采用 C++ 语言开发，基于 Eigen 矩阵库进行数值计算。

### 1.1 项目结构

```
gnssLab-2.4/
├── lib/               # 核心库代码
├── examples/          # 示例程序
├── data/              # 数据文件（RINEX观测文件、星历文件等）
├── doc/               # 参考文档
├── cmake-build-debug/ # 构建目录
├── CHANGELOG.md       # 版本变更日志
├── LICENSE            # 许可证文件
└── README.md          # 项目说明文档
```

### 1.2 重要参考文档

`doc/` 目录包含以下重要参考文档，开发时应参考：

| 文档名称 | 内容说明 |
|---------|---------|
| `ICD-GPS-200C.pdf` | GPS 接口控制文档，定义 GPS 信号规范和数据格式 |
| `rinex304.pdf` / `rinex305.pdf` / `rinex_4.00.pdf` | RINEX 格式规范，用于 GNSS 观测数据和导航数据交换 |
| `sp3d.pdf` | SP3 轨道产品格式规范 |
| `GPS-SV-velocity-and-acceleration.pdf` | GPS 卫星速度和加速度计算相关文档 |
| `OEM7_Commands_Logs_Manual.pdf` | NovAtel OEM7 接收机命令和日志手册 |
| `NovAtel_OEM4主板的GPS原始数据解码_贾蓉.pdf` | OEM4 主板 GPS 原始数据解码参考 |

### 1.3 lib 核心库目录结构

`lib/` 目录包含项目核心功能模块，按功能分类如下：

| 模块类别 | 文件 | 功能说明 |
|---------|------|---------|
| **基础工具** | `TimeConvert.h/cpp`, `TimeStruct.h/cpp`, `Time2020Convert.h/cpp` | 时间格式转换、时间系统处理 |
| **坐标转换** | `CoordConvert.h`, `CoordStruct.h` | 坐标系统转换（ECEF、BLH、ENU等） |
| **数据结构** | `GnssStruct.h/cpp`, `Const.h` | GNSS 数据结构定义、常量定义 |
| **星历处理** | `NavEphGPS.h/cpp`, `NavEphBDS.h/cpp`, `NavEphGLONASS.h/cpp`, `NavEphGalileo.h/cpp`, `NavEphQZSS.h/cpp`, `NavEphIRNSS.h/cpp`, `NavEphBase.h/cpp`, `NavEphRegistry.cpp` | 各GNSS系统广播星历解析与轨道计算 |
| **文件读取** | `RinexObsReader.h/cpp`, `RinexNavStore.h/cpp`, `SP3Store.h/cpp`, `Rx3ClockReader.h/cpp` | RINEX观测/导航文件、SP3精密星历、时钟文件读取 |
| **定位算法** | `SPPCode.h/cpp`, `SPPIFCode.h/cpp`, `SPPGFCode.h/cpp`, `SPPUCCodePhase.h/cpp` | SPP单点定位（码伪距、IF组合、GF组合、无电离层组合） |
| **求解器** | `SolverLSQ.h/cpp`, `SolverKalman.h/cpp`, `KalmanFilter.hpp/cpp` | 最小二乘、卡尔曼滤波求解器 |
| **高级功能** | `ARLambda.hpp/cpp`, `CSDetector.h/cpp` | LAMBDA模糊度固定算法、周跳检测 |
| **配置工具** | `ConfigReader.h/cpp`, `ConfigData.h`, `StringUtils.h` | 配置文件读取、字符串工具函数 |
| **数据输出** | `DataExporter.h/cpp`, `OrbitExporter.h/cpp`, `write.h/cpp` | 数据导出、轨道数据输出 |

### 1.4 examples 示例程序目录

`examples/` 目录包含各章节示例程序，对应课程实验内容：

| 示例文件 | 章节 | 功能说明 |
|---------|------|---------|
| `exam-1.1-parse_opt.cpp` | 第1章 | 命令行参数解析 |
| `exam-1.2-parse_config.cpp` | 第1章 | 配置文件读取 |
| `exam-2.1-eigen.cpp` | 第2章 | Eigen矩阵库入门 |
| `exam-2.2-time_convert.cpp` | 第2章 | 时间格式转换 |
| `exam-2.3-coord_convert*.cpp` | 第2章 | 坐标转换示例 |
| `exam-2.3-myself.cpp` | 第2章 | 坐标转换综合实验 |
| `exam-2.4-skyplot.cpp` | 第2章 | 天空图绘制 |
| `exam-3.1-satid.cpp` | 第3章 | 卫星ID管理 |
| `exam-3.2-read_rinex_data.cpp` | 第3章 | RINEX数据读取与星历验证 |
| `exam-4.1-gps_eph.cpp` | 第4章 | GPS广播星历处理 |
| `exam-5.1-system_bias.cpp` | 第5章 | 系统误差改正 |
| `exam-5.5-bds-satweight-test.cpp` | 第5章 | BDS卫星类型权重测试 |
| `exam-5.6-spp-error-model-test.cpp` | 第5章 | SPP误差模型检测（相对论效应、地球自转改正） |
| `exam-6.1-sppif.cpp` | 第6章 | SPP IF组合定位 |
| `exam-6.2-spp.cpp` | 第6章 | SPP单点定位 |
| `exam-6.3-GFCode.cpp` | 第6章 | GF组合定位 |
| `exam-6.4-sppif-bds-test.cpp` | 第6章 | BDS SPPIF观测码组合测试 |
| `exam-7.1-cs_detect_mw.cpp` | 第7章 | MW组合周跳检测 |
| `exam-8.1-sync_obs.cpp` | 第8章 | 观测数据同步 |
| `exam-8.2-diff_station.cpp` | 第8章 | 站间差分 |
| `exam-8.3-lambda.cpp` | 第8章 | LAMBDA模糊度固定 |
| `exam-8.4-rtk_lsq.cpp` | 第8章 | RTK最小二乘求解 |
| `exam-8.5-rtk_kal.cpp` | 第8章 | RTK卡尔曼滤波求解 |

### 1.5 支持的卫星系统

| 系统标识 | 卫星系统 | 说明 | 支持状态 |
|---------|---------|------|----------|
| G | GPS | 美国全球定位系统 | ✅ 支持 |
| C | BDS | 中国北斗卫星导航系统 | ✅ 支持 |
| R | GLONASS | 俄罗斯格洛纳斯系统 | ✅ 支持 |
| E | Galileo | 欧洲伽利略系统 | ✅ 支持 |
| J | QZSS | 日本准天顶卫星系统 | ✅ 支持 |
| I | IRNSS | 印度区域导航系统 | ✅ 支持 |
| S | SBAS | 卫星增强系统 | ⏳ 暂不支持 |

### 1.6 数据导出规范

**数据流向**：`gnssLab-2.4` 作为数据处理核心，负责 GNSS 数据解析和计算，处理结果直接输出到 `gnss_draw/data/` 目录供可视化使用。

**导出目录结构**：
```
gnss_draw/data/
├── orbit/           # 轨道数据
├── time_system/    # 时间系统数据
├── coord_system/   # 坐标系统数据
└── <其他类别>/     # 可扩展目录
```

**统一文件格式**（适用于所有数据类型）：

1. **文件头部**（注释行）：
   ```text
   # <数据描述>
   # Format: <字段1>,<字段2>,...
   ```

2. **数据部分**：
   - 使用**逗号分隔**
   - 无需表头行（表头信息已在第二行注释中说明）
   - 示例：
     ```text
     # Orbit Data for Satellite G1
     # Format: time(MJD+SOD),x(m),y(m),z(m),radius(m),lat(deg),lon(deg)
     60676.0000000000,15931688.883,2160463.852,21149133.895,26566373.092,52.80244991,7.72265516
     60676.0034722222,16127773.863,2937130.969,20905518.362,26566379.620,51.94305632,10.32138153
     ```

3. **时间输出格式（硬性规则）**：
   - **必须使用 `YDSTime` 格式**（与 exam5.3 保持一致）
   - 格式说明：`年(4位) 年内第几天(3位) 当日秒数(浮点) 时间系统`
   - 示例：`2025 001 3600.0 GPS`
   - 实现方式：使用 `CommonTime2YDSTime(epoch)` 转换，直接输出 `YDSTime` 对象
   - **禁止使用**：MJD 格式、CivilTime 格式或其他时间格式

**文件命名规范**：
- 轨道数据：`<卫星ID>_orbit.txt`，如 `G1_orbit.txt`
- 组合轨道：`<卫星ID>_combined_orbit.txt`，如 `G1_combined_orbit.txt`
- 地面轨迹：`ground_track.txt`
- 时间转换：`time_conversion.txt`
- 坐标转换：`coord_conversion.txt`
- 还可以根据需要添加其他数据类型，如误差分析数据等。

**导出流程**：
```
gnssLab-2.4/examples/
    ↓ 运行示例程序
gnss_draw/data/<类别>/
    ↓ 直接写入
gnss_draw/figure/<类别>/
    ↓ 可视化
```

---

## 2. 命名规范

### 2.1 文件命名

- **头文件**：使用 `.hpp` 或 `.h` 扩展名
- **源文件**：使用 `.cpp` 扩展名
- **命名规则**：全部小写，单词之间用下划线分隔
  - 示例：`gnss_func.hpp`, `nav_eph_gps.cpp`

### 2.2 类和结构体命名

- 使用 **PascalCase**（大驼峰式）
- 类名使用名词或名词短语
- 结构体名使用名词或名词短语
- 示例：`NavEphGPS`, `SatID`, `ObsData`

### 2.3 函数命名

- 使用 **camelCase**（小驼峰式）
- 函数名使用动词或动词短语
- 示例：`computeSatPos`, `parseRinexObs`, `writefileSatPos`

### 2.4 变量命名

- 使用 **camelCase**（小驼峰式）
- 变量名应具有描述性
- 避免使用单个字符作为变量名（循环计数器除外）
- 示例：`satXvtTransTime`, `obsData`, `rinexHeader`

### 2.5 常量命名

- 使用 **全大写**，单词之间用下划线分隔
- 常量名应具有描述性
- 示例：`L1_FREQ_GPS`, `C_MPS`, `PI`

### 2.6 类型别名命名

- 使用 **PascalCase**
- 以 `Map`, `Set`, `Vector`, `List` 等结尾
- 示例：`SatValueMap`, `VariableSet`, `SatEpochValueMap`

### 2.7 宏命名

- 使用 **全大写**，单词之间用下划线分隔
- 示例：`MAX_SATELLITES`, `DEFAULT_RH`

---

## 3. 代码风格

### 3.1 缩进

- 使用 **4 个空格**进行缩进
- 不要使用 Tab 字符

### 3.2 大括号

- **函数/类/结构体**：左大括号单独占一行
- **控制语句**：左大括号单独占一行

```cpp
// 正确
class NavEphGPS : public NavEphBase {
public:
    NavEphGPS(void) {}
};

// 正确
if (condition) {
    // code
} else {
    // code
}
```

### 3.3 空格

- 二元运算符两侧各有一个空格
- 逗号后应有一个空格
- 函数调用时，函数名与左括号之间无空格
- 左括号后、右括号前无空格

```cpp
// 正确
double result = a + b * c;
void func(int a, int b);
func(1, 2);

// 错误
double result=a+b*c;
void func(int a,int b);
func( 1 , 2 );
```

### 3.4 行长度

- 每行代码不超过 **120 个字符**
- 过长的表达式应适当换行

### 3.5 头文件保护

- 使用 `#pragma once`（推荐）或传统的 include guard

```cpp
#pragma once

// 或传统方式
#ifndef GNSSLAB_GNSSFUNC_H
#define GNSSLAB_GNSSFUNC_H
// ...
#endif // GNSSLAB_GNSSFUNC_H
```

---

## 4. 类和结构体设计

### 4.1 类设计原则

1. **单一职责原则**：一个类只负责一个功能
2. **开放封闭原则**：对扩展开放，对修改封闭
3. **Liskov 替换原则**：子类可以替换父类

### 4.2 成员变量

- **访问控制**：
  - `public`：对外接口
  - `protected`：子类可访问
  - `private`：内部实现

### 4.3 成员函数

- 提供清晰的接口文档
- 使用 `const` 关键字标识不修改对象状态的方法
- 虚函数应声明为 `virtual`，必要时使用 `override`

```cpp
class NavEphGPS : public NavEphBase {
public:
    double svClockBias(const CommonTime &t) const;  // const 方法
    virtual TimeSystem getTimeSystem() const override;  // override
};
```

### 4.4 结构体设计

- 结构体用于数据聚合，不包含复杂逻辑
- 成员变量通常为 `public`

---

## 5. 函数设计

### 5.1 函数参数

- 参数顺序：输入参数在前，输出参数在后
- 使用引用传递避免大对象拷贝
- 使用 `const` 引用传递不需要修改的参数

```cpp
void computeSatPos(const ObsData& obsData,           // 输入
                   const RinexNavStore& navStore,     // 输入
                   std::map<SatID, Xvt>& satXvt);     // 输出
```

### 5.2 返回值

- 避免返回复杂对象的拷贝，使用引用或指针
- 对于可能失败的操作，考虑返回 `bool` 或使用异常

### 5.3 函数长度

- 单个函数不应超过 **50 行**
- 复杂逻辑应拆分为多个小函数

---

## 6. 常量和宏

### 6.1 常量定义

- 使用 `const` 或 `constexpr` 定义常量
- 避免使用宏定义常量（除条件编译外）

```cpp
// 推荐
const double C_MPS = 2.99792458e8;
constexpr double PI = 3.14159265358979323846;

// 避免
#define C_MPS 2.99792458e8
```

### 6.2 宏使用

- 宏应使用大写命名
- 复杂宏应使用括号保护参数

```cpp
#define MAX(a, b) ((a) > (b) ? (a) : (b))
```

---

## 7. 注释规范

### 7.1 文件头部注释

每个源文件和头文件开头必须包含版权声明：

```cpp
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
 * References:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
 */
```

### 7.2 类和函数注释

- 使用 Doxygen 风格注释
- 类注释说明类的职责和用途
- 函数注释说明参数、返回值和功能

```cpp
/**
 * Compute satellite position at transmission time
 * 
 * @param tr        Transmission time
 * @param pr        Pseudorange (meters)
 * @param sat       Satellite ID
 * @param navStore  Navigation ephemeris store
 * @param IF        Frequency index (default: 0)
 * @return          Xvt structure containing position, velocity and clock data
 */
Xvt computeAtTransmitTime(const CommonTime& tr,
                          const double& pr,
                          const SatID& sat,
                          RinexNavStore& navStore,
                          int IF=0);
```

### 7.3 行内注释

- 注释应解释**为什么**而不是**做什么**
- 避免冗余注释
- 复杂算法应添加注释说明

---

## 8. 异常处理

### 8.1 异常类型

- 使用标准异常：`std::invalid_argument`, `std::out_of_range`, `std::runtime_error`
- 自定义异常应继承自 `std::exception`

### 8.2 异常使用场景

- 参数验证失败时抛出异常
- 资源获取失败时抛出异常
- 不使用异常进行流程控制

```cpp
SatID::SatID(string satStr) {
    if (satStr.length() < 3) {
        throw std::invalid_argument("SatID: Invalid satellite string format");
    }
}
```

### 8.3 错误处理

- 使用 `std::cerr` 输出错误信息
- 关键错误应终止程序执行

---

## 9. 性能注意事项

### 9.1 避免不必要的拷贝

- 使用引用传递大对象
- 使用移动语义（C++11+）

### 9.2 Eigen 矩阵优化

- 使用 Eigen 的对齐特性
- 避免动态内存分配
- 使用适当的矩阵存储顺序

### 9.3 循环优化

- 减少循环内的计算量
- 使用高效的数据结构
- 考虑并行化处理

---

## 10. 代码组织

### 10.1 头文件顺序

1. 标准库头文件（按字母顺序）
2. 第三方库头文件（如 Eigen）
3. 项目内部头文件（按模块组织）

```cpp
#include <string>
#include <vector>
#include <map>
#include <Eigen/Eigen>
#include "CoordStruct.h"
#include "GnssStruct.h"
```

### 10.2 命名空间

- 避免使用 `using namespace std;`（头文件中禁止）
- 可以使用 `using namespace Eigen;`（项目约定）

---

## 11. 测试规范

### 11.1 测试文件命名

- 测试文件命名为 `test_xxx.cpp`
- 测试数据放在 `data/` 目录

### 11.2 测试覆盖

- 单元测试覆盖核心函数
- 集成测试验证模块协作
- 边界条件测试

---

## 12. 版本控制

### 12.1 提交规范

- 提交信息应清晰描述修改内容
- 使用英文或中文描述
- 格式：`[模块] 简要描述`

### 12.2 分支管理

- `main`：稳定版本
- `develop`：开发分支
- `feature/*`：功能分支
- `fix/*`：修复分支

### 12.3 CHANGELOG 维护规范

**重要要求**：每次运行开发任务前，必须先查看 `CHANGELOG.md` 了解最新变更记录。完成任务后，必须在获得用户允许的情况下，将完成的工作记录到 `CHANGELOG.md` 中。

**CHANGELOG 记录格式**：

```markdown
### YYYY-MM-DD

**今日任务完成情况**:

| 序号 | 任务 | 模块 | 状态 |
|------|------|------|------|
| 1 | 任务描述 | 所属模块 | ✅ 完成 |

**新增**:

1. **功能名称** (`文件路径`):
   - 功能描述
   - 关键实现细节

**修复**:

1. **问题描述** (`文件路径`):
   - 问题原因
   - 解决方案
```

**记录内容要求**：
- 记录新增功能、修复的 bug、代码重构
- 注明涉及的文件路径
- 描述关键实现细节或问题原因
- 使用统一的状态标识（✅ 完成、🔄 进行中、⏳ 待办）

---

## 附录：常用缩写

| 缩写 | 全称 | 中文 |
|------|------|------|
| GNSS | Global Navigation Satellite System | 全球导航卫星系统 |
| GPS | Global Positioning System | 全球定位系统 |
| BDS | BeiDou Navigation Satellite System | 北斗卫星导航系统 |
| GLONASS | Global Navigation Satellite System | 格洛纳斯系统 |
| SBAS | Satellite-Based Augmentation System | 卫星增强系统 |
| RINEX | Receiver Independent Exchange Format | 接收机独立交换格式 |
| SP3 | Standard Product 3 | 标准产品3格式 |
| ECEF | Earth-Centered, Earth-Fixed | 地心地固坐标系 |
| ITRF | International Terrestrial Reference Frame | 国际地球参考框架 |
| SPP | Single Point Positioning | 单点定位 |
| RTK | Real-Time Kinematic | 实时动态定位 |