//
// Created by zhang on 2026/5/5.
//

#ifndef BRDC00IGS_R_20250010000_01D_MN_RNX_SPPCODE_H
#define BRDC00IGS_R_20250010000_01D_MN_RNX_SPPCODE_H

#endif //BRDC00IGS_R_20250010000_01D_MN_RNX_SPPCODE_H
/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *  As stipulated by the MulanPSL-2.0, you are granted the following freedoms:
 *      To copy, use, and modify the software;
 *      To use the software for commercial purposes;
 *      To redistribute the software.
 *
 * Author: Shoujian Zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * References:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
 * 2. Eckel, Bruce. Thinking in C++. 2nd ed., Prentice Hall, 2000.
 */

#ifndef GNSSLAB_SPPCode_H
#define GNSSLAB_SPPCode_H

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "RinexNavStore.hpp"
#include "RinexObsReader.h"
#include <Eigen/Eigen>

// SPP 单历元解算结果结构体
struct SPPResult {
    CommonTime epoch;
    YDSTime ydsTime;
    Vector3d xyz;
    Vector3d enu;
    double pdop;
    int nSat;
    double sigma0;
    double meanResidual;
    double rmsResidual;
    double maxResidual;
    double meanTGD;
    double maxTGD;
    double minTGD;
    std::map<SatID, double> satTGDData;  // 各卫星的 TGD 改正值（单位：米）
    double meanRelativity;                // 平均相对论效应改正（单位：米）
    double maxRelativity;                 // 最大相对论效应改正（单位：米）
    double minRelativity;                 // 最小相对论效应改正（单位：米）
    std::map<SatID, double> satRelativityData;  // 各卫星的相对论效应改正值（单位：米）
};

// 单点测速结果结构体
struct SPPVelocityResult {
    Eigen::Vector3d vel;    // 接收机速度 (m/s), ECEF
    double cdt_dot;          // 接收机钟漂 × C (m/s)
    double vdop;             // 速度精度因子
    int nSat;                // 参与测速的卫星数
};

// 历元跳过统计结构体
struct EpochSkipStats {
    int totalEpochs = 0;
    int svNumException = 0;     // 卫星数不足
    int sigma0Zero = 0;         // sigma0 == 0
    int pdopInvalid = 0;        // PDOP 无效（NaN/∞/负）
    int pdopExceed = 0;         // PDOP > 10
    int iterNotConverge = 0;    // 迭代 > 10 次未收敛
    int sigma0Exceed = 0;       // sigma0 > 10.0
    int satNoCodeType = 0;      // 无匹配观测类型
    int satNoObsValue = 0;      // 观测值不存在
    int satEphFailed = 0;       // 星历计算失败
    int satOutlierDeleted = 0;  // 粗差剔除

    int totalSkipped() const {
        return svNumException + sigma0Zero + pdopInvalid +
               pdopExceed + iterNotConverge + sigma0Exceed;
    }
};

class SPPCode {
public:
    SPPCode()
    : pEphStore(NULL), isRover(true), sigCode(1.0), cutOffElev(15), sysCode(""),
      meoWeight(1.0), igsoWeight(0), geoWeight(0.0),
      relativityEnable(true), earthRotationEnable(true)  // 默认开启相对论效应和地球自转改正
    {}

    // TGD 改正值存储（每个历元更新）
    std::map<SatID, double> satTGDData;
    // 相对论效应改正值存储（每个历元更新）
    std::map<SatID, double> satRelativityData;

    void setStationAsBase()
    {
        isRover = false;
    }

    void setRinexNavStore(RinexNavStore* pStore)
    {
        pEphStore = pStore;
    };

    void setSystemCode(const std::string& sys)
    {
        sysCode = sys;
    }

    // 设置卫星类型权重
    void setSatTypeWeights(double meoW, double igsoW, double geoW)
    {
        meoWeight = meoW;
        igsoWeight = igsoW;
        geoWeight = geoW;
    }

    // 设置相对论效应改正开关
    void setRelativityEnable(bool enable)
    {
        relativityEnable = enable;
    }

    // 设置地球自转改正开关
    void setEarthRotationEnable(bool enable)
    {
        earthRotationEnable = enable;
    }

    void setSysTypes(const std::map<std::string, std::set<std::string>>& types_)
    {
        sysTypes = types_;
    }

    // 历元跳过统计
    void resetEpochSkipStats()
    {
        epochSkipStats = EpochSkipStats{};
    }

    static void printEpochSkipStats(const EpochSkipStats& stats);

    const EpochSkipStats& getEpochSkipStats() const { return epochSkipStats; }


    void clearOutlierSats() { outlierSats.clear(); }
    const std::set<SatID>& getOutlierSats() const { return outlierSats; }

    void solve(ObsData &obsData,bool TGD_bool=false,bool Trop_Bool=true,bool Iono_Bool=true);

    std::vector<SPPResult> full_solve(RinexNavStore* pStore, string roverFile,std::map<string, std::set<string>> sysTypes ,bool TGD_Bool=true, bool Trop_Bool=true, bool Iono_Bool=true);

    std::map<SatID,Xvt> computeSatPos(ObsData &obsData);

    bool strangeDataDelete(ObsData &obsData,double parameter=3.0);

    EquSys linearizeVelocity(ObsData &obsData, VariableSet &varSet, int &nSat);

    SPPVelocityResult solveVelocity(ObsData &obsData);


    Xvt computeAtTransmitTime(const CommonTime& tr,
                              const double& pr,
                              const SatID& sat);


    void correctTGD(ObsData &obsdata);

    void computeElevAzim(Eigen::Vector3d& xyz,
                          std::map<SatID,Xvt> & satXvtTransTime,
                          SatValueMap& tempElevData,
                          SatValueMap& tempAzimData);

    void convertObsType(ObsData &obsData);
    void computeIF(ObsData &obsData);
    std::map<SatID,double> computeTropDelay(ObsData &obsdata,std::map<SatID, double>&satElevData);
    std::map<SatID,Xvt> earthRotation(Eigen::Vector3d& xyz,
                                      std::map<SatID,Xvt> & satXvtTransTime);
    EquSys linearize(Eigen::Vector3d& xyz,
                     std::map<SatID,Xvt>& satXvtRecTime,
                     SatValueMap& satElevData,
                     ObsData& obsData,std::map<SatID,double>tropDelay);
    
    // BDS卫星类型处理
    std::string getBDSSatType(const SatID& sat,CommonTime epoch);

    double getTypeWeight(const SatID& sat,CommonTime epoch);

    EquSys getEquSys()
    {
        return equSys;
    };

    SatID getDatumSat()
    {
        double maxElev(0.0);
        SatID datumSat;
        for(auto se: satElevData)
        {
            if(se.second>maxElev)
            {
                maxElev = se.second;
                datumSat = se.first;
            }
        }
        return datumSat;
    };

    SatValueMap getSatElevData()
    {
        return satElevData;
    }

    Vector3d getXYZ()
    {
        return xyz;
    }

    Result getResult();

    ~SPPCode(){};

    // 继承类需要访问这个成员
protected:
    double cutOffElev;

    bool isRover;
    double sigCode;

    std::string sysCode;

    std::map<string, std::set<string>> sysTypes;

    // 卫星类型权重参数
    double meoWeight;
    double igsoWeight;
    double geoWeight;

    // 误差模型控制开关
    bool relativityEnable;      // 相对论效应改正开关
    bool earthRotationEnable;   // 地球自转改正开关

    EquSys equSys;
    Result result;

    Vector3d xyz;
    Vector3d dxyz;

    std::map<SatID,Xvt> satXvtTransTime;
    std::map<SatID,Xvt> satXvtRecTime;

    SatValueMap  satElevData;
    SatValueMap  satAzimData;
    SatValueMap  satTropData;
    SatValueMap  satIonoData;

    SolverLSQ  solverLsq;

    RinexNavStore* pEphStore;

    SatID datumSat;

    // 历元跳过统计实例
    EpochSkipStats epochSkipStats;

    // 粗差卫星降权集合（不删除，仅降权）
    std::set<SatID> outlierSats;

};


#endif //GNSSLAB_SPPCode_H