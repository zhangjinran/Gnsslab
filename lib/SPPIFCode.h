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

#ifndef GNSSLAB_SPPIFCODE_H
#define GNSSLAB_SPPIFCODE_H

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "RinexNavStore.hpp"
#include "RinexObsReader.h"
#include <Eigen/Eigen>

// SPP IF组合单历元解算结果结构体
struct SPPIFResult {
    CommonTime epoch;
    YDSTime ydsTime;  // 使用 YDSTime 格式（年/日/秒），与 exam5.3 保持一致
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
    std::map<SatID, double> satTGDData;
};

class SPPIFCode {
public:
    SPPIFCode()
    : pEphStore(NULL), isRover(true), sigIFCode(1.0), cutOffElev(10), meoWeight(1.0), igsoWeight(0.25), geoWeight(0.25)
    {}

    void setStationAsBase()
    {
        isRover = false;
    }

    void setRinexNavStore(RinexNavStore* pStore)
    {
        pEphStore = pStore;
    };
    bool strangeDataDelete(ObsData &obsData,double parameter=3.0);
    void setIFCodeTypes(std::map<string, std::pair<string, string>>& ifTypes)
    {
        ifCodeTypes = ifTypes;
    };
    
    void setSelectedTypes(const std::map<string, std::set<string>>& types)
    {
        selectedTypes = types;
    }

    void solve(ObsData &obsData,bool TGD_bool=true,bool Trop_Bool=true);

    std::vector<SPPIFResult> full_solve(RinexNavStore* pStore, std::map<string, std::pair<string, string>> ifCodeTypes, string roverFile, bool TGD_Bool=true, bool Trop_Bool=true);

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

    void setRelativityEnable(bool enable)
    {
        relativityEnable = enable;
    }

    void setEarthRotationEnable(bool enable)
    {
        earthRotationEnable = enable;
    }

    // BDS卫星类型处理
    std::string getBDSSatType(const SatID& sat, CommonTime epoch);
    double getTypeWeight(const SatID& sat, CommonTime epoch);

    std::map<SatID,Xvt> computeSatPos(ObsData &obsData);



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

    ~SPPIFCode(){};

    // 继承类需要访问这个成员
protected:
    double cutOffElev;

    bool isRover;
    double sigIFCode;

    std::string sysCode;
    
    // 卫星类型权重
    double meoWeight ;    // MEO 卫星权重
    double igsoWeight ;   // IGSO 卫星权重
    double geoWeight ;    // GEO 卫星权重

    bool relativityEnable = true;      // 相对论效应改正开关
    bool earthRotationEnable = true;   // 地球自转改正开关

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

    std::map<string, std::pair<string, string>> ifCodeTypes;
    
    std::map<string, std::set<string>> selectedTypes;

    std::map<SatID, double> satTGDData;

    SatID datumSat;

    // 粗差卫星降权集合（不删除，仅降权）
    std::set<SatID> outlierSats;

};



#endif //GNSSLAB_SPPIFCODE_H