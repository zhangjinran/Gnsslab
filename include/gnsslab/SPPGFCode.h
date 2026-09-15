//
// Created by zhang on 2026/5/5.
//

#ifndef BRDC00IGS_R_20250010000_01D_MN_RNX_SPPGFCode_H
#define BRDC00IGS_R_20250010000_01D_MN_RNX_SPPGFCode_H

#endif //BRDC00IGS_R_20250010000_01D_MN_RNX_SPPGFCode_H
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

#ifndef GNSSLAB_SPPGFCode_H
#define GNSSLAB_SPPGFCode_H

#include <gnsslab/GnssStruct.h>
#include <gnsslab/SolverLSQ.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/RinexObsReader.h>
#include <Eigen/Eigen>

// 历元跳过统计（精简版）
struct GFEpochSkipStats {
    int totalEpochs = 0;
    int svNumException = 0;
    int pdopInvalid = 0;
    int pdopExceed = 0;
    int iterNotConverge = 0;
    int sigma0Exceed = 0;
    int satOutlierDeleted = 0;

    int totalSkipped() const {
        return svNumException + pdopInvalid + pdopExceed + iterNotConverge + sigma0Exceed;
    }
};

class SPPGFCode {
public:
    SPPGFCode()
    : pEphStore(NULL), isRover(true), sigGFCode(1.0), cutOffElev(10)
    {}

    void setStationAsBase()
    {
        isRover = false;
    }

    void setRinexNavStore(RinexNavStore* pStore)
    {
        pEphStore = pStore;
    };

    void setSysTypes(const std::map<std::string, std::set<std::string>>& types_)
    {
        sysTypes = types_;
    }

    void setRelativityEnable(bool enable)
    {
        relativityEnable = enable;
    }

    void setEarthRotationEnable(bool enable)
    {
        earthRotationEnable = enable;
    }

    void resetEpochSkipStats() { gfEpochSkip = GFEpochSkipStats{}; }
    void printEpochSkipStats(int totalEpochs = 0) const;
    const GFEpochSkipStats& getEpochSkipStats() const { return gfEpochSkip; }


    void solve(ObsData &obsData,bool TGD_bool=true,bool Trop_Bool=true);

    void full_solve(RinexNavStore* pStore,string solFile,string roverFile,bool TGD_Bool=true,bool Trop_Bool=true);

    std::map<SatID,Xvt> computeSatPos(ObsData &obsData);

    bool strangeDataDelete(ObsData &obsData,double parameter=3.0);



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

    double getPDOP() const { return result.pdop; }
    double getSigma0() const { return sigma0Val; }
    double getMeanResidual() const { return meanResidual; }
    double getMaxResidual() const { return maxResidual; }
    int getNSat() const { return result.numSats; }

    Result getResult();

    ~SPPGFCode(){};

    // 继承类需要访问这个成员
protected:
    double cutOffElev;

    bool isRover;
    double sigGFCode;

    std::map<std::string, std::set<std::string>> sysTypes;

    // 粗差卫星降权集合（不删除，仅降权）
    std::set<SatID> outlierSats;

    double sigma0Val = 0.0;
    double meanResidual = 0.0, maxResidual = 0.0;

    GFEpochSkipStats gfEpochSkip;

    EquSys equSys;
    Result result;

    Vector3d xyz;
    Vector3d dxyz;

    std::map<SatID,Xvt> satXvtTransTime;
    std::map<SatID,Xvt> satXvtRecTime;

    SatValueMap  satElevData;
    SatValueMap  satAzimData;
    SatValueMap  satTropData;

    SolverLSQ  solverLsq;

    RinexNavStore* pEphStore;

    bool relativityEnable = true;
    bool earthRotationEnable = true;

    SatID datumSat;

};


#endif //GNSSLAB_SPPGFCode_H