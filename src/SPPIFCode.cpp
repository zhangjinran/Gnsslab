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

#include <gnsslab/SPPIFCode.h>
#include <gnsslab/CoordConvert.h>
#include <iostream>
#include <numbers>
#include <gnsslab/RinexObsReader.h>
#include <Eigen/Eigen>


#include <gnsslab/ARLambda.hpp>
#include <gnsslab/GnssFunc.h>
#include <gnsslab/NavEphGPS.hpp>
#include <gnsslab/NavEphBDS.hpp>
#include <gnsslab/NavEphGLONASS.hpp>

#define debug 0

void SPPIFCode::solve(ObsData &obsData,bool TGD_bool,bool Trop_Bool) {
    //----------------------
    // 去掉通道号，C1W, C1C => C1;
    // 后面computeSatPos里用与通道号无关的观测值计算卫星发射时刻位置
    //----------------------
    if (debug) {
    for (auto tv: obsData.satTypeValueData) {
        if (tv.first.system=="G") {
            cout<<"sat:"<<tv.first<<endl;
            cout<<tv.second<<endl;
        }
    }
    }
    convertObsType(obsData);

    if(debug)
    {
        cout << "after convertObsType" << endl;
        for (auto tv: obsData.satTypeValueData) {
            if (tv.first.system=="G") {
                cout<<"sat:"<<tv.first<<endl;
                cout<<tv.second<<endl;
            }
        }
    }

    // 先进行TGD改正（必须在IF组合之前）
    // TGD是频率相关误差，必须在形成IF之前应用于单频观测
    if (TGD_bool) {
        correctTGD(obsData);
    }

    // 计算IF组合
    computeIF(obsData); 
    if(debug)
        cout<<obsData<<endl;

    // 计算发射时刻卫星位置（参考框架为时刻的）
    satXvtTransTime = computeSatPos(obsData);
    if(debug)
    {
        cout << "satXvtTransTime:" << CommonTime2CivilTime(obsData.epoch) << endl;
        for(auto sx: satXvtTransTime)
        {
            cout << sx.first  << endl;
            cout << sx.second << endl;
        };
    }

    //----------------------
    // 得到卫星发射时刻位置和钟差、相对论和TGD后，改正观测值延迟，并更新C1/C2等观测值
    //----------------------
    // todo:
    //if (TGD_bool)
       // correctTGD(obsData);

    xyz = obsData.antennaPosition;
    dxyz = {100, 100, 100};


    int iter(0);
    while (true) {
        YDSTime ydstime(2025,1,38190);
        YDSTime yds_epoch=CommonTime2YDSTime(obsData.epoch);
        if (yds_epoch==ydstime) {
            cout<<endl;
        }


        if (earthRotationEnable)
            satXvtRecTime = earthRotation(xyz, satXvtTransTime);
        else
            satXvtRecTime = satXvtTransTime;

        if(debug)
        {
            cout << "satXvtRecTime" << endl;
            for(auto sx: satXvtRecTime)
            {
                cout << sx.first << " xvt:" << endl;
                cout << sx.second << endl;
            };
        }

        // step 1: 确定观测值和未知参数的纬数
        // 根据数据结构中已经有的satTypePrefitData, satTypeVarCoeffData;
        // 得到numObs, numUnk的数值
        int numSats = obsData.satTypeValueData.size();

        // 这里应该抛出异常，而不是break，因为无法解算，所以后续rtk也不能算，
        // 所以在rtk的主程序里捕获这个异常，然后再continue下一个历元；
        // 如果break了，就不知道问题在哪里了
        if (numSats < 4 ) {
            SVNumException e("num of satellites is less than 4");
            throw(e);
        }

        std::map<SatID,double>tropdelaymap=std::map<SatID,double>();
        // 地球表面才计算高度角和大气改正
        if(std::abs(xyz.norm() - RadiusEarth) < 100000.0)
        {
            satElevData.clear();
            satAzimData.clear();
            if(debug)
                cout << "computeElevAzim" << endl;

            computeElevAzim(xyz, satXvtRecTime,satElevData,satAzimData);

            if(debug)
            {
                cout << "satElevData:" << endl;
                cout << satElevData << endl;
            }

            // todo:
            // computeIonoDelay();
            if (Trop_Bool) {
                tropdelaymap=computeTropDelay(obsData,satElevData);
                if (debug) {
                    cout << "\n=== 对流层延迟 ===" << endl;
                    for (auto& td : tropdelaymap)
                        cout << td.first << "  " << fixed << setprecision(3) << td.second << " m" << endl;
                }
            }
        }

        equSys = linearize(xyz, satXvtRecTime, satElevData, obsData,tropdelaymap);

        if(debug)
            cout << "afte linearize:" << endl;

        // 如果是基准站，完成线性化后就退出
        // 因为基准站位置是准确的
        if(!isRover)
            break;

        solverLsq.solve(equSys);

        if (debug) {
            VariableSet vs = equSys.varSet;
            VectorXd st = solverLsq.getState();
            double cdt_val = 0;
            try { cdt_val = solverLsq.getSolution(Parameter::cdt, vs, st); } catch (...) {}
            cout << "\n=== LSQ 结果 ===" << endl;
            cout << "dxyz: (" << fixed << setprecision(3)
                 << solverLsq.getdxyz()[0] << ", "
                 << solverLsq.getdxyz()[1] << ", "
                 << solverLsq.getdxyz()[2] << ")  cdt: " << cdt_val << endl;
        }

        dxyz = solverLsq.getdxyz();

        xyz += dxyz;
        if (debug) {
            cout << "iteration:" << iter<<" "
            << "dxyz:" << dxyz.transpose()<<" "
            << "xyz:" << xyz.transpose() << endl;
        }

        // 根据 sigma0 动态设置粗差探测参数
        double sigma0 = solverLsq.getSigma0();
        
        // 检查 sigma0 是否为异常值（NaN、无穷大、非正数），如果异常则跳过整个历元
        if (std::isnan(sigma0) || std::isinf(sigma0) || sigma0 <= 0) {
            if (debug) {
                cout << "\n【sigma0异常】sigma0 = " << sigma0 << "，跳过该历元" << endl;
            }
            throw InvalidSolver("sigma0 is invalid");
        }
        
        double rejectThreshold = 0;
        if (sigma0 < 3) {
            // sigma0 < 3，不需要粗差探测
        } else if (sigma0 < 8) {
            rejectThreshold = 3 ;
        } else if (sigma0 < 15) {
            rejectThreshold = 2 ;
        } else {
            rejectThreshold = 1.5;
        }

        if (rejectThreshold > 0 && strangeDataDelete(obsData, rejectThreshold)) {
            xyz = obsData.antennaPosition;
            dxyz = {100, 100, 100};
            continue;
        }

        if (dxyz.norm() < 0.001) {
            break;
        }

        if (iter > 10) {
            InvalidSolver e("too many iterations");
            throw(e);
        }
        iter++;

    }

    result.xyz = xyz;
}


std::map<SatID, Xvt> SPPIFCode::computeSatPos(ObsData &obsData) {
    std::map<SatID, Xvt> satXvtData;
    SatIDSet satRejectedSet;
    CommonTime time = obsData.epoch;
    // Loop through all the satellites
    for (auto stv: obsData.satTypeValueData) {
        SatID sat(stv.first);
        Xvt xvt;
        // compute satellite ephemeris at transmitting time
        // Scalar to hold temporal value
        double obs(0.0);
        string codeType;
        
        // 从 ifCodeTypes 动态获取观测类型
        auto sysIt = ifCodeTypes.find(sat.system);
        if (sysIt != ifCodeTypes.end()) {
            codeType = sysIt->second.first;  // 使用第一个频率类型
            if (debug) {
                cout << "[DEBUG computeSatPos] Satellite: " << sat 
                     << " System: " << sat.system 
                     << " Selected codeType: " << codeType 
                     << " (from IF pair: " << sysIt->second.first << "," << sysIt->second.second << ")" << endl;
            }
        } else {
            if (debug) {
                cout << "[DEBUG computeSatPos] Satellite: " << sat 
                     << " System: " << sat.system 
                     << " Rejected - no IF config found" << endl;
            }
            satRejectedSet.insert(sat);
            continue;
        }

        // 显示该卫星所有可用的观测类型
        if (debug) {
            cout << "[DEBUG computeSatPos] Satellite: " << sat << " Available obs types: ";
            for (const auto& tv : stv.second) {
                cout << tv.first << "(" << fixed << setprecision(1) << tv.second << "m) ";
            }
            cout << endl;
        }

        // code obs
        try {
            obs = stv.second.at(codeType);
            if(debug)
                cout << "[DEBUG computeSatPos] Satellite: " << sat 
                     << " Successfully read " << codeType << " = " << fixed << setprecision(3) << obs << " m" << endl;
        }
        catch (...) {
            if (debug) {
                cout << "[DEBUG computeSatPos] Satellite: " << sat 
                     << " Rejected - " << codeType << " not found in available types" << endl;
            }
            satRejectedSet.insert(sat);
            continue;
        }

        // now, compute xvt
        try {
            xvt = computeAtTransmitTime(time, obs, sat);
        }
        catch (InvalidRequest &e) {
            satRejectedSet.insert(sat);
            continue;
        }
        satXvtData[sat] = xvt;
    }

    // remove bad sat;
    for (auto sat: satRejectedSet) {
        obsData.satTypeValueData.erase(sat);
    }

    return satXvtData;

};

Xvt SPPIFCode::computeAtTransmitTime(const CommonTime &tr,
                                     const double &pr,
                                     const SatID &sat)
noexcept(false) {
    Xvt xvt;

    CommonTime tt;
    CommonTime transmit = tr;

    transmit -= pr / C_MPS;
    tt = transmit;

    // 这里也可以用while循环来替换这里的迭代次数
    for (int i = 0; i < 2; i++) {
        if (pEphStore != NULL) {
            xvt = pEphStore->getXvt(sat, tt);

        }
        tt = transmit;
        if (relativityEnable)
            tt -= (xvt.clkbias + xvt.relcorr);
        else
            tt -= xvt.clkbias;
    }
    return xvt;
};

void SPPIFCode::convertObsType(ObsData &obsData) {

    SatTypeValueMap stvData;
    for (auto sd: obsData.satTypeValueData) {
        TypeValueMap tvData;
        for (auto td: sd.second) {
            tvData[td.first.substr(0, 2)] = td.second;
        }
        stvData[sd.first] = tvData;
    }

    // 替代
    obsData.satTypeValueData = stvData;
};

void SPPIFCode::computeIF(ObsData &obsData) {
    SatIDSet satRejectedSet;
    
    if (debug) {
        cout << "[DEBUG computeIF] Starting IF combination computation" << endl;
        cout << "[DEBUG computeIF] ifCodeTypes configuration:" << endl;
        for (const auto& it : ifCodeTypes) {
            cout << "[DEBUG computeIF]   System " << it.first << ": (" 
                  << it.second.first << ", " << it.second.second << ")" << endl;
        }
    }
    
    // Loop through all the satellites
    for (auto &stv: obsData.satTypeValueData) {
        SatID sat = stv.first;
        string sys = stv.first.system;
        
        if (debug) {
            cout << "[DEBUG computeIF] Processing satellite: " << sat 
                 << " System: " << sys << endl;
        }
        
        // get type for current system
        std::pair<string, string> ifPair;
        try {
            ifPair = ifCodeTypes.at(sys);
            if (debug) {
                cout << "[DEBUG computeIF]   Found IF config: " << ifPair.first << " + " << ifPair.second << endl;
            }
        }
        catch (...) {
            if (debug) {
                cout << "[DEBUG computeIF]   No IF config found for system " << sys << ", rejecting satellite" << endl;
            }
            satRejectedSet.insert(stv.first);
            continue;
        }

        // if组合的具体公式为：
        // if12 = (f1^2*P1 - f2^2*P2)/(f1^2-f2^2);
        if (debug) {
            cout << "[DEBUG computeIF]   IF combination formula: IF = (f1^2*P1 - f2^2*P2)/(f1^2-f2^2)" << endl;
        }

        double f1, f2;
        
        // 对于 GLONASS，需要从星历中获取 freqNum 来计算实际频率
        if (sys == "R" && pEphStore != NULL) {
            try {
                NavEphGLONASS gloEph = pEphStore->findGLOEph(sat, obsData.epoch);
                f1 = gloEph.getFreq(ifPair.first);
                f2 = gloEph.getFreq(ifPair.second);
                
                if (debug) {
                    cout << "[DEBUG computeIF]   GLONASS satellite " << sat 
                         << " freqNum = " << gloEph.freqNum << endl;
                    cout << "[DEBUG computeIF]   Frequencies (using freqNum): " << ifPair.first 
                         << " = " << fixed << setprecision(6) << f1/1e6 << " MHz, " 
                         << ifPair.second << " = " << fixed << setprecision(6) << f2/1e6 << " MHz" << endl;
                }
            } catch (...) {
                // 如果无法获取星历，使用默认频率
                f1 = getFreq(sys, ifPair.first);
                f2 = getFreq(sys, ifPair.second);
                if (debug) {
                    cout << "[DEBUG computeIF]   Failed to get GLONASS ephemeris, using default frequencies" << endl;
                }
            }
        } else {
            f1 = getFreq(sys, ifPair.first);
            f2 = getFreq(sys, ifPair.second);
        }
        
        if(debug) {
            cout << "[DEBUG computeIF]   Frequencies: " << ifPair.first << " = " << fixed << setprecision(3) << f1/1e6 << " MHz, " 
                 << ifPair.second << " = " << fixed << setprecision(3) << f2/1e6 << " MHz" << endl;
        }

        // 提取观测值
        double value1, value2, ifValue;

        try {
            value1 = stv.second.at(ifPair.first);
            value2 = stv.second.at(ifPair.second);
            
            if (debug) {
                cout << "[DEBUG computeIF]   Raw observations:" << endl;
                cout << "[DEBUG computeIF]     " << ifPair.first << " = " << fixed << setprecision(3) << value1 << " m" << endl;
                cout << "[DEBUG computeIF]     " << ifPair.second << " = " << fixed << setprecision(3) << value2 << " m" << endl;
            }
            
            // IF组合计算
            double f1_2 = f1 * f1;
            double f2_2 = f2 * f2;
            ifValue = (f1_2 * value1 - f2_2 * value2) / (f1_2 - f2_2);

            if (debug) {
                cout << "[DEBUG computeIF]   IF combination calculation:" << endl;
                cout << "[DEBUG computeIF]     IF = (" << f1_2/1e12 << "e12 * " << value1 << " - " 
                     << f2_2/1e12 << "e12 * " << value2 << ") / (" << (f1_2 - f2_2)/1e12 << "e12)" << endl;
                cout << "[DEBUG computeIF]     IF = " << fixed << setprecision(3) << ifValue << " m" << endl;
            }
            
            // 生成IF组合观测码名称
            string ifCodeStr = "CC" + ifPair.first.substr(1, 1) + ifPair.second.substr(1, 1);
            stv.second[ifCodeStr] = ifValue;
            
            if (debug) {
                cout << "[DEBUG computeIF]   Generated IF code: " << ifCodeStr << " = " << fixed << setprecision(3) << ifValue << " m" << endl;
                cout << "[DEBUG computeIF]   Updated obs types for " << sat << ": ";
                for (const auto& tv : stv.second) {
                    cout << tv.first << " ";
                }
                cout << endl;
            }
        }
        catch (...) {
            if (debug) {
                cout << "[DEBUG computeIF]   Failed to get observations for " << ifPair.first << " or " << ifPair.second << endl;
                cout << "[DEBUG computeIF]   Available obs types: ";
                for (const auto& tv : stv.second) {
                    cout << tv.first << " ";
                }
                cout << endl;
                cout << "[DEBUG computeIF]   Rejecting satellite " << sat << endl;
            }
            satRejectedSet.insert(stv.first);
        }
    }

    // remove bad sat;
    if (debug && !satRejectedSet.empty()) {
        cout << "[DEBUG computeIF] Rejected " << satRejectedSet.size() << " satellites:" << endl;
        for (const auto& sat : satRejectedSet) {
            cout << "[DEBUG computeIF]   " << sat << endl;
        }
    }
    
    for (auto sat: satRejectedSet) {
        obsData.satTypeValueData.erase(sat);
    }

    if (debug) {
        cout << "[DEBUG computeIF] IF combination completed. Remaining satellites: " << obsData.satTypeValueData.size() << endl;
    }
};

std::map<SatID, Xvt> SPPIFCode::earthRotation(Eigen::Vector3d &xyz,
                                              std::map<SatID, Xvt> &satXvtTransTime) {

    std::map<SatID, Xvt> satXvtRecTime;
    for(auto stv: satXvtTransTime) {
        SatID sat = stv.first;
        XYZ xyzSat(stv.second.x);
        double dt = (xyzSat - xyz).norm() / C_MPS;

        double wt(0.0);
        wt = OMEGA_EARTH * dt;

        // todo:
        // Eigen中Vector3d是不是支持坐标旋转？
        // 请查询并修改

        double xSat, ySat, zSat;
        xSat = stv.second.x[0];
        ySat = stv.second.x[1];
        zSat = stv.second.x[2];

        double xSatRot(0.0), ySatRot(0.0);
        xSatRot = +std::cos(wt) * xSat + std::sin(wt) * ySat;
        ySatRot = -std::sin(wt) * xSat + std::cos(wt) * ySat;

        XYZ xyzRecTime;
        xyzRecTime[0] = xSatRot;
        xyzRecTime[1] = ySatRot;
        xyzRecTime[2] = zSat; // z轴不变

        double vxSat, vySat, vzSat;
        vxSat = stv.second.v[0];
        vySat = stv.second.v[1];
        vzSat = stv.second.v[2];

        double vxSatRot(0.0), vySatRot(0.0);
        vxSatRot = +std::cos(wt) * vxSat + std::sin(wt) * vySat;
        vySatRot = -std::sin(wt) * vxSat + std::cos(wt) * vySat;

        XYZ velRecTime;
        velRecTime[0] = vxSatRot;
        velRecTime[1] = vySatRot;
        velRecTime[2] = vzSat; // 不变

        // 替换位置和速度，得到旋转后的卫星产品
        Xvt xvtRecTime = stv.second;
        xvtRecTime.x = xyzRecTime;
        xvtRecTime.v = velRecTime;

        satXvtRecTime[sat] = xvtRecTime;
    };

    return satXvtRecTime;
};

void SPPIFCode::computeElevAzim(Eigen::Vector3d& xyz,
                                std::map<SatID,Xvt> & satXvt,
                                SatValueMap& tempElevData,
                                SatValueMap& tempAzimData
                                 )
{

    for(auto sx: satXvt)
    {
        SatID sat = sx.first;

        XYZ satXYZ = sx.second.x;

        // elevation
        double elev(0.0);
        double azim(0.0);
        elev = elevation(xyz, satXYZ);
        azim = azimuth(xyz, satXYZ);


        tempElevData[sat] = elev;
        tempAzimData[sat] = azim;
    }
};


EquSys SPPIFCode::linearize(Eigen::Vector3d& xyz,
                                  std::map<SatID,Xvt>& satXvtRecTime,
                                  SatValueMap& satElevData,
                                  ObsData &obsData,std::map<SatID,double>tropDelay=std::map<SatID,double>()) {
    EquSys equSysTemp;
    equSysTemp.station = obsData.station;

    // 计算 GLONASS 平均频道号（IFB 基准）
    double glonassK0 = 0.0;
    int nGlo = 0;
    {
        double sumK = 0;
        for (auto& st : obsData.satTypeValueData) {
            if (st.first.system == "R" && pEphStore != NULL) {
                try {
                    NavEphGLONASS e = pEphStore->findGLOEph(st.first, obsData.epoch);
                    sumK += e.freqNum; nGlo++;
                } catch (...) {}
            }
        }
        glonassK0 = nGlo > 0 ? sumK / nGlo : 0.0;
    }

    // 自动收集当前观测中的所有系统
    std::set<std::string> availableSystems;

    for (auto& st : obsData.satTypeValueData) {
        SatID sat = st.first;
        availableSystems.insert(sat.system);
    }


    VariableSet varSetTemp;
    for (auto stv: obsData.satTypeValueData) {
        SatID sat = stv.first;

        double elev = satElevData.at(sat);
        double elevRad = elev*DEG_TO_RAD;

        // 跳过这颗卫星，不形成观测方程和未知参数数据
        if(elev < cutOffElev)
        {
            continue;
        }

        // 这里卫星的位置，应该是地球自转以后的卫星位置
        XYZ satXYZ;
        satXYZ = satXvtRecTime[sat].x;

        // rho
        double rho(0.0);
        rho = ( satXYZ - xyz).norm();
        if(debug)
        {
            cout << "Sat:" << sat << endl;
            cout << "xyz:" << xyz << endl;
            cout << "satXYZ:" << satXYZ << endl;
        }

        // convert unit form second to meter
        double clkBias = satXvtRecTime.at(sat).clkbias * C_MPS;
        double relCorr = satXvtRecTime.at(sat).relcorr * C_MPS;



        double slantTrop(0.0);
        if (tropDelay.count(sat)&&!tropDelay.empty()) {
            slantTrop=tropDelay[sat];
        }
        // to do
        // extract slant trop

        // partials
        Eigen::Vector3d cosines;
        cosines[0] = (xyz.x() - satXYZ[0]) / rho;
        cosines[1] = (xyz.y() - satXYZ[1]) / rho;
        cosines[2] = (xyz.z() - satXYZ[2]) / rho;

        if (debug) {
            // 输出线性化信息（对照课本表 6-6, 6-7）
            double clk = satXvtRecTime.at(sat).clkbias * C_MPS;
            double rel = satXvtRecTime.at(sat).relcorr * C_MPS;
            double obs = 0;
            for (auto& tv : stv.second) {
                if (tv.first.size() >= 2 && tv.first.substr(0,2) == "IF") {
                    obs = tv.second;
                    break;
                }
            }
            double prefit_val = obs - (rho - clk - rel + slantTrop);
            double w = 1.0;
            if (elev < 30) w = sin(elevRad) * sin(elevRad);
            cout << "[" << sat << "] rho=" << fixed << setprecision(3) << rho
                 << " obs=" << obs << " prefit=" << prefit_val
                 << " cos=(" << cosines[0] << "," << cosines[1] << "," << cosines[2] << ")"
                 << " w=" << w
                 << " trop=" << slantTrop
                 << endl;
        }

        // todo
        // 请补充rhoDot，用于后续的单点测速

        // 首先定义所有可能的未知参数
        // 位置参数 3 个（永远共用）
        Variable dx(obsData.station, Parameter::dX);
        Variable dy(obsData.station, Parameter::dY);
        Variable dz(obsData.station, Parameter::dZ);

        // 钟差参数：根据系统自动生成
        std::map<std::string, Variable> clockParams;

        for (auto& sys : availableSystems) {
            if (sys == "G") {
                clockParams[sys] = Variable(obsData.station, Parameter::cdt);
            } else if (sys == "C") {
                clockParams[sys] = Variable(obsData.station, Parameter::cdt_BDS);
            } else if (sys == "E") {
                clockParams[sys] = Variable(obsData.station, Parameter::cdt_GAL);
            } else if (sys == "R") {
                clockParams[sys] = Variable(obsData.station, Parameter::cdt_GLO);
            } else if (sys == "J") {
                clockParams[sys] = Variable(obsData.station, Parameter::cdt_QZS);
            } else if (sys == "I") {
                clockParams[sys] = Variable(obsData.station, Parameter::cdt_IRN);
            }
        }

        // 对每个观测值，都需要存储对应的未知参数及其偏导数
        for (auto tv: stv.second)
        {
            // 通用 IF 组合识别：类型名前缀为 "CC"
            if (tv.first.substr(0, 2) == "CC")
            {
                EquID equID = EquID(sat, tv.first);


                //>> 先验残差
                double prefit;
                double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                prefit = tv.second - computedObs;

                if (debug) {
                    cout<<"prefit:"<<prefit<<endl;
                    cout << "obs:" << tv.second << endl;
                    cout << "rho:" << rho << endl;
                    cout << "clkBias:" << clkBias << endl;
                    cout << "relCorr:" << relCorr << endl;
                    cout << "slantTrop:" << slantTrop << endl;
                    cout<<endl;
                }

                equSysTemp.obsEquData[equID].prefit = prefit;
                equSysTemp.obsEquData[equID].varCoeffData[dx] = cosines[0];
                equSysTemp.obsEquData[equID].varCoeffData[dy] = cosines[1];
                equSysTemp.obsEquData[equID].varCoeffData[dz] = cosines[2];
                SatID sat = stv.first;
                std::string sys = sat.system;

                // 自动给对应系统的钟差赋系数 1.0
                equSysTemp.obsEquData[equID].varCoeffData[ clockParams[sys] ] = 1.0;




                // GLONASS IFB: 估计线性频间偏差 a₁·(k - k₀)
                // k₀ 在 linearize 入口处已计算，a₀ 被钟差吸收
                // 至少 6 颗 GLONASS 卫星以保证自由度 > 0
                if (sys == "R" && pEphStore != NULL && nGlo >= 6) {
                    int freqNum = 0;
                    try {
                        NavEphGLONASS e = pEphStore->findGLOEph(sat, obsData.epoch);
                        freqNum = e.freqNum;
                    } catch (...) {}

                    Variable ifbVar(obsData.station, Parameter::ifb);
                    equSysTemp.obsEquData[equID].varCoeffData[ifbVar] = freqNum - glonassK0;
                    varSetTemp.insert(ifbVar);
                }

                // Compute the weight according to elevation
                double elevWeight;
                if(elev >= 30){
                    elevWeight = 1.0 / (sigIFCode * sigIFCode);
                }
                else
                {
                    elevWeight = 1.0 / (sigIFCode * sigIFCode) * std::pow(std::sin(elevRad), 2);
                }

                // 获取卫星类型权重（仅BDS卫星有效）
                double typeWeight = getTypeWeight(sat, obsData.epoch);
                
                // 综合权重 = 高程权重 * 卫星类型权重
                double weight = elevWeight * typeWeight;

                equSysTemp.obsEquData[equID].weight = weight; // IF组合方差为1.0m

                // 把当前观测方程未知参数插入到总体的未知参数
                varSetTemp.insert(dx);
                varSetTemp.insert(dy);
                varSetTemp.insert(dz);
                varSetTemp.insert(clockParams[sys]);
                equSysTemp.satList.push_back(sat);

                break;
            }
        }
    }

    equSysTemp.varSet = varSetTemp;
    if (debug)
        cout<<equSysTemp.satList.size()<<endl;

    return equSysTemp;
};


void SPPIFCode::correctTGD(ObsData &obsdata) {
    CommonTime epoch = obsdata.epoch;

    for (auto &tv : obsdata.satTypeValueData) {
        SatID sat = tv.first;
        std::string sys = sat.system;

        // 只给 BDS 系统做 TGD 改正，其他系统跳过
        if (sys != "C") {
            continue;
        }

        // 获取 IF 组合类型
        std::pair<string, string> ifPair;
        try {
            ifPair = ifCodeTypes.at(sys);
        } catch (...) {
            continue;
        }

        // 获取频率
        double f1 = getFreq(sys, ifPair.first);
        double f2 = getFreq(sys, ifPair.second);

        // 获取 TGD 参数
        double TGD1 = 0.0, TGD2 = 0.0;
        try {
            NavEphBDS nav_eph_bds = pEphStore->findBDSEph(sat, epoch);
            TGD1 = nav_eph_bds.TGD1;
            TGD2 = nav_eph_bds.TGD2;
        } catch (...) {
            // 找不到星历或 TGD 字段则跳过
            continue;
        }

        // 关键修正：在 IF 组合前对单频观测值分别应用 TGD 改正
        // P1_corr = P1 - c*TGD1
        // P2_corr = P2 - c*TGD2
        auto it1 = tv.second.find(ifPair.first);
        if (it1 != tv.second.end()) {
            double tgdCorr1 = C_MPS * TGD1;
            it1->second -= tgdCorr1;
            if (debug) {
                cout << "[correctTGD] sat:" << sat << " " << ifPair.first 
                     << " TGD1:" << TGD1 << "s (" << tgdCorr1 << "m)" << endl;
            }
        }

        auto it2 = tv.second.find(ifPair.second);
        if (it2 != tv.second.end()) {
            double tgdCorr2 = C_MPS * TGD2;
            it2->second -= tgdCorr2;
            if (debug) {
                cout << "[correctTGD] sat:" << sat << " " << ifPair.second 
                     << " TGD2:" << TGD2 << "s (" << tgdCorr2 << "m)" << endl;
            }
        }
    }
}



std::map<SatID,double> SPPIFCode::computeTropDelay(ObsData &obsdata,std::map<SatID, double>&satElevData) {
    Vector3d xyz=obsdata.antennaPosition;
    std::map<SatID,double>tropDelayMap= tropDelay(xyz,satElevData);
    return tropDelayMap;
}


std::vector<SPPIFResult> SPPIFCode::full_solve(RinexNavStore* pStore, 
                                                std::map<string, std::pair<string, string>> ifCodeTypes,
                                                string roverFile, 
                                                bool TGD_Bool, 
                                                bool Trop_Bool) {
    std::vector<SPPIFResult> results;

    std::fstream roverObsStream(roverFile);
    if (!roverObsStream) {
        cerr << "rover file open error!" << strerror(errno) << endl;
        exit(-1);
    }

    // 使用外部设置的 selectedTypes，如果为空则使用默认配置
    if (this->selectedTypes.empty()) {
        this->selectedTypes["G"].insert("C1W");
        this->selectedTypes["G"].insert("C2W");
        this->selectedTypes["C"].insert("C1X");
        this->selectedTypes["C"].insert("C2I");
        this->selectedTypes["E"].insert("C1X");
        this->selectedTypes["E"].insert("C5X");
        this->selectedTypes["R"].insert("C1C");
        this->selectedTypes["R"].insert("C2C");
    }

    RinexObsReader readObsRover;
    readObsRover.setFileStream(&roverObsStream);
    readObsRover.setSelectedTypes(this->selectedTypes);

    setRinexNavStore(pStore);
    setIFCodeTypes(ifCodeTypes);

    // ENU 坐标固定参考点
    Vector3d refXYZ(-2267750.275, 5009154.471, 3221294.345);
    std::unique_ptr<ReferenceFrame> frame = ReferenceFrameFactory::create("G");

    while (true) {
        ObsData roverData;

        try {
            roverData = readObsRover.parseRinexObs();
        }
        catch (EndOfFile &e) { break; }

        CommonTime epoch = roverData.epoch;
        readObsRover.chooseObs(roverData);

        SatTypeValueMap keep_data;
        // 根据 sysCode 筛选卫星
        if (sysCode.empty()) {
            // 默认模式：处理所有支持的系统
            for (auto &st: roverData.satTypeValueData) {
                if (st.first.system == "C" && pEphStore->bdsEphData.find(st.first) != pEphStore->bdsEphData.end()) {
                    keep_data.insert(st);
                } else if (st.first.system == "G" && pEphStore->gpsEphData.find(st.first) != pEphStore->gpsEphData.end()) {
                    keep_data.insert(st);
                } else if (st.first.system == "E" && pEphStore->galEphData.find(st.first) != pEphStore->galEphData.end()) {
                    keep_data.insert(st);
                } else if (st.first.system == "R" && pEphStore->gloEphData.find(st.first) != pEphStore->gloEphData.end()) {
                    keep_data.insert(st);
                } else if (st.first.system == "J" && pEphStore->qzssEphData.find(st.first) != pEphStore->qzssEphData.end()) {
                    keep_data.insert(st);
                } else if (st.first.system == "I" && pEphStore->irnssEphData.find(st.first) != pEphStore->irnssEphData.end()) {
                    keep_data.insert(st);
                }
            }
        } else {
            // 单系统模式：只处理指定系统
            for (auto &st: roverData.satTypeValueData) {
                if (st.first.system == sysCode) {
                    if (sysCode == "G" && pEphStore->gpsEphData.find(st.first) != pEphStore->gpsEphData.end()) {
                        keep_data.insert(st);
                    } else if (sysCode == "C" && pEphStore->bdsEphData.find(st.first) != pEphStore->bdsEphData.end()) {
                        keep_data.insert(st);
                    } else if (sysCode == "E" && pEphStore->galEphData.find(st.first) != pEphStore->galEphData.end()) {
                        keep_data.insert(st);
                    } else if (sysCode == "R" && pEphStore->gloEphData.find(st.first) != pEphStore->gloEphData.end()) {
                        keep_data.insert(st);
                    } else if (sysCode == "J" && pEphStore->qzssEphData.find(st.first) != pEphStore->qzssEphData.end()) {
                        keep_data.insert(st);
                    } else if (sysCode == "I" && pEphStore->irnssEphData.find(st.first) != pEphStore->irnssEphData.end()) {
                        keep_data.insert(st);
                    }
                }
            }
        }
        roverData.satTypeValueData.swap(keep_data);

        // 最小二乘
        try {
            solve(roverData, TGD_Bool, Trop_Bool);
        }
        catch (...) {
            if(debug) {
                cout << "epoch:" << epoch << endl;
                cout << "too many iterations" << endl;
            }
            continue;
        }

        Vector3d xyzRover = getXYZ();

        // 将 XYZ 转换为 ENU 坐标
        BLH blhRover = xyz2blh(xyzRover, *frame);
        XYZ enuRover = blh2ENU(blhRover, *frame, refXYZ);
        Vector3d enuVec(enuRover.X(), enuRover.Y(), enuRover.Z());

        // 计算统计信息
        int nSat = roverData.satTypeValueData.size();

        // 计算 PDOP
        double pdop = 0.0;
        MatrixXd covMatrix = solverLsq.getCovMatrix();
        double sigma0_val = solverLsq.getSigma0();
        
        if (covMatrix.rows() >= 3 && sigma0_val > 0) {
            double varX = covMatrix(0, 0) * sigma0_val * sigma0_val;
            double varY = covMatrix(1, 1) * sigma0_val * sigma0_val;
            double varZ = covMatrix(2, 2) * sigma0_val * sigma0_val;
            pdop = sqrt(varX + varY + varZ) / sigma0_val;
        }

        // 跳过 sigma0 过大的历元
        if (sigma0_val > 10.0) {
            continue;
        }

        // 检查 PDOP 是否为异常值（NaN、无穷大、负数）
        if (std::isnan(pdop) || std::isinf(pdop) || pdop < 0) {
            if (debug) {
                cout << "[DEBUG full_solve] PDOP is invalid (" << pdop << "), skipping epoch " << CommonTime2YDSTime(epoch) << endl;
            }
            continue;
        }
        
        // PDOP 检测：PDOP > 10 时跳过该历元
        if (pdop > 10.0) {
            if (debug) {
                cout << "[DEBUG full_solve] PDOP " << pdop << " > 10, skipping epoch " << CommonTime2YDSTime(epoch) << endl;
            }
            continue;
        }

        // 计算残差统计
        VectorXd residuals = solverLsq.getResiduals();
        double meanResidual = residuals.mean();
        double rmsResidual = sqrt(residuals.squaredNorm() / residuals.size());
        double maxResidual = residuals.cwiseAbs().maxCoeff();

        // 计算 TGD 改正值统计
        double meanTGD = 0.0, maxTGD = -1e9, minTGD = 1e9;
        std::map<SatID, double> currentTGDData;

        for (auto &tgd : satTGDData) {
            currentTGDData[tgd.first] = tgd.second;
            meanTGD += tgd.second;
            maxTGD = std::max(maxTGD, tgd.second);
            minTGD = std::min(minTGD, tgd.second);
        }
        if (!currentTGDData.empty()) {
            meanTGD /= currentTGDData.size();
        }

        if(debug)
            cout << "sppif:" << CommonTime2YDSTime(epoch) << " XYZ:" << xyzRover.transpose() 
                 << " ENU:" << enuVec.transpose() << " PDOP:" << pdop 
                 << " NSAT:" << nSat << " Sigma0:" << sigma0_val << endl;

        // 填充结果结构体
        SPPIFResult result;
        result.epoch = epoch;
        result.ydsTime = CommonTime2YDSTime(epoch);  // 使用 YDSTime 格式，与 exam5.3 一致
        result.xyz = xyzRover;
        result.enu = enuVec;
        result.pdop = pdop;
        result.nSat = nSat;
        result.sigma0 = sigma0_val;
        result.meanResidual = meanResidual;
        result.rmsResidual = rmsResidual;
        result.maxResidual = maxResidual;
        result.meanTGD = meanTGD;
        result.maxTGD = maxTGD;
        result.minTGD = minTGD;
        result.satTGDData = currentTGDData;

        results.push_back(result);
    }

    roverObsStream.close();
    return results;
}
bool SPPIFCode::strangeDataDelete(ObsData &obsData,double parameter) {
    VectorXd residuals = solverLsq.getResiduals();
    double sigma0 = solverLsq.getSigma0();
    
    // 计算自适应阈值（sigma0 * parameter）
    double adaptiveThreshold = parameter * sigma0;
    
    // 设置硬阈值上限（单位：米），防止 sigma0 过大时阈值失效
    const double MAX_THRESHOLD = 5;  // 最大阈值不超过 5 米
    double threshold = std::min(adaptiveThreshold, MAX_THRESHOLD);
    
    if(debug)
    {
        cout << "Residuals:" << endl;
        cout << residuals << endl;

        cout << "Sigma0:" << sigma0 << endl;
        cout << "Adaptive threshold:" << adaptiveThreshold << endl;
        cout << "Final threshold (with max limit):" << threshold << endl;
    }
    double maxv = 0.0;
    int badIndex = -1;

    for(int i=0;i<residuals.size();i++)
    {
        if(abs(residuals(i)) > maxv)
        {
            maxv = abs(residuals(i));
            badIndex = i;
        }
    }
    if(maxv > threshold || maxv > 100.0){

        SatID badSat = equSys.satList[badIndex];

        if (debug) {
            cout<<"remove outlier: "
                <<badSat
                <<" residual="
                <<maxv<<endl;
        }

        // 超大残差（>100m）：直接删星
        if (maxv > 100.0) {
            obsData.satTypeValueData.erase(badSat);
            if (debug) cout << "  超大残差，直接删星: " << badSat << " (" << maxv << "m)" << endl;
            return true;
        }

        // 中等残差：加入降权集合
        if (outlierSats.find(badSat) != outlierSats.end()) {
            if (debug) cout << "  卫星已在降权集合，跳过" << endl;
            return false;
        }
        outlierSats.insert(badSat);
        if (debug) cout << "  加入降权集合: " << badSat << endl;
        return true;

    }
    else{
        return false;
    }
}

// BDS卫星类型判断（基于轨道参数）
std::string SPPIFCode::getBDSSatType(const SatID& sat, CommonTime epoch)
{
    if (sat.system != "C") {
        if (debug) {
            cout << "[getBDSSatType] Sat " << sat << " is not BDS, return OTHER" << endl;
        }
        return "OTHER";
    }

    if (pEphStore == nullptr) {
        if (debug) {
            cout << "[getBDSSatType] Sat " << sat << " pEphStore is null, return MEO" << endl;
        }
        return "MEO";
    }

    try {
        NavEphBDS eph = pEphStore->findBDSEph(sat, epoch);

        // ===== 1. 半长轴 =====
        double a = eph.sqrt_A * eph.sqrt_A;

        // ===== 2. 倾角（rad）=====
        double inc = std::fabs(eph.i0) * std::numbers::pi;
        
        if (debug) {
            cout << "[getBDSSatType] Sat " << sat << ":" << endl;
            cout << "  sqrt_A: " << eph.sqrt_A << endl;
            cout << "  a (semi-major axis): " << a << " m" << endl;
            cout << "  i0 (semicircles): " << eph.i0 << endl;
            cout << "  inc (rad): " << inc << " (" << inc * 180 / std::numbers::pi << " deg)" << endl;
        }

        // ===== 3. 第一层：MEO vs 同步轨道 =====
        constexpr double MEO_MAX_A = 3.0e7;      // ~30000 km
        constexpr double GEO_MIN_A = 4.10e7;     // 同步轨道下界
        constexpr double GEO_MAX_A = 4.30e7;     // 同步轨道上界

        if (a < MEO_MAX_A) {
            if (debug) {
                cout << "  a(" << a << ") < MEO_MAX_A(" << MEO_MAX_A << "), return MEO" << endl;
            }
            return "MEO";
        }

        // ===== 4. 同步轨道候选 =====
        if (a >= GEO_MIN_A && a <= GEO_MAX_A)
        {
            // ===== GEO / IGSO 分界 =====
            // GEO：低倾角
            // IGSO：高倾角（约 55°）

            constexpr double GEO_INC_THRESHOLD = 0.3;  // ~17°

            if (inc < GEO_INC_THRESHOLD) {
                if (debug) {
                    cout << "  a in GEO range, inc(" << inc << ") < threshold(" << GEO_INC_THRESHOLD << "), return GEO" << endl;
                }
                return "GEO";
            } else {
                if (debug) {
                    cout << "  a in GEO range, inc(" << inc << ") >= threshold(" << GEO_INC_THRESHOLD << "), return IGSO" << endl;
                }
                return "IGSO";
            }
        }

        // ===== 5. 其他异常 =====
        if (debug) {
            cout << "  a(" << a << ") out of range, return MEO (default)" << endl;
        }
        return "MEO";

    } catch (const std::exception& e) {
        if (debug) {
            cout << "[getBDSSatType] Sat " << sat << " exception: " << e.what() << ", return MEO" << endl;
        }
        return "MEO";
    } catch (...) {
        if (debug) {
            cout << "[getBDSSatType] Sat " << sat << " unknown exception, return MEO" << endl;
        }
        return "MEO";
    }
}

// 根据卫星类型获取权重
double SPPIFCode::getTypeWeight(const SatID& sat, CommonTime epoch)
{
    if (sat.system != "C") {
        return 1.0;  // 非BDS卫星权重为1.0
    }

    std::string satType = getBDSSatType(sat, epoch);
    if (debug) {
        cout<<"epoch:"<<epoch<<endl;
        cout<<"satID:"<<sat<<"  "<<satType<<endl;
    }
    if (satType == "MEO") {
        return meoWeight;
    } else if (satType == "IGSO") {
        return igsoWeight;
    } else if (satType == "GEO") {
        return geoWeight;
    } else {
        return meoWeight;
    }
}
