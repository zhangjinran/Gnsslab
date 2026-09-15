//
// Created by zhang on 2026/5/5.
//
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

#include <gnsslab/SPPCode.h>
#include <gnsslab/CoordConvert.h>
#include <iostream>
#include <numbers>
#include <gnsslab/RinexObsReader.h>
#include <Eigen/Eigen>


#include <gnsslab/GnssFunc.h>

#define debug 0

void SPPCode::solve(ObsData &obsData,bool TGD_bool,bool Trop_Bool,bool Iono_Bool) {
    outlierSats.clear();
    if(debug) {
        cout << "\n" << string(70, '=') << endl;
        cout << "=== SPPCode::solve() - SPP单点定位求解 ===" << endl;
        cout << "历元: " << CommonTime2CivilTime(obsData.epoch) << endl;
        cout << string(70, '-') << endl;
    }

    convertObsType(obsData);

    if(debug) {
        cout << "\n【1. 观测值类型转换】" << endl;
        cout << "转换后观测卫星数: " << obsData.satTypeValueData.size() << endl;
        cout << "卫星列表: " << endl;
        for (auto &st : obsData.satTypeValueData) {
            cout << st.first << " " << st.second << endl;
        }
        cout << endl;
    }

    // 计算发射时刻卫星位置
    satXvtTransTime = computeSatPos(obsData);
    if(debug) {
        cout << "\n【2. 卫星发射时刻位置计算】" << endl;
        cout << "有效卫星数: " << satXvtTransTime.size() << endl;
        cout << string(40, '-') << endl;
        for(auto sx: satXvtTransTime) {
            cout << "卫星: " << sx.first << endl;
            cout << "  位置 (m): " << sx.second.x.transpose() << endl;
            cout << "  速度 (m/s): " << sx.second.v.transpose() << endl;
            cout << "  钟差 (m): " << sx.second.clkbias * C_MPS << endl;
            cout << "  相对论改正 (m): " << sx.second.relcorr * C_MPS << endl;
        }
        cout << string(40, '-') << endl;
    }

    // TGD改正
    if (TGD_bool) {
        correctTGD(obsData);
        if(debug) {
            cout << "\n【3. TGD改正】已应用" << endl;
        }
    }

    xyz = obsData.antennaPosition;
    dxyz = {100, 100, 100};
    if (debug) {
        cout << "\n【4. 初始位置设置】" << endl;
        cout << "初始天线位置 (XYZ): " << fixed << setprecision(3) << xyz.transpose() << " m" << endl;
        cout << "初始修正量 (dXYZ): " << dxyz.transpose() << " m" << endl;
    }


    int iter(0);
    while (true) {
        // 根据开关控制是否应用地球自转改正
        if (earthRotationEnable) {
            satXvtRecTime = earthRotation(xyz, satXvtTransTime);
            
            if(debug) {
                cout << "\n【迭代 " << iter << "】地球自转改正后卫星位置" << endl;
                cout << string(40, '-') << endl;
                for(auto sx: satXvtRecTime) {
                    cout << "卫星: " << sx.first << endl;
                    cout << "  位置 (m): " << sx.second.x.transpose() << endl;
                }
            }
        } else {
            // 不进行地球自转改正，直接使用发射时刻的卫星位置
            satXvtRecTime = satXvtTransTime;
            
            if(debug) {
                cout << "\n【迭代 " << iter << "】跳过地球自转改正" << endl;
            }
        }

        int numSats = obsData.satTypeValueData.size();

        if (numSats < 5) {
            epochSkipStats.svNumException++;
            SVNumException e("num of satellites is less than 4");
            throw(e);
        }

        std::map<SatID,double> tropdelaymap;
        if(std::abs(xyz.norm() - RadiusEarth) < 100000.0)
        {
            satElevData.clear();
            satAzimData.clear();
            computeElevAzim(xyz, satXvtRecTime, satElevData, satAzimData);

            if(debug) {
                cout << "\n【迭代 " << iter << "】卫星高度角和方位角" << endl;
                cout << string(40, '-') << endl;
                for (auto &sd : satElevData) {
                    cout << "卫星: " << sd.first 
                         << "  高度角: " << fixed << setprecision(2) << sd.second << "°"
                         << "  方位角: " << fixed << setprecision(2) << satAzimData[sd.first] << "°" << endl;
                }
            }

            if (Iono_Bool) {
                satIonoData.clear();
                satIonoData = ionoDelay(xyz, obsData.epoch, satElevData, satAzimData, *pEphStore);
                if(debug) {
                    cout << "\n【迭代 " << iter << "】电离层延迟" << endl;
                    for (auto &id : satIonoData) {
                        cout << "卫星: " << id.first << "  延迟: " << fixed << setprecision(3) << id.second << " m" << endl;
                    }
                }
            }
            
            if (Trop_Bool) {
                tropdelaymap = computeTropDelay(obsData, satElevData);
                if(debug) {
                    cout << "\n【迭代 " << iter << "】对流层延迟" << endl;
                    for (auto &td : tropdelaymap) {
                        cout << "卫星: " << td.first << "  延迟: " << fixed << setprecision(3) << td.second << " m" << endl;
                    }
                }
            }
        }

        equSys = linearize(xyz, satXvtRecTime, satElevData, obsData, tropdelaymap);

        if(debug) {
            cout << "\n【迭代 " << iter << "】线性化完成" << endl;
            cout << "观测方程数: " << equSys.obsEquData.size() << endl;
            cout << "未知参数数: " << equSys.varSet.size() << endl;
        }

        if(!isRover) {
            if(debug) cout << "\n【基准站模式】线性化完成，退出迭代" << endl;
            break;
        }

        solverLsq.solve(equSys);
        dxyz = solverLsq.getdxyz();

        // PDOP 判定
        double pdop = 0.0;
        MatrixXd covMatrix = solverLsq.getCovMatrix();
        if (covMatrix.rows() >= 3) {
            double sigma0 = solverLsq.getSigma0();
            if (sigma0 > 0) {
                double varX = covMatrix(0, 0) * sigma0 * sigma0;
                double varY = covMatrix(1, 1) * sigma0 * sigma0;
                double varZ = covMatrix(2, 2) * sigma0 * sigma0;
                pdop = sqrt(varX + varY + varZ) / sigma0;
                if (debug) {
                    cout << "varX: " << varX << endl;
                    cout << "varY: " << varY << endl;
                    cout << "varZ: " << varZ << endl;
                    cout << "pdop: " << pdop << endl;
                }

            }
        }
        CommonTime epoch(60676,20910);
        if (epoch==obsData.epoch&&debug)
            cout<<"epoch: "<<epoch<<endl;
        // 检查 PDOP 是否为异常值（NaN、无穷大、负数）
        if (std::isnan(pdop) || std::isinf(pdop) || pdop < 0) {
            epochSkipStats.pdopInvalid++;
            if (debug) {
                cout << "\n【PDOP异常】PDOP = " << pdop << "，跳过该历元" << endl;
            }
            throw std::runtime_error("PDOP is invalid");
        }
        
        if (pdop > 10.0) {
            epochSkipStats.pdopExceed++;
            if (debug) {
                cout << "\n【PDOP超限】PDOP = " << fixed << setprecision(2) << pdop << " > 10.0，跳过该历元" << endl;
            }
            throw std::runtime_error("PDOP exceeds threshold");
        }

        xyz += dxyz;
        if (debug) {
            cout << "\n【迭代 " << iter << "】求解结果" << endl;
            cout << string(40, '-') << endl;
            cout << "dXYZ (m): " << fixed << setprecision(6) << dxyz.transpose() << endl;
            cout << "XYZ (m):   " << fixed << setprecision(3) << xyz.transpose() << endl;
            cout << "delta_norm: " << fixed << setprecision(6) << dxyz.norm() << " m" << endl;
            cout << "PDOP: " << fixed << setprecision(2) << pdop << endl;
            cout << "sigma0: " << fixed << setprecision(3) << solverLsq.getSigma0() << " m" << endl;
        }

        // 根据 sigma0 动态设置粗差探测参数
        double sigma0 = solverLsq.getSigma0();
        double rejectThreshold = 0;
        if (sigma0 < 3) {
            rejectThreshold = 4;
            // sigma0 < 3，不需要粗差探测
        } else {
            rejectThreshold = 2.5;
        }

        if ( strangeDataDelete(obsData, rejectThreshold)) {
            epochSkipStats.satOutlierDeleted++;
            if(debug) cout << "\n【粗差剔除】发现异常数据，重新初始化迭代 (sigma0=" << sigma0 << ", threshold=" << rejectThreshold << ")" << endl;
            iter = 0;
            xyz = obsData.antennaPosition;
            dxyz = {100, 100, 100};
            continue;
        }

        if (dxyz.norm() < 0.001) {
            if(debug) cout << "\n【收敛判定】delta_norm = " << dxyz.norm() << " < 0.001m，迭代收敛" << endl;
            break;
        }

        if (iter > 10) {
            epochSkipStats.iterNotConverge++;
            InvalidSolver e("too many iterations");
            throw(e);
        }
        iter++;

    }

    if(debug) {
        cout << "\n【SPP求解完成】" << endl;
        cout << string(40, '-') << endl;
        cout << "最终位置 (XYZ): " << fixed << setprecision(3) << xyz.transpose() << " m" << endl;
        cout << string(70, '=') << endl;
    }

    result.xyz = xyz;
}


std::map<SatID, Xvt> SPPCode::computeSatPos(ObsData &obsData) {
    std::map<SatID, Xvt> satXvtData;
    SatIDSet satRejectedSet;
    CommonTime time = obsData.epoch;
    // Loop through all the satellites
    for (auto stv: obsData.satTypeValueData) {
        SatID sat(stv.first);
        
        // 调试输出：测试 SatID 的 id 字段
        if (debug) {
            cout << "  [DEBUG] 卫星: " << sat 
                 << "  system: " << sat.system 
                 << "  id(PRN): " << sat.id 
                 << "  generation: " << sat.generation << endl;
        }
        
        Xvt xvt;
        // compute satellite ephemeris at transmitting time
        // Scalar to hold temporal value
        double obs(0.0);
        string codeType;
        auto sysIt = sysTypes.find(sat.system);
        if (sysIt != sysTypes.end() && !sysIt->second.empty()) {
            codeType = *sysIt->second.begin();
        }
        else {
            epochSkipStats.satNoCodeType++;
            satRejectedSet.insert(sat);
            continue;
        }

        try {
            obs = stv.second.at(codeType);
            if(debug) {
                cout << "  卫星: " << sat 
                     << "  观测值类型: " << codeType 
                     << "  伪距: " << fixed << setprecision(3) << obs << " m" << endl;
            }
        }
        catch (...) {
            epochSkipStats.satNoObsValue++;
            satRejectedSet.insert(sat);
            continue;
        }

        // now, compute xvt
        try {
            xvt = computeAtTransmitTime(time, obs, sat);
        }
        catch (InvalidRequest &e) {
            epochSkipStats.satEphFailed++;
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

Xvt SPPCode::computeAtTransmitTime(const CommonTime &tr,
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
        // 根据开关控制是否应用相对论效应改正
        if (relativityEnable) {
            if (debug) {
                cout << "[DEBUG computeAtTransmitTime] 卫星: " << sat 
                     << " 迭代: " << i 
                     << " clkbias: " << fixed << setprecision(12) << xvt.clkbias << " s"
                     << " relcorr: " << fixed << setprecision(12) << xvt.relcorr << " s"
                     << " (相对论效应已启用)" << endl;
            }
            tt -= (xvt.clkbias + xvt.relcorr);
        } else {
            if (debug) {
                cout << "[DEBUG computeAtTransmitTime] 卫星: " << sat 
                     << " 迭代: " << i 
                     << " clkbias: " << fixed << setprecision(12) << xvt.clkbias << " s"
                     << " relcorr: " << fixed << setprecision(12) << xvt.relcorr << " s"
                     << " (相对论效应已禁用，跳过relcorr)" << endl;
            }
            tt -= xvt.clkbias;  // 不应用相对论效应改正
        }

    }
    return xvt;
};

void SPPCode::convertObsType(ObsData &obsData) {

    SatTypeValueMap stvData;
    for (auto sd: obsData.satTypeValueData) {
        TypeValueMap tvData;
        for (auto td: sd.second) {
            tvData[td.first.substr(0, 2)] = td.second;
        }
        stvData[sd.first] = tvData;
    }
    std::map<string, std::set<string>> tempsSysTypes;
    for (auto stv: sysTypes) {
        std::set<string> sysTypeTemp;
        for (auto sysType: stv.second) {
            sysTypeTemp.insert(sysType.substr(0, 2));
        }
        tempsSysTypes[stv.first] = sysTypeTemp;
    }



    // 替代
    sysTypes = tempsSysTypes;
    obsData.satTypeValueData = stvData;
};



std::map<SatID, Xvt> SPPCode::earthRotation(Eigen::Vector3d &xyz,
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

void SPPCode::computeElevAzim(Eigen::Vector3d& xyz,
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


EquSys SPPCode::linearize(Eigen::Vector3d& xyz,
                                  std::map<SatID,Xvt>& satXvtRecTime,
                                  SatValueMap& satElevData,
                                  ObsData &obsData,std::map<SatID,double>tropDelay=std::map<SatID,double>()) {
    EquSys equSysTemp;
    equSysTemp.station = obsData.station;
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
        rho = (satXYZ - xyz).norm();
        if(debug) {
            cout << "\n【卫星 " << sat << "】" << endl;
            cout << "  接收机位置 (XYZ): " << fixed << setprecision(3) << xyz.transpose() << " m" << endl;
            cout << "  卫星位置 (XYZ):   " << fixed << setprecision(3) << satXYZ.transpose() << " m" << endl;
            cout << "  几何距离 (rho):   " << fixed << setprecision(3) << rho << " m" << endl;
        }

        // convert unit form second to meter
        double clkBias = satXvtRecTime.at(sat).clkbias * C_MPS;
        double relCorr = satXvtRecTime.at(sat).relcorr * C_MPS;
        
        // 存储相对论效应改正值（单位：米）
        satRelativityData[sat] = relCorr;



        double slantTrop(0.0);
        if (tropDelay.count(sat)&&!tropDelay.empty()) {
            slantTrop=tropDelay[sat];
        }

        // 电离层延迟
        double slantIono(0.0);
        if (satIonoData.count(sat) && !satIonoData.empty()) {
            slantIono = satIonoData[sat];
        }

        // partials
        Eigen::Vector3d cosines;
        cosines[0] = (xyz.x() - satXYZ[0]) / rho;
        cosines[1] = (xyz.y() - satXYZ[1]) / rho;
        cosines[2] = (xyz.z() - satXYZ[2]) / rho;

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
            auto sysIt = sysTypes.find(sat.system);
            if (sysIt != sysTypes.end() && sysIt->second.count(tv.first))
            {

                EquID equID = EquID(sat, tv.first);


                // 根据开关控制是否应用相对论效应改正
                double computedObs = (rho - clkBias + slantTrop + slantIono);
                if (relativityEnable) {
                    computedObs -= relCorr;
                }
                double prefit = tv.second - computedObs;

                if (debug) {
                    cout << "  观测值 (" << tv.first << "): " << fixed << setprecision(3) << tv.second << " m" << endl;
                    cout << "  计算观测值:   " << fixed << setprecision(3) << computedObs << " m" << endl;
                    cout << "  先验残差:     " << fixed << setprecision(3) << prefit << " m" << endl;
                    cout << "  各项改正:" << endl;
                    cout << "    - 钟差改正:      " << fixed << setprecision(3) << -clkBias << " m" << endl;
                    if (relativityEnable) {
                        cout << "    - 相对论改正:    " << fixed << setprecision(3) << -relCorr << " m" << endl;
                    } else {
                        cout << "    - 相对论改正:    " << fixed << setprecision(3) << -relCorr << " m (已禁用)" << endl;
                    }
                    cout << "    - 对流层延迟:    " << fixed << setprecision(3) << slantTrop << " m" << endl;
                    cout << "    - 电离层延迟:    " << fixed << setprecision(3) << slantIono << " m" << endl;
                }

                equSysTemp.obsEquData[equID].prefit = prefit;
                equSysTemp.obsEquData[equID].varCoeffData[dx] = cosines[0];
                equSysTemp.obsEquData[equID].varCoeffData[dy] = cosines[1];
                equSysTemp.obsEquData[equID].varCoeffData[dz] = cosines[2];
                SatID sat = stv.first;
                std::string sys = sat.system;

                // 自动给对应系统的钟差赋系数 1.0
                equSysTemp.obsEquData[equID].varCoeffData[ clockParams[sys] ] = 1.0;




                // Compute the weight according to elevation
                double w_elev;
                if(elev >= 30){
                    w_elev = 1.0 / (sigCode * sigCode);
                }
                else
                {
                    w_elev = 1.0 / (sigCode * sigCode) * std::pow(std::sin(elevRad), 2);
                }

                // BDS卫星类型权重
                double w_type = getTypeWeight(sat,obsData.epoch);

                // 最终权重 = 仰角权重 * 类型权重
                double weight = w_elev * w_type;

                // 粗差降权（不删星，仅设极小的权）
                if (outlierSats.find(sat) != outlierSats.end()) {
                    weight = 1e-10;
                }

                equSysTemp.obsEquData[equID].weight = weight;

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

    return equSysTemp;
};


void SPPCode::correctTGD( ObsData &obsdata) {
    // 清空上一历元的 TGD 数据
    satTGDData.clear();

    CommonTime epoch=obsdata.epoch;
    for (auto &tv: obsdata.satTypeValueData) {
        if (tv.first.system == "G")
        {
            // 获取该系统的观测类型集合
            auto sysIt = sysTypes.find(tv.first.system);
            string obstype;
            if (sysIt != sysTypes.end() && !sysIt->second.empty()) {
                obstype = *sysIt->second.begin();
            } else {
                obstype = "C1"; // 默认值
            }
            for (auto &st: tv.second) {
                double Delta_TGD(0.0);
                NavEphGPS nav_eph_gps=pEphStore->findGPSEph(tv.first,epoch);
                double TGD=nav_eph_gps.TGD;
                if (st.first == "C1") {
                    Delta_TGD = -C_MPS * TGD;
                    st.second += Delta_TGD;
                    satTGDData[tv.first] = Delta_TGD;  // 存储 TGD 改正值
                    if(debug) {
                        cout << "  卫星: " << tv.first 
                             << "  TGD参数: " << fixed << setprecision(6) << TGD << " s"
                             << "  改正量: " << fixed << setprecision(3) << Delta_TGD << " m" << endl;
                    }
                    break;
                }
            }

        }
        else if (tv.first.system == "C")
        {
            NavEphBDS nav_eph_bds = pEphStore->findBDSEph(tv.first, epoch);
            for (auto &st: tv.second) {
                double Delta_TGD(0.0);
                if (st.first == "C1") {
                    // BDS C1（B1C）相对于 B3I 的 TGD 需要查 ICD
                    // 当前暂改为 0，后续如有 B1C TGD 参数再补充
                    satTGDData[tv.first] = 0.0;
                    if(debug) {
                        cout << "  卫星: " << tv.first 
                             << "  改正量: " << fixed << setprecision(3) << Delta_TGD << " m (BDS C1)" << endl;
                    }
                }
                else if (st.first == "C2") {
                    // BDS C2（B1I = C2I）：δt_B1 = δt_B3 - TGD1
                    Delta_TGD = -C_MPS * nav_eph_bds.TGD1;
                    st.second += Delta_TGD;
                    satTGDData[tv.first] = Delta_TGD;
                    if(debug) {
                        cout << "  卫星: " << tv.first 
                             << "  TGD1: " << scientific << setprecision(6) << nav_eph_bds.TGD1 << " s"
                             << "  改正量: " << fixed << setprecision(3) << Delta_TGD << " m (BDS C2)" << endl;
                    }
                }
            }
        }
        else if (tv.first.system == "E")
        {
            for (auto &st: tv.second) {
                double Delta_TGD(0.0);
                NavEphGalileo nav_eph_gal=pEphStore->findGalileoEph(tv.first,epoch);
                double BGD_E5aE1=nav_eph_gal.BGD_E5aE1;
                if (st.first == "C1") {
                    Delta_TGD = -C_MPS * BGD_E5aE1;
                    st.second += Delta_TGD;
                    satTGDData[tv.first] = Delta_TGD;  // 存储 TGD 改正值
                    if(debug) {
                        cout << "  卫星: " << tv.first 
                             << "  BGD(E5a,E1): " << fixed << setprecision(6) << BGD_E5aE1 << " s"
                             << "  改正量: " << fixed << setprecision(3) << Delta_TGD << " m" << endl;
                    }
                    break;
                }
            }

        }
        // GLONASS 没有 TGD，存储 0
        else if (tv.first.system == "R") {
            satTGDData[tv.first] = 0.0;
        }
    }

}

std::map<SatID,double> SPPCode::computeTropDelay(ObsData &obsdata,std::map<SatID, double>&satElevData) {
    Vector3d xyz=obsdata.antennaPosition;
    std::map<SatID,double>tropDelayMap= tropDelay(xyz,satElevData);
    return tropDelayMap;
}


EquSys SPPCode::linearizeVelocity(ObsData &obsData, VariableSet &varSet, int &nSat) {
    EquSys velEquSys;
    velEquSys.station = obsData.station;
    nSat = 0;

    if (debug) {
        cout << "\n" << string(70, '=') << endl;
        cout << "=== SPPCode::linearizeVelocity() ===" << endl;
        cout << "历元: " << CommonTime2CivilTime(obsData.epoch) << endl;
        cout << string(70, '-') << endl;
    }

    const string DOPPLER_PREFIX = "D";

    for (auto &stv : obsData.satTypeValueData) {
        SatID sat = stv.first;
        double elev = satElevData.at(sat);
        if (elev < cutOffElev) continue;
        double elevRad = elev * DEG_TO_RAD;

        XYZ satXYZ = satXvtRecTime[sat].x;
        double rho = (satXYZ - xyz).norm();

        Eigen::Vector3d e;
        e[0] = (xyz.x() - satXYZ[0]) / rho;
        e[1] = (xyz.y() - satXYZ[1]) / rho;
        e[2] = (xyz.z() - satXYZ[2]) / rho;

        Eigen::Vector3d satVel = satXvtRecTime[sat].v;
        double satClkDrift = satXvtRecTime[sat].clkdrift;

        for (auto &tv : stv.second) {
            string obsType = tv.first;
            if (obsType.size() < 2 || obsType[0] != DOPPLER_PREFIX[0]) continue;

            int freqNum = 0;
            try { freqNum = stoi(obsType.substr(1, 1)); }
            catch (...) { continue; }

            // GLONASS 频率依赖卫星频道号，需从星历获取
            double wavelength;
            if (sat.system == "R" && pEphStore != nullptr) {
                try {
                    NavEphGLONASS gloEph = pEphStore->findGLOEph(sat, obsData.epoch);
                    string freqType = "C" + to_string(freqNum);
                    double freq = gloEph.getFreq(freqType);
                    wavelength = C_MPS / freq;
                } catch (...) {
                    wavelength = getWavelength(sat.system, freqNum);
                }
            } else {
                wavelength = getWavelength(sat.system, freqNum);
            }
            if (wavelength == 0.0) continue;

            double dopplerHz = tv.second;
            double rangeRate = -wavelength * dopplerHz;
            // l_rs = -λ·D + cosines·Ẋ^s + c·δṫ_s
            // 其中 cosines = (X_r - X^s)/ρ = -e_教科书
            double prefit = rangeRate + e.dot(satVel) + C_MPS * satClkDrift;

            if (debug) {
                cout << "  " << sat << " D" << freqNum
                     << "  λ=" << scientific << setprecision(6) << wavelength
                     << " λ·D=" << fixed << setprecision(1) << -rangeRate
                     << " l_rs=" << setprecision(3) << prefit << " m/s"
                     << endl;
            }

            EquID equID(sat, obsType);
            velEquSys.obsEquData[equID].prefit = prefit;
            velEquSys.obsEquData[equID].varCoeffData[Variable(obsData.station, Parameter::dVx)]     = e[0];
            velEquSys.obsEquData[equID].varCoeffData[Variable(obsData.station, Parameter::dVy)]     = e[1];
            velEquSys.obsEquData[equID].varCoeffData[Variable(obsData.station, Parameter::dVz)]     = e[2];
            velEquSys.obsEquData[equID].varCoeffData[Variable(obsData.station, Parameter::cdt_dot)] =  1.0;

            double w = 1.0 / (sigCode * sigCode);
            if (elev < 30) w *= pow(sin(elevRad), 2);
            velEquSys.obsEquData[equID].weight = w;

            varSet.insert(Variable(obsData.station, Parameter::dVx));
            varSet.insert(Variable(obsData.station, Parameter::dVy));
            varSet.insert(Variable(obsData.station, Parameter::dVz));
            varSet.insert(Variable(obsData.station, Parameter::cdt_dot));
            velEquSys.satList.push_back(sat);
            nSat++;
            break;
        }
    }

    velEquSys.varSet = varSet;
    return velEquSys;
}


SPPVelocityResult SPPCode::solveVelocity(ObsData &obsData) {
    SPPVelocityResult result;
    result.vel = Vector3d(0, 0, 0);
    result.cdt_dot = 0;
    result.vdop = 0;
    result.nSat = 0;

    VariableSet varSet;
    EquSys velEquSys = linearizeVelocity(obsData, varSet, result.nSat);

    if (result.nSat < 4) {
        if (debug) cout << "[solveVelocity] Doppler卫星数不足: " << result.nSat << " < 4" << endl;
        return result;
    }

    SolverLSQ velSolver;
    velSolver.solveGeneral(velEquSys);

    result.vel[0] = velSolver.getSolution(Parameter::dVx, varSet, velSolver.getState());
    result.vel[1] = velSolver.getSolution(Parameter::dVy, varSet, velSolver.getState());
    result.vel[2] = velSolver.getSolution(Parameter::dVz, varSet, velSolver.getState());
    result.cdt_dot = velSolver.getSolution(Parameter::cdt_dot, varSet, velSolver.getState());

    MatrixXd cov = velSolver.getCovMatrix();
    if (cov.rows() >= 3) {
        double sigma0 = velSolver.getSigma0();
        if (sigma0 > 0) {
            result.vdop = sqrt(cov(0,0)*sigma0*sigma0 + cov(1,1)*sigma0*sigma0
                             + cov(2,2)*sigma0*sigma0) / sigma0;
        }
    }

    if (debug) {
        cout << "Vx=" << fixed << setprecision(3) << result.vel[0]
             << " Vy=" << result.vel[1] << " Vz=" << result.vel[2]
             << " cdt_dot=" << result.cdt_dot
             << " VDOP=" << setprecision(2) << result.vdop
             << " nSat=" << result.nSat << endl;
    }

    return result;
}


std::vector<SPPResult> SPPCode::full_solve(RinexNavStore* pStore, string roverFile,std::map<string, std::set<string>> sysType, bool TGD_Bool, bool Trop_Bool, bool Iono_Bool) {
    std::vector<SPPResult> results;

    std::fstream roverObsStream(roverFile);
    if (!roverObsStream) {
        cerr << "rover file open error!" << strerror(errno) << endl;
        exit(-1);
    }
    RinexObsReader readObsRover;
    readObsRover.setFileStream(&roverObsStream);
    readObsRover.setSelectedTypes(sysType);
    sysTypes=sysType;

    setRinexNavStore(pStore);

    // ENU 坐标固定参考点
    Vector3d refXYZ(-2267750.275, 5009154.471, 3221294.345);
    // GPS 系统的参考框架（WGS84）
    std::unique_ptr<ReferenceFrame> frame = ReferenceFrameFactory::create("G");

    while (true) {
        // solve spp for rover
        ObsData roverData;

        try {
            roverData = readObsRover.parseRinexObs();
        }
        catch (EndOfFile &e) { break; }
        readObsRover.chooseObs(roverData);
        if (debug) {
            cout <<"roverData: "<< roverData<< endl;
        }

        CommonTime epoch = roverData.epoch;
        SatTypeValueMap keep_data;
        
        // 根据 sysCode / sysTypes 筛选卫星
        if (sysCode.empty()) {
            // 多系统模式：遍历 sysTypes 中的所有系统
            for (auto &st: roverData.satTypeValueData) {
                string sys = st.first.system;
                if (sysTypes.find(sys) == sysTypes.end()) continue;
                bool hasEph = false;
                if (sys == "G") hasEph = pEphStore->gpsEphData.find(st.first) != pEphStore->gpsEphData.end();
                else if (sys == "C") hasEph = pEphStore->bdsEphData.find(st.first) != pEphStore->bdsEphData.end();
                else if (sys == "E") hasEph = pEphStore->galEphData.find(st.first) != pEphStore->galEphData.end();
                else if (sys == "R") hasEph = pEphStore->gloEphData.find(st.first) != pEphStore->gloEphData.end();
                else if (sys == "J") hasEph = pEphStore->qzssEphData.find(st.first) != pEphStore->qzssEphData.end();
                else if (sys == "I") hasEph = pEphStore->irnssEphData.find(st.first) != pEphStore->irnssEphData.end();
                if (hasEph) keep_data.insert(st);
            }
        } else {
            // 单系统模式：只处理指定系统
            for (auto &st: roverData.satTypeValueData) {
                if (st.first.system == sysCode) {
                    // 根据系统代码选择对应的星历数据
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
            solve(roverData, TGD_Bool, Trop_Bool, Iono_Bool);
        }
        catch (...) {
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
        if (covMatrix.rows() >= 3) {
            double sigma0_val = solverLsq.getSigma0();
            double varX = covMatrix(0, 0) * sigma0_val * sigma0_val;
            double varY = covMatrix(1, 1) * sigma0_val * sigma0_val;
            double varZ = covMatrix(2, 2) * sigma0_val * sigma0_val;
            pdop = sqrt(varX + varY + varZ) / sigma0_val;
        }

        // 计算残差统计
        VectorXd residuals = solverLsq.getResiduals();
        double sigma0_val = solverLsq.getSigma0();
        if (sigma0_val > 5.0) {
            epochSkipStats.sigma0Exceed++;
            continue;
        }
        double meanResidual = residuals.mean();
        double rmsResidual = sqrt(residuals.squaredNorm() / residuals.size());
        double maxResidual = residuals.cwiseAbs().maxCoeff();
        // 调试：BDS 第一历元，残差 > 100 时打印详情
        if (maxResidual > 100 && sysCode.empty()) {
            for (auto &st : roverData.satTypeValueData) {
                if (st.first.system == "C") {
                    cerr << "[BDS RES] epoch=" << CommonTime2YDSTime(epoch)
                         << " sat=" << st.first
                         << " res=" << residuals.transpose()
                         << " sigma0=" << sigma0_val
                         << " nSat=" << nSat
                         << " prefit=" << st.second  // can't access prefit here
                         << endl;
                    break;
                }
            }
        }
        if (maxResidual > 100&&debug) {
            cout<<"epoch"<<epoch<<endl;
            cout<<"max_residual"<<maxResidual<<endl;
            cout<<"nSat"<<nSat<<endl;
            for (auto &st: roverData.satTypeValueData) {
                cout<<"sat"<<st.first<<endl;
            }
            for (auto &residual : residuals) {
                cout<<"residual"<<residual<<endl;
            }

        }

        // 计算 TGD 改正值统计
        double meanTGD = 0.0, maxTGD = -1e9, minTGD = 1e9;
        std::map<SatID, double> currentTGDData;
        
        // 从 solve 函数中获取 TGD 数据（通过成员变量 satTGDData）
        for (auto &tgd : satTGDData) {
            currentTGDData[tgd.first] = tgd.second;
            meanTGD += tgd.second;
            maxTGD = std::max(maxTGD, tgd.second);
            minTGD = std::min(minTGD, tgd.second);
        }
        if (!currentTGDData.empty()) {
            meanTGD /= currentTGDData.size();
        }

        // 计算相对论效应改正值统计
        double meanRelativity = 0.0, maxRelativity = -1e9, minRelativity = 1e9;
        std::map<SatID, double> currentRelativityData;
        
        // 从 solve 函数中获取相对论效应数据（通过成员变量 satRelativityData）
        for (auto &rel : satRelativityData) {
            currentRelativityData[rel.first] = rel.second;
            meanRelativity += rel.second;
            maxRelativity = std::max(maxRelativity, rel.second);
            minRelativity = std::min(minRelativity, rel.second);
        }
        if (!currentRelativityData.empty()) {
            meanRelativity /= currentRelativityData.size();
        }

        if(debug)
            cout << "spp:" << CommonTime2YDSTime(epoch) << " XYZ:" << xyzRover.transpose() << " ENU:" << enuVec.transpose() 
                 << " PDOP:" << pdop << " NSAT:" << nSat << " Sigma0:" << sigma0_val << endl;

        // 填充结果结构体
        SPPResult result;
        result.epoch = epoch;
        result.ydsTime = CommonTime2YDSTime(epoch);
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
        result.meanRelativity = meanRelativity;
        result.maxRelativity = maxRelativity;
        result.minRelativity = minRelativity;
        result.satRelativityData = currentRelativityData;
        
        results.push_back(result);

        // 调试代码时，设置一个stopEpoch，有助于快速得到结果
        CivilTime stopCivilTime = CivilTime(2025, 01, 01, 00, 01 , 30);
        CommonTime stopEpoch = CivilTime2CommonTime(stopCivilTime);
    }

    epochSkipStats.totalEpochs = results.size() + epochSkipStats.totalSkipped();

    roverObsStream.close();
    return results;
}

// BDS卫星类型判断（基于轨道参数）
std::string SPPCode::getBDSSatType(const SatID& sat, CommonTime epoch)
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
// 获取卫星类型权重
double SPPCode::getTypeWeight(const SatID& sat,CommonTime epoch) {
    if (sat.system != "C") {
        return 1.0;  // 非BDS卫星权重为1.0
    }

    std::string satType = getBDSSatType(sat,epoch);
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

bool SPPCode::strangeDataDelete(ObsData &obsData, double parameter) {
    VectorXd residuals = solverLsq.getResiduals();
    double sigma0 = solverLsq.getSigma0();
    double adaptiveThreshold = parameter * sigma0;
    double fixedThreshold = 10.0;

    if(debug) {
        cout << "\n【粗差探测】" << endl;
        cout << string(40, '-') << endl;
        cout << "Sigma0: " << fixed << setprecision(3) << sigma0 << " m" << endl;
        cout << "自适应阈值 (" << parameter << "σ): " << fixed << setprecision(3) << adaptiveThreshold << " m" << endl;
        cout << "固定阈值: " << fixed << setprecision(1) << fixedThreshold << " m" << endl;
        cout << "\n残差列表:" << endl;
        for (int i = 0; i < residuals.size(); ++i) {
            cout << "  卫星[" << i << "] " << equSys.satList[i] 
                 << ": " << fixed << setprecision(3) << residuals(i) << " m";
            if (abs(residuals(i)) > adaptiveThreshold) {
                cout << "  *** 超过自适应阈值 ***";
            }
            if (abs(residuals(i)) > fixedThreshold) {
                cout << "  *** 超过固定阈值 ***";
            }
            cout << endl;
        }
    }

    double maxv = 0.0;
    int badIndex = -1;

    for (int i = 0; i < residuals.size(); ++i) {
        if (abs(residuals(i)) > maxv) {
            maxv = abs(residuals(i));
            badIndex = i;
        }
    }

    if (maxv > adaptiveThreshold || maxv > fixedThreshold) {
        SatID badSat = equSys.satList[badIndex];

        if (debug) {
            cout << "\n【粗差剔除】" << endl;
            cout << "  被剔除卫星: " << badSat << endl;
            cout << "  残差值: " << fixed << setprecision(3) << maxv << " m" << endl;
            cout << "  剔除原因: ";
            if (maxv > fixedThreshold) {
                cout << "超过固定阈值 (" << fixedThreshold << "m)";
            } else {
                cout << "超过自适应阈值 (" << parameter << "σ = " << adaptiveThreshold << "m)";
            }
            cout << endl;
        }

        // 超大残差（>100m）：直接删星，不降权
        if (maxv > 100.0) {
            obsData.satTypeValueData.erase(badSat);
            if (debug) cout << "  超大残差，直接删星: " << badSat << " (" << maxv << "m)" << endl;
            return true;
        }

        // 中等残差：加入降权集合，不删星
        if (outlierSats.find(badSat) != outlierSats.end()) {
            if (debug) cout << "  卫星已在降权集合，跳过" << endl;
            return false;
        }
        outlierSats.insert(badSat);
        if (debug) cout << "  加入降权集合: " << badSat << endl;
        return true;
    } else {
        if(debug) {
            cout << "\n【粗差探测】无异常数据，所有残差均在阈值范围内" << endl;
        }
        return false;
    }
}

void SPPCode::printEpochSkipStats(const EpochSkipStats& stats) {
    int totalSkipped = stats.totalSkipped();
    int totalEpochs = stats.totalEpochs;

    cout << "\n========================================" << endl;
    cout << " 历元跳过统计 (Epoch Skip Statistics)" << endl;
    cout << "========================================" << endl;
    cout << " 总历元数                    : " << totalEpochs << endl;
    cout << "----------------------------------------" << endl;
    cout << " --- 历元级跳过（整个历元丢弃）---" << endl;
    cout << " 卫星数不足 (SVNumException) : " << stats.svNumException << endl;
    cout << " Sigma0 == 0                 : " << stats.sigma0Zero << endl;
    cout << " PDOP 无效 (NaN/∞/负)       : " << stats.pdopInvalid << endl;
    cout << " PDOP > 5                  : " << stats.pdopExceed << endl;
    cout << " 迭代不收敛 (>10次)          : " << stats.iterNotConverge << endl;
    cout << " Sigma0 > 10.0               : " << stats.sigma0Exceed << endl;
    cout << "----------------------------------------" << endl;
    cout << " 小计                        : " << totalSkipped;
    if (totalEpochs > 0) {
        cout << " / " << totalEpochs << " (" << (100.0 * totalSkipped / totalEpochs) << "%)";
    }
    cout << endl;
    cout << "----------------------------------------" << endl;
    cout << " --- 卫星级剔除（颗数）---" << endl;
    cout << " 无匹配观测类型              : " << stats.satNoCodeType << endl;
    cout << " 观测值不存在                : " << stats.satNoObsValue << endl;
    cout << " 星历计算失败                : " << stats.satEphFailed << endl;
    cout << " 粗差剔除 (strangeDataDelete): " << stats.satOutlierDeleted << endl;
    cout << "========================================\n" << endl;
}
