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

#include "SPPGFCode.h"
#include "CoordConvert.h"
#include <iostream>
#include "RinexObsReader.h"
#include <Eigen/Eigen>


#include "GnssFunc.h"

#define debug 0

void SPPGFCode::solve(ObsData &obsData,bool TGD_bool,bool Trop_Bool) {
    //----------------------
    // 去掉通道号，C1W, C1C => C1;
    // 后面computeSatPos里用与通道号无关的观测值计算卫星发射时刻位置
    //----------------------
    convertObsType(obsData);

    if(debug)
    {
        cout << "after convertObsType" << endl;
        cout << obsData << endl;
    }

    // 计算IF组合
    //computeIF(obsData);
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
    if (TGD_bool)
        correctTGD(obsData);

    xyz = obsData.antennaPosition;
    dxyz = {100, 100, 100};

    int iter(0);
    while (true) {
        CommonTime time(60676,10380);
        if (obsData.epoch==time) {
            //cout<<"time:"<<obsData.epoch<<endl;
        }


        satXvtRecTime = earthRotation(xyz, satXvtTransTime);

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
        if (numSats < 5 ) {
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
            if (Trop_Bool)
                tropdelaymap=computeTropDelay(obsData,satElevData);
        }

        equSys = linearize(xyz, satXvtRecTime, satElevData, obsData,tropdelaymap);

        if(debug)
            cout << "afte linearize:" << endl;

        // 如果是基准站，完成线性化后就退出
        // 因为基准站位置是准确的
        if(!isRover)
            break;

        solverLsq.solve(equSys);
        dxyz = solverLsq.getdxyz();

        xyz += dxyz;
        if (debug) {
            cout << "iteration:" << iter<<" "
            << "dxyz:" << dxyz.transpose()<<" "
            << "xyz:" << xyz.transpose() << endl;
        }



        // convergence threshold
        if (dxyz.norm() < 0.1) {
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


std::map<SatID, Xvt> SPPGFCode::computeSatPos(ObsData &obsData) {
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
        string GFCodeType;
        if (sat.system == "G") {
            GFCodeType = "C1";
        }
        else if (sat.system == "C") {
            GFCodeType="C1";
        }
        // todo
        // 请增加bds或其他系统的观测值选择
        else {
            satRejectedSet.insert(sat);
            continue;
        }

        // GFCode obs
        try {
            obs = stv.second.at(GFCodeType);
            if(debug)
                cout << "sat:" << sat << "obs:" << GFCodeType << "value:" << obs << endl;
        }
        catch (...) {
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

Xvt SPPGFCode::computeAtTransmitTime(const CommonTime &tr,
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
        tt -= (xvt.clkbias + xvt.relcorr);
    }
    return xvt;
};

void SPPGFCode::convertObsType(ObsData &obsData) {

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



std::map<SatID, Xvt> SPPGFCode::earthRotation(Eigen::Vector3d &xyz,
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

void SPPGFCode::computeElevAzim(Eigen::Vector3d& xyz,
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


EquSys SPPGFCode::linearize(Eigen::Vector3d& xyz,
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
            }
            // 可无限扩展
        }

        // 对每个观测值，都需要存储对应的未知参数及其偏导数
        for (auto tv: stv.second)
        {
            if ((sat.system=="G" &&( (tv.first == "C1")||tv.first=="C2"))||(sat.system=="C" && (tv.first == "C1"||tv.first=="C2")) )
            {
                EquID equID = EquID(sat, tv.first);
                ObsID obs_id(sat.system, tv.first);
                Variable iono(obsData.station,sat, Parameter::iono);

                //>> 先验残差
                double prefit;
                double computedObs = (rho - clkBias - relCorr + slantTrop) ;
                prefit = tv.second - computedObs;

                if (debug) {
                    cout << "obs:" << tv.second << endl;
                    cout << "rho:" << rho << endl;
                    cout << "clkBias:" << clkBias << endl;
                    cout << "relCorr:" << relCorr << endl;
                    cout << "slantTrop:" << slantTrop << endl;
                }

                equSysTemp.obsEquData[equID].prefit = prefit;
                equSysTemp.obsEquData[equID].varCoeffData[dx] = cosines[0];
                equSysTemp.obsEquData[equID].varCoeffData[dy] = cosines[1];
                equSysTemp.obsEquData[equID].varCoeffData[dz] = cosines[2];

                double f2=getFreq(equID.toString(),tv.first);
                double gamma =pow(L1_FREQ_GPS,2)/pow(f2,2);

                equSysTemp.obsEquData[equID].varCoeffData[iono] = gamma;
                SatID sat = stv.first;
                std::string sys = sat.system;

                // 自动给对应系统的钟差赋系数 1.0
                equSysTemp.obsEquData[equID].varCoeffData[ clockParams[sys] ] = 1.0;




                // Compute the weight according to elevation
                double weight;
                if(elev >= 30){
                    weight = 1.0 / (sigGFCode * sigGFCode);
                }
                else
                {
                    weight = 1.0 / (sigGFCode * sigGFCode) * std::pow(std::sin(elevRad), 2);
                }

                equSysTemp.obsEquData[equID].weight = weight; // IF组合方差为1.0m

                // 把当前观测方程未知参数插入到总体的未知参数
                varSetTemp.insert(dx);
                varSetTemp.insert(dy);
                varSetTemp.insert(dz);
                varSetTemp.insert(clockParams[sys]);
                varSetTemp.insert(iono);
                equSysTemp.satList.push_back(sat);

            }
        }
    }

    equSysTemp.varSet = varSetTemp;

    return equSysTemp;
};


void SPPGFCode::correctTGD( ObsData &obsdata) {

    CommonTime epoch=obsdata.epoch;
    for (auto &tv: obsdata.satTypeValueData) {
        if (tv.first.system == "G")
        {
            for (auto &st: tv.second) {
                double Delta_TGD(0.0);
                NavEphGPS nav_eph_gps=pEphStore->findGPSEph(tv.first,epoch);
                double TGD=nav_eph_gps.TGD;
                if (st.first == "C1") {


                    Delta_TGD = -C_MPS*TGD;
                    st.second+=Delta_TGD;
                    //cout<<"sat:"<<tv.first<<"Delta_TGD:"<<Delta_TGD<<endl;


                }
                else if (st.first == "C2") {
                    double f1_2=pow(L1_FREQ_GPS,2);
                    double f2_2=pow(L2_FREQ_GPS,2);
                    Delta_TGD = -f1_2/f2_2*C_MPS*TGD;
                    st.second+=Delta_TGD;
                }
            }

        }
        else if (tv.first.system == "C")
        {
            for (auto &st: tv.second) {
                double Delta_TGD(0.0);
                NavEphBDS nav_eph_bds=pEphStore->findBDSEph(tv.first,epoch);
                double TGD1=nav_eph_bds.TGD1;
                if (st.first == "C1") {

                    Delta_TGD = 0;
                    st.second+=Delta_TGD;
                   // cout<<"sat:"<<tv.first<<" Delta_TGD:"<<Delta_TGD<<endl;
                }
                else if (st.first == "C2") {
                    Delta_TGD = -C_MPS*TGD1;
                    st.second+=Delta_TGD;
                }
            }

        }
    }

}

std::map<SatID,double> SPPGFCode::computeTropDelay(ObsData &obsdata,std::map<SatID, double>&satElevData) {
    Vector3d xyz=obsdata.antennaPosition;
    std::map<SatID,double>tropDelayMap= tropDelay(xyz,satElevData);
    return tropDelayMap;
}


void SPPGFCode::full_solve(RinexNavStore* pStore,string solFile,string roverFile,bool TGD_Bool,bool Trop_Bool ) {

    std::fstream roverObsStream(roverFile);
    if (!roverObsStream) {
        cerr << "rover file open error!" << strerror(errno) << endl;
        exit(-1);
    }
    RinexObsReader readObsRover;
    readObsRover.setFileStream(&roverObsStream);
    setRinexNavStore(pStore);


    //>>> classes for rtk;
    SolverLSQ solver;

    std::fstream solStream(solFile, ios::out);
    if (!solStream) {
        cerr << "solution file open error!" << strerror(errno) << endl;
        exit(-1);
    }

    while (true) {

        // solve spp for rover
        ObsData roverData;

        try {
            roverData = readObsRover.parseRinexObs();
            // cout << "roverData:" << roverData << endl;
        }
        catch (EndOfFile &e) { break; }

        CommonTime epoch = roverData.epoch;

        SatTypeValueMap keep_data;
        for (auto &st: roverData.satTypeValueData) {
            if (st.first.system == "C") {
                if (pEphStore->bdsEphData.find(st.first)!=pEphStore->bdsEphData.end() ) {
                    keep_data.insert(st);
                }
            }
            if (st.first.system == "G") {
                if (pEphStore->gpsEphData.find(st.first)!=pEphStore->gpsEphData.end() ) {
                    keep_data.insert(st);
                }
            }
        }
        roverData.satTypeValueData.swap(keep_data);

        // 最小二乘
        try {
            solve(roverData,TGD_Bool,Trop_Bool);
        }
        catch (...) {
            continue;
        }
        EquSys equSysRover = getEquSys();
        Vector3d xyzRover = getXYZ();

        if(debug)
            cout << "spp:" << CommonTime2YDSTime(epoch) << xyzRover.transpose() << endl;

        // print solution to files
        printSolution(solStream, epoch, xyzRover);

        // 调试代码时，设置一个stopEpoch，有助于快速得到结果
        // 2022 03 03 06 48 37.0000000  0 45
        CivilTime stopCivilTime = CivilTime(2025, 01, 01, 00, 01 , 30);
        //   CivilTime stopCivilTime = CivilTime(2021, 01, 01, 10, 00 , 00);

        CommonTime stopEpoch = CivilTime2CommonTime(stopCivilTime);


    }

    solStream.close();
    roverObsStream.close();


}
bool SPPGFCode::strangeDataDelete(ObsData &obsData,double parameter) {
    VectorXd residuals = solverLsq.getResiduals();
    double sigma0 = solverLsq.getSigma0();
    double threshold = parameter * sigma0;
    if(debug)
    {
        cout << "Residuals:" << endl;
        cout << residuals << endl;

        cout << "Sigma0:" << sigma0 << endl;
        cout << "threshold:" << threshold << endl;
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
    if(maxv > threshold){
        SatID badSat = equSys.satList[badIndex];

        if (debug) {
            cout<<"remove outlier: "
                <<badSat
                <<" residual="
                <<maxv<<endl;
        }

        obsData.satTypeValueData.erase(badSat);
        //cout<<obsData.satTypeValueData<<endl;
        return true;

    }
    else{
        return false;
    }
}