/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *  As stipulated by the MulanPSL-2.0, you are granted the following freedoms:
 *      To copy, use, and modify the software;
 *      To use the software for commercial purposes;
 *      To redistribute the software.
 *
 * Author: shoujian zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * References:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
 * 2. Eckel, Bruce. Thinking in C++. 2nd ed., Prentice Hall, 2000.
 */


#include <string>
#include <fstream>
#include <iostream>
#include <cstring>
#include <set>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/TimeConvert.h>
#include <gnsslab/GnssFunc.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/RinexObsReader.h>
#include <gnsslab/SPPIFCode.h>
#include <gnsslab/write.h>

#define debug 0

using namespace std;

int main() {

    //--------------------
    // 打开文件流
    //--------------------

    // Replace with your actual RINEX file path
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
//    string dirPath = "D:\\documents\\Source\\gnssLab-2.1\\data\\";

    // rover obs file name
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    //cout << roverFile << endl;

    // nav file name, download from IGS ftp site:ftp://gssc.esa.int/gnss/data/daily/YYYY/brdc
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";

    // 2022 03 03 06 48 37.0000000  0 45
    CivilTime stopCivilTime = CivilTime(2025, 1, 1, 0, 0, 30);
    CommonTime stopEpoch = CivilTime2CommonTime(stopCivilTime);

    std::fstream roverObsStream(roverFile);
    if (!roverObsStream) {
        cerr << "rover file open error!" << strerror(errno) << endl;
        exit(-1);
    }

    // read nav file data before rtk
    RinexNavStore navStore;
    navStore.loadFile(navFile);


    cout << "after NavStore" << endl;

    std::map<string, std::set<string>> selectedTypes;
    selectedTypes["G"].insert("C1C");
    selectedTypes["G"].insert("C2W");
    selectedTypes["G"].insert("L1C");
    selectedTypes["G"].insert("L2W");

    //-------------------
    // 定义数据处理的对象
    //-------------------
    //>>> classes for rover
    RinexObsReader readObsRover;
    readObsRover.setFileStream(&roverObsStream);
    readObsRover.setSelectedTypes(selectedTypes);

    std::map<CommonTime,std::map<SatID, double>> IonoDelay;
    std::map<CommonTime, std::map<SatID, double>> TropDelay;
    int count =0;
    while (true) {

        // solve spp for rover
        ObsData roverData;

        try {
            if (count>=614)
                cout << count << endl;

            roverData = readObsRover.parseRinexObs();

            if (count>=180)
                cout<<roverData<<endl;
        }
        catch (EndOfFile &e) { break; }
        //if (roverData.satTypeValueData.empty())
          //  break;

        CommonTime epoch = roverData.epoch;
        //----------------------
        // 去掉通道号，C1W, C1C => C1;
        // 后面computeSatPos里用与通道号无关的观测值计算卫星发射时刻位置
        //----------------------
        convertObsType(roverData);

        // 计算发射时刻卫星位置（参考框架为时刻的）
        CivilTime civil_time=CommonTime2CivilTime(epoch);
        std::map<SatID, Xvt> satXvtTransTime = computeSatPos(roverData, navStore,0);
        std::map<SatID, Xvt> satXvtTransTimeIF = computeSatPos(roverData, navStore,1);
        writefileSatPos(satXvtTransTime, satXvtTransTimeIF, civil_time, "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/satXvtTransTime_IF.txt");

        if (debug) {
            cout << "satXvtTransTime  " << CommonTime2CivilTime(roverData.epoch) << endl;

            for (auto sx: satXvtTransTime) {
                if (sx.first.toString()=="G10") {
                    cout << sx.first << endl;
                    cout << sx.second << endl;
                }
            };
            cout << "satXvtTransTimeIF  " << CommonTime2CivilTime(roverData.epoch) << endl;
            for (auto sx: satXvtTransTimeIF) {
                if (sx.first.toString()=="G10") {
                    cout << sx.first << endl;
                    cout << sx.second << endl;
                }
            };
        }

        //----------------------
        // 得到卫星发射时刻位置和钟差、相对论和TGD后，改正观测值延迟，并更新C1/C2等观测值
        //----------------------
        //
        //correctTGD(obsData);

        Vector3d xyz = roverData.antennaPosition;
        std::map<SatID, Xvt> satXvtRecTime = earthRotation(xyz, satXvtTransTime);

        if (debug) {
            cout << "satXvtRecTime" << endl;
            for (auto sx: satXvtRecTime) {
                if (sx.first.toString()=="G10") {
                    cout << sx.first << " xvt:" << endl;
                    cout << sx.second << endl;
                }
            };
        }

        SatValueMap satElevData, satAzimData;
        // 地球表面才计算高度角和大气改正
        if (std::abs(xyz.norm() - RadiusEarth) < 100000.0) {
             satElevData.clear();
             satAzimData.clear();
             computeElevAzim(xyz, satXvtRecTime, satElevData, satAzimData);

             std::map<SatID, double> ionodelay;
             std::map<SatID, double> tropdelay;

             ionodelay=ionoDelay(xyz, epoch, satElevData, satAzimData,navStore);
             tropdelay=tropDelay(xyz,satElevData,0.5);
             cout<<"epoch:"<<epoch<<endl;
             IonoDelay[roverData.epoch]=ionodelay;
            TropDelay[roverData.epoch]=tropdelay;
            count++;

         }

        // 调试代码时，设置一个stopEpoch，有助于快速得到结果

    }
    writeDelay(IonoDelay,"IonoDelay");
    writeDelay(TropDelay,"TropDelay");

    roverObsStream.close();

};