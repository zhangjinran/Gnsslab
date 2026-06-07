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
#include "GnssStruct.h"
#include "TimeConvert.h"
#include "GnssFunc.h"
#include "RinexNavStore.hpp"
#include "RinexObsReader.h"
#include "SPPCode.h"


#define debug 1

using namespace std;
void test(string roverFile,RinexNavStore* pNavStore,std::map<string, std::pair<string, string>> BDSCodeTypes,
    std::map<string, std::pair<string, string>> CodeTypes,
    std::map<string, std::pair<string, string>> GPSCodeTypes,string name="",bool TGD_Bool=true,bool Trop_Bool=true,bool Iono_Bool=true
    ){


    string temp_file=roverFile+name;

    std::string solFile = temp_file + ".spp.norm.out";
    SPPCode spp;
    spp.full_solve(pNavStore,solFile,roverFile,TGD_Bool,Trop_Bool,Iono_Bool);

}


int main() {

    //--------------------
    // 打开文件流
    //--------------------

    // Replace with your actual RINEX file path
    string dirPath = "/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/data/";
    //    string dirPath = "D:\\documents\\Source\\gnssLab-2.1\\data\\";

    // rover obs file name
    std::string roverFile = dirPath + "WUH200CHN_R_20250010000_01D_30S_MO.rnx";
    //std::string roverFile = dirPath + "ABMF00GLP_R_20210010000_01D_30S_MO.rnx";

    cout << roverFile << endl;

    // nav file name, download from IGS ftp site:ftp://gssc.esa.int/gnss/data/daily/YYYY/brdc
    std::string navFile = dirPath + "BRDC00IGS_R_20250010000_01D_MN.rnx";
    // std::string navFile = dirPath + "ABMF00GLP_R_20210010000_01D_MN.rnx";



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


    std::map<string, std::pair<string, string>> CodeTypes;
    CodeTypes["G"].first = "C1";
    CodeTypes["G"].second = "C2";
    CodeTypes["C"].first = "C1";
    CodeTypes["C"].second = "C2";

    std::map<string, std::pair<string, string>> GPSCodeTypes;
    GPSCodeTypes["G"].first = "C1";
    GPSCodeTypes["G"].second = "C2";

    std::map<string, std::pair<string, string>> BDSCodeTypes;
    BDSCodeTypes["C"].first = "C1";
    BDSCodeTypes["C"].second = "C2";
    //-------------------
    // 定义数据处理的对象
    //-------------------
    //>>> classes for rover


    RinexObsReader readObsRover;
    readObsRover.setFileStream(&roverObsStream);
    readObsRover.setSelectedTypes(selectedTypes);
    test(roverFile,&navStore,BDSCodeTypes,CodeTypes,GPSCodeTypes,"no-TGD-Trop",false,false);
    test(roverFile,&navStore,BDSCodeTypes,CodeTypes,GPSCodeTypes,"TGD",true,false);
    test(roverFile,&navStore,BDSCodeTypes,CodeTypes,GPSCodeTypes,"Trop",false,true);
    test(roverFile,&navStore,BDSCodeTypes,CodeTypes,GPSCodeTypes);

}