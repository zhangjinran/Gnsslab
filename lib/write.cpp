//
// Created by zhang on 2026/5/2.
//
#include <string>
#include <algorithm> //replace 函数
#include "TimeConvert.h"
#include "GnssStruct.h"
#include "GnssFunc.h"
#include "ARLambda.hpp"
#include "CoordConvert.h"
#include "RinexNavStore.hpp"
#include"CoordConvert.h"
#include"CoordStruct.h"
#include"write.h"
#define debug 1
#define debugCSMW 1


void writeDelay(satValueEpochMap delayMap, string filename) {
    string dirpath  ="/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnss_draw/data/";
    string filepath=dirpath+filename+".txt";
    ofstream fout(filepath,std::ios::out);
    if (!fout) {
        cerr << "Unable to open file for writing" << endl;
        return;
    }

    fout<<"epoch"<<" ";
    auto filehead=delayMap.begin();
    vector<string> sat_vector;

    for (auto it:filehead->second) {
        fout<<it.first.toString()<<" ";
        sat_vector.push_back(it.first.toString());
    }

    fout<<endl;
    for (auto epoch:delayMap) {
        CommonTime epochTime=epoch.first;
        JulianDate epoch_civil=CommonTime2JulianDate(epochTime);
        fout<<epoch_civil.toString()<<" ";
        for (auto str:sat_vector) {
            int choose=0;
            for (auto it:epoch.second) {
                if (it.first.toString()==str)
                {
                    fout<<it.second<<" ";
                    choose=1;
                    break;
                }
            }
            if (choose==0)
                fout<<"#"<<" ";
        }
        fout<<endl;
    }
};
