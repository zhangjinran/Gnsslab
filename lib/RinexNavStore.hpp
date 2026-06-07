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

#ifndef RinexNavStore_HPP
#define RinexNavStore_HPP

#include <iostream>
#include <string>
#include <list>
#include <map>
#include <algorithm>
#include <fstream>
#include <memory>

#include "NavEphGPS.hpp"
#include "NavEphBDS.hpp"
#include "NavEphGLONASS.hpp"
#include "NavEphGalileo.hpp"
#include "NavEphQZSS.hpp"
#include "NavEphIRNSS.hpp"
#include "NavEphBase.hpp"
#include "GnssStruct.h"
#include "SP3Store.hpp"

using namespace std;

class RinexNavStore {

    public:

    RinexNavStore() : isLoadedFlag(false) {};

    void loadGPSEph(NavEphGPS &gpsEph, string &line, fstream &navFile);//读取单个卫星、单历元的导航电文所有数据
    void loadBDSEph(NavEphBDS &bdsEph, string &line, fstream &navFile);//读取单个卫星、单历元的导航电文所有数据
    void loadGLOEph(NavEphGLONASS &gloEph, string &line, fstream &navFile);//读取GLONASS卫星导航电文数据
    void loadGalileoEph(NavEphGalileo &galEph, string &line, fstream &navFile);//读取Galileo卫星导航电文数据
    void loadQZSSEph(NavEphQZSS &qzssEph, string &line, fstream &navFile);//读取QZSS卫星导航电文数据
    void loadIRNSSEph(NavEphIRNSS &irnssEph, string &line, fstream &navFile);//读取IRNSS卫星导航电文数据
    NavEphGLONASS findGLOEph(const SatID &sat, const CommonTime &epoch);//寻找GLONASS卫星星历数据
    NavEphGalileo findGalileoEph(const SatID &sat, const CommonTime &epoch);//寻找Galileo卫星星历数据
    NavEphQZSS findQZSSEph(const SatID &sat, const CommonTime &epoch);//寻找QZSS卫星星历数据
    NavEphIRNSS findIRNSSEph(const SatID &sat, const CommonTime &epoch);//寻找IRNSS卫星星历数据
    bool loadFile(string &file);//读取单篇文件的所有数据。可以调用loadGPSEph函数。返回true表示成功，false表示失败。
    Xvt getXvt(const SatID &sat, const CommonTime &epoch);//用于计算位置速度和钟差。
    NavEphGPS findGPSEph(const SatID &sat, const CommonTime &epoch);//寻找指定卫星编号和历元的导航电文数据。
    NavEphBDS findBDSEph(const SatID &sat, const CommonTime &epoch);//寻找指定卫星编号和历元的导航电文数据。
    std::unique_ptr<NavEphBase> findEph(const SatID &sat, const CommonTime &epoch);//使用工厂函数统一查找星历数据
    void gerContrast(string system ,int Cout,CommonTime predictedTimeInit,CommonTime stoptime ,SP3Store sp3Store) ;
    void getContrastData(SP3Store &sp3Eph,CommonTime predictedTime,CommonTime stoptime,int period);
    void writeFile(std::string filename,std::string name);
    void writeContrastData(std::ofstream& fout, const std::map<SatID, ContrastData>& dataSet);
    /// 检查是否已成功加载数据
    bool isLoaded() const { return isLoadedFlag; }

    /// 检查是否有星历数据
    bool hasEphData() const { 
        return !gpsEphData.empty() || !bdsEphData.empty(); 
    }

    /// 检查指定系统是否有星历数据
    bool hasEphData(const std::string& system) const;

    /// 获取已加载的系统列表
    std::vector<std::string> getSystems() const;

    /// 获取支持的系统列表（通过工厂类）
    std::vector<std::string> getSupportedSystems() const;

    /// destructor
    virtual ~RinexNavStore() {};

    //时间系统改正参数，从一个时间系统修改到另一个时间系统。这是一个多项式架构。
    struct TimeSysCorr {
        double A0;
        double A1;
        int refSOW;
        int refWeek;
        string geoProvider;
        int geoUTCid;
    };

    // BDS电离层参数结构
    struct BDSIonoParam {
        double alpha[4] = {0};
        double beta[4] = {0};
        bool hasAlpha = false;
        bool hasBeta = false;
    };
    
    //电离层改正参数的数据结构
    typedef map<string, vector<double>> ionoCorrMap;
    //BDS专用电离层参数（按SatID存储）
    typedef map<SatID, BDSIonoParam> ionoCorrBDSMap;
    //时间系统改正参数的数据结构
    typedef map<string, TimeSysCorr> timeSysCorrMap;

    string rx3NavFile;
    double version;                ///< RINEX Version
    std::string fileType;          ///< File type "N...."
    std::string fileSys;           ///< File system string
    std::string fileProgram;       ///< Program string
    std::string fileAgency;        ///< Agency string
    std::string date;              ///< Date string; includes "UTC" at the end
    std::vector<std::string> commentList;  ///< Comment list

    ionoCorrMap ionoCorrData;
    ionoCorrBDSMap ionoCorrDataBDS;  // BDS专用电离层参数
    timeSysCorrMap timeSysCorrData;

    long leapSeconds;              ///< Leap seconds
    long leapDelta;                ///< Change in Leap seconds at ref time
    long leapWeek;                 ///< Week number of ref time
    long leapDay;                  ///< Day of week of ref time

    static const std::string stringVersion;      /// "RINEX VERSION / TYPE"
    static const std::string stringRunBy;        /// "PGM / RUN BY / DATE"
    static const std::string stringComment;      /// "COMMENT"
    // R3.x
    static const std::string stringIonoCorr;     /// "IONOSPHERIC CORR"
    static const std::string stringTimeSysCorr;  /// "TIME SYSTEM CORR"
    static const std::string stringLeapSeconds;  /// "LEAP SECONDS"
    static const std::string stringDeltaUTC;     /// "DELTA-UTC: A0,A1,T,W" // R2.11 GPS
    static const std::string stringCorrSysTime;  /// "CORR TO SYSTEM TIME"  // R2.10 GLO
    static const std::string stringDUTC;         /// "D-UTC A0,A1,T,W,S,U"  // R2.11 GEO
    static const std::string stringIonAlpha;     /// "ION ALPHA"            // R2.11
    static const std::string stringIonBeta;      /// "ION BETA"             // R2.11
    static const std::string stringEoH;          /// "END OF HEADER"

    ///tables that record the sat of different sat system
    ///in order to get the eph data num easily
    typedef map<SatID, std::map<CommonTime, NavEphGPS>> GpsEphMap;
    typedef map<SatID, std::map<CommonTime, NavEphBDS>> BdsEphMap;
    typedef map<SatID, std::map<CommonTime, NavEphGLONASS>> GloEphMap;
    typedef map<SatID, std::map<CommonTime, NavEphGalileo>> GalEphMap;
    typedef map<SatID, std::map<CommonTime, NavEphQZSS>> QzssEphMap;
    typedef map<SatID, std::map<CommonTime, NavEphIRNSS>> IrnssEphMap;
    typedef map<SatID, std::map<CommonTime, std::unique_ptr<NavEphBase>>> EphDataMap;
    
    vector<SatID> satTable;

    GpsEphMap gpsEphData;
    BdsEphMap bdsEphData;
    GloEphMap gloEphData;
    GalEphMap galEphData;
    QzssEphMap qzssEphData;
    IrnssEphMap irnssEphData;
    EphDataMap ephData;
    std::set<std::string> loadedSystems;
    map<SatID,ContrastData> gpscontrastDataSet;
    map<SatID,ContrastData> bdscontrastDataSet;
    map<SatID,ContrastData> gloContrastDataSet;
    map<SatID,ContrastData> galContrastDataSet;

    map<string,int> EphData{{"G",0},{"C",0},{"R",0},{"E",0},{"J",0},{"I",0},{"S",0}};

    /// 加载状态标志
    bool isLoadedFlag;




};


#endif // RinexNavStore_HPP