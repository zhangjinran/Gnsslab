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


// ============================================================================
// 文件概述：GNSS数据处理核心函数库
// 功能：包含RINEX文件解析、卫星位置计算、误差校正、周跳探测、差分处理等
// 主要模块：
// 1. RINEX文件读写解析 (parseRinexHeader, parseRinexObs, parseTime)
// 2. 观测数据预处理 (chooseObs, convertObsType)
// 3. 卫星位置计算 (computeSatPos, computeAtTransmitTime)
// 4. 误差校正 (correctTGD, earthRotation, ionoDelay, tropDelay)
// 5. 周跳探测 (detectCSMW)
// 6. 差分处理 (differenceStation, differenceSat)
// 7. 模糊度固定 (fixSolution, ambiguityDatum)
// 8. 结果输出 (printSolution)
// ============================================================================

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
#define debug 0
#define debugCSMW 0

/**
 * 函数：parseRinexHeader
 * 功能：解析RINEX文件头信息
 * 参数：
 *   rinexFileStream - RINEX文件输入流
 *   rinexHeader     - 输出：RINEX头信息结构体
 * 说明：
 *   1. 读取RINEX文件版本，仅支持3.04版本
 *   2. 解析测站名、近似位置、观测类型等信息
 *   3. 按系统(GPS/BDS)存储观测类型
 *   4. 遇到"END OF HEADER"标签时停止解析
 */
void parseRinexHeader(std::fstream &rinexFileStream, RinexHeader &rinexHeader) {

    double version;
    XYZ antennaPosition;
    string satSys;
    std::map<string, std::vector<string>> mapObsTypes;
    while (true) {
        string line;
        getline(rinexFileStream, line);

        if(debug)
            cout << "parseRinexHeader:" << line << endl;

        string label;
        if (line.size() >= 80)
            label = line.substr(60, 20);

        strip(label);

        if (label == "END OF HEADER") {
            break;
        } else if (label == "MARKER NAME") {
            string markerName = line.substr(0, 60);
            std::replace(markerName.begin(), markerName.end(), ' ', '_');
            rinexHeader.station = markerName;

        } else if (label == "RINEX VERSION / TYPE") {
            version = safeStod(line.substr(0, 20));
            if (version != 3.04) {
                cerr << "only support rinex 3.04 version!" << endl;
                exit(-1);
            }
            rinexHeader.version = version;
        } else if (label == "APPROX POSITION XYZ") {
            antennaPosition[0] = safeStod(line.substr(0, 14));
            antennaPosition[1] = safeStod(line.substr(14, 14));
            antennaPosition[2] = safeStod(line.substr(28, 14));
            rinexHeader.antennaPosition = antennaPosition;
        } else if (label == "SYS / # / OBS TYPES") {
            string sysStr;
            sysStr = line.substr(0, 1);
            strip(sysStr);

            int numObs;

            if (sysStr != "") {
                numObs = stoi(line.substr(3, 3));
                satSys = sysStr;
            }

            const int maxObsPerLine = 13;
            for (int i = 0; i < maxObsPerLine && mapObsTypes[satSys].size() < numObs; i++) {
                std::string typeStr = (line.substr(4 * i + 7, 3));
                // insert into mapObsTypes
                mapObsTypes[satSys].push_back(typeStr);
            }
            rinexHeader.mapObsTypes = mapObsTypes;
        }
    }
};

/**
 * 函数：parseRinexObs
 * 功能：解析RINEX观测文件的一个历元观测数据
 * 参数：
 *   rinexFileStream - RINEX文件输入流
 * 返回值：ObsData - 包含一个历元的观测数据
 * 说明：
 *   1. 首次调用时解析文件头（静态变量isHeaderRead控制）
 *   2. 读取历元行，检查历元标记和历元标志
 *   3. 解析时间、卫星数量
 *   4. 读取每个卫星的观测值，按观测类型存储
 *   5. 支持GPS("G")和北斗("C")系统，其他系统跳过
 *   6. 载波相位观测值转换为距离（米）
 *   7. 观测值异常（值为0）跳过
 */
ObsData parseRinexObs(std::fstream &rinexFileStream) {
    static bool isHeaderRead = false;
    static RinexHeader rinexHeader;

    if (!isHeaderRead) {
        parseRinexHeader(rinexFileStream, rinexHeader);
        isHeaderRead = true;
    }

    // 读取观测值
    std::string line;
    getline(rinexFileStream, line);

    if (rinexFileStream.eof()) {
        EndOfFile err("EOF encountered!");
        throw err;
    }

    if (debug) {
        std::cout << "current record line is:" << std::endl;
        std::cout << line << std::endl;
    }

    // 检查并解析历元行
    // 检查历元标记 ('>') 和随后的空格。
    if (line[0] != '>' || line[1] != ' ') {
        FFStreamError e("Bad epoch line: >" + line + "<");
        throw e;
    }

    int epochFlag = stoi(line.substr(31, 1));
    if (epochFlag < 0 || epochFlag > 6) {
        FFStreamError e("Invalid epoch flag: " + std::to_string(epochFlag));
        throw e;
    }

    CommonTime currEpoch = parseTime(line);
    if (debug) {
        std::cout << " currEpoch" << currEpoch << std::endl;
    }

    int numSats = stoi(line.substr(32, 3));

    if (debug) cout << numSats << endl;

    // 读取观测：SV ID 和数据
    SatTypeValueMap stvData;
    if (epochFlag == 0 || epochFlag == 1 || epochFlag == 6) {

        std::vector<SatID> satIndex(numSats);
        for (int isv = 0; isv < numSats; ++isv) {
            getline(rinexFileStream, line); // 修改了这里的变量名以匹配上下文

            if (debug) {
                cout << "parseRinexObs:" << line << endl;
            }

            if (rinexFileStream.eof()) {
                EndOfFile err("EOF encountered!");
                throw err;
            }

            // 获取 SV ID
            try {
                satIndex[isv] = SatID(line.substr(0, 3));
            } catch (std::exception &e) {
                FFStreamError ffse(e.what());
                throw ffse;
            }

            SatID sat = SatID(satIndex[isv]);

            // 如果卫星系统不是GPS("G")也不是北斗("C")，则跳过当前循环迭代。
            if (sat.system != "G" && sat.system != "C") {
                continue;
            }

            int size = rinexHeader.mapObsTypes.at(satIndex[isv].system).size();

            // 有些文件没有观测值，后面就没有输出，这里用空格来替换，否则解析错误
            size_t minSize = 3 + 16 * size;
            if (line.size() < minSize) {
                line += std::string(minSize - line.size(), ' ');
            }

            // 获取数据 (# entries in ObsType map of maps from header)
            TypeValueMap typeObs;
            TypeValueMap typeLLI;
            TypeValueMap typeSSI;
            for (int i = 0; i < size; ++i) {
                size_t pos = 3 + 16 * i;
                std::string str = line.substr(pos, 16);

                // ObsType
                std::string obsTypeStr = rinexHeader.mapObsTypes.at(sat.system)[i];

                // 观测值
                std::string tmpStr = str.substr(0, 14);

                double data = safeStod(tmpStr);

                // 载波相位
                if (obsTypeStr[0] == 'L') {
                    double wavelength = 0.0;

                    // 获取观测值频率，比如L1C，其频率为1
                    int n;
                    if (obsTypeStr[1] == 'A') {
                        n = 1;
                    } else {
                        n = stoi(obsTypeStr.substr(1, 1));
                    }

                    wavelength = getWavelength(sat.system, n);

                    if (wavelength == 0.0) continue;

                    if (debug) {
                        std::cout << obsTypeStr << " wavelength"
                                  << std::setprecision(12)
                                  << wavelength << std::endl;
                    }

                    // 将周期转换为米
                    data = data * wavelength;
                }

                // 观测值异常
                if (std::abs(data) == 0.0) {
                    continue;
                }

                typeObs[obsTypeStr] = data;
            }

            // 插入当前卫星的数据到 stvData
            stvData[satIndex[isv]] = typeObs;

        }
    }

    ObsData obsData;
    obsData.station = rinexHeader.station;
    obsData.epoch = currEpoch;
    obsData.satTypeValueData = stvData;

    return obsData;
}


/**
 * 函数：parseTime
 * 功能：从RINEX历元行解析时间信息
 * 参数：
 *   line - RINEX历元行字符串
 * 返回值：CommonTime - 通用时间格式
 * 说明：
 *   1. 检查时间格式空格位置，检测文件损坏
 *   2. 解析年、月、日、时、分、秒
 *   3. 处理秒值异常情况（≥60.0的情况）
 *   4. 将历元时间转换为通用时间格式
 */
CommonTime parseTime(const string &line) {

    // check if the spaces are in the right place - an easy
    // way to check if there's corruption in the file
    if ((line[1] != ' ') || (line[6] != ' ') || (line[9] != ' ') ||
        (line[12] != ' ') || (line[15] != ' ') || (line[18] != ' ') ||
        (line[29] != ' ') || (line[30] != ' ')) {
        FFStreamError e("Invalid time format");
        throw (e);
    }

    // if there's no time, just return a bad time
    if (line.substr(2, 27) == string(27, ' '))
        return BEGINNING_OF_TIME;

    int year, month, day, hour, min;
    double sec;

    year = stoi(line.substr(2, 4));
    month = stoi(line.substr(7, 2));
    day = stoi(line.substr(10, 2));
    hour = stoi(line.substr(13, 2));
    min = stoi(line.substr(16, 2));
    sec = safeStod(line.substr(19, 11));

    // Real Rinex has epochs 'yy mm dd hr 59 60.0' surprisingly often.
    double ds = 0;
    if (sec >= 60.0) {
        ds = sec;
        sec = 0.0;
    }

    CommonTime ctime;
    CivilTime cv = CivilTime(year, month, day, hour, min, sec);
    ctime = CivilTime2CommonTime(cv);

    if (ds != 0)
        ctime = ctime + ds;

    return ctime;


}  // end parseTime

/**
 * 函数：chooseObs
 * 功能：根据系统类型过滤观测数据
 * 参数：
 *   obsData  - 输入/输出：观测数据，过滤后的数据将替换原数据
 *   sysTypes - 映射：系统类型 -> 允许的观测类型集合
 * 说明：
 *   1. 遍历所有卫星观测数据
 *   2. 检查卫星系统是否在sysTypes中
 *   3. 只保留允许的观测类型
 *   4. 如果卫星没有剩余观测值，则从结果中删除
 */
void chooseObs(ObsData &obsData, std::map<std::string, std::set<std::string>> &sysTypes) {
    SatTypeValueMap filteredSatTypeValueData;

    // Iterate over all satellite entries in satTypeValueData
    for (const auto &satEntry: obsData.satTypeValueData) {
        const auto &satId = satEntry.first;
        const auto &typeValueMap = satEntry.second;

        // Check if the satellite's system is in sysTypes
        auto itSys = sysTypes.find(satId.system);
        if (itSys != sysTypes.end()) { // If the system is found in sysTypes
            const auto &allowedTypes = itSys->second;
            TypeValueMap filteredTypeValueMap;

            // Filter the observations based on the allowed types
            for (const auto &typeValueEntry: typeValueMap) {
                if (allowedTypes.find(typeValueEntry.first) != allowedTypes.end()) {
                    filteredTypeValueMap.insert(typeValueEntry);
                }
            }

            // Only add the satellite entry if there are any remaining observations
            if (!filteredTypeValueMap.empty()) {
                filteredSatTypeValueData[satId] = std::move(filteredTypeValueMap);
            }
        }
    }

    // Replace the original data with the filtered data
    obsData.satTypeValueData.swap(filteredSatTypeValueData);
}

// L1C => L1
// L2W => L2
/**
 * 函数：convertObsType
 * 功能：转换观测类型标识符（简化命名）
 * 参数：
 *   obsData - 输入/输出：观测数据
 * 说明：
 *   1. 将观测类型标识符从3字符简化为2字符
 *   2. 例如：L1C => L1, L2W => L2
 *   3. 用于统一不同接收机的观测类型命名
 */
void convertObsType(ObsData &obsData) {
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

/**
 * 函数：writefileSatPos
 * 功能：将卫星位置信息写入文件（用于调试和可视化）
 * 参数：
 *   satXvtTransTime    - 卫星在发射时刻的位置速度时间信息
 *   satXvtTransTimeIF  - 无电离层组合处理后的卫星位置速度时间信息
 *   epoch              - 当前历元时间
 * 说明：
 *   1. 将数据写入固定路径的文件
 *   2. 输出原始数据、IF组合数据及其差值
 *   3. 主要用于调试和数据分析
 */
void writefileSatPos(std::map<SatID, Xvt> satXvtTransTime, std::map<SatID, Xvt> satXvtTransTimeIF, CivilTime epoch, const std::string& outputPath) {
    ofstream fout(outputPath, std::ios::out);
    if (!fout) {
        cerr << "Unable to open file for writing: " << outputPath << endl;
        return;
    }
    fout << std::fixed << std::setprecision(15);
    fout<<"epoch:"<<epoch<<endl;
    for (auto it:satXvtTransTime) {
        auto decide=satXvtTransTimeIF.find(it.first);
        if (decide==satXvtTransTimeIF.end()) {
            continue;
        }
        SatID satID=it.first;
        Xvt satXvt=it.second;
        Xvt satXvtIF=satXvtTransTimeIF[satID];
        Xvt diff;
        diff.x=satXvt.x-satXvtIF.x;
        diff.v=satXvt.v-satXvtIF.v;
        diff.clkbias=satXvt.clkbias-satXvtIF.clkbias;
        diff.clkdrift=satXvt.clkdrift-satXvtIF.clkdrift;
        diff.relcorr=satXvt.relcorr-satXvtIF.relcorr;
        fout << std::fixed << std::setprecision(15);

        // 输出一行数据，字段间用空格分隔
        fout <<satID<< ' '
             <<it.second<<std::endl;
        fout<<satID<<' '
            <<satXvtTransTimeIF[satID]<<std::endl;
        fout<<satID<<' '
            <<diff<<std::endl;

    }

};


/**
 * 函数：computeSatPos
 * 功能：计算所有卫星在发射时刻的位置
 * 参数：
 *   obsData  - 观测数据（输入/输出，删除无法计算位置的卫星）
 *   navStore - 导航星历存储对象
 *   IF       - 无电离层组合标志：0=单频，1=双频无电离层组合
 * 返回值：std::map<SatID, Xvt> - 卫星在发射时刻的位置速度时间信息
 * 说明：
 *   1. 遍历观测数据中的所有卫星
 *   2. 根据IF标志选择观测值：
 *      - IF=0：自动查找伪距观测值（支持GPS、BDS、Galileo、GLONASS、QZSS、IRNSS）
 *      - IF=1：使用双频无电离层组合（当前仅支持GPS和BDS）
 *   3. 调用computeAtTransmitTime计算发射时刻卫星位置
 *   4. 删除无法计算位置的卫星（观测值缺失、星历无效等）
 */
std::map<SatID, Xvt> computeSatPos(ObsData &obsData, RinexNavStore& navStore,int IF=0) {
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
            if (!IF) {
                // 单频模式：从观测数据中查找伪距观测值
                bool found = false;
                for (const auto& entry : stv.second) {
                    const std::string& type = entry.first;
                    // 查找伪距观测值（以C或P开头）
                    if (type.size() >= 2 && (type[0] == 'C' || type[0] == 'P')) {
                        codeType = type;
                        obs = entry.second;
                        found = true;
                        break;
                    }
                }
                
                if (!found) {
                    satRejectedSet.insert(sat);
                    continue;
                }
                
                if (debug) {
                    cout << "sat:" << sat << " obs:" << codeType << " value:" << obs << endl;
                }
            }
            else {
                string name_basic_string[]={"C1","C2","C5","C6","C7"};
                map<string,array<double,2>> select_code;
                int count=0;

                if (sat.system == "G"||sat.system=="C")
                    {

                        for (auto x:name_basic_string) {
                            if (count==2)
                                break;
                            try {
                                obs=stv.second.at(x);
                                select_code[x][0]=obs;
                                select_code[x][1]=pow(codeSelectFrequency(x),2);
                                count+=1;
                            }
                            catch (...) {

                            }
                        }


                    }
                else {
                    satRejectedSet.insert(sat);
                    continue;
                }
                if (count!=2) {
                    satRejectedSet.insert(sat);
                    continue;
                }
                auto fCode=select_code.begin();
                auto temp=fCode;
                auto lCode=++temp;
                obs=(fCode->second[0]*fCode->second[1]-lCode->second[0]*lCode->second[1])/(fCode->second[1]-lCode->second[1]);
            }

            // code obs


            // now, compute xvt
            try {
                xvt = computeAtTransmitTime(time, obs, sat, navStore,IF);
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



/**
 * 函数：computeAtTransmitTime
 * 功能：迭代计算卫星在信号发射时刻的位置（考虑卫星钟差和相对论效应）
 * 参数：
 *   tr       - 接收机接收时间
 *   pr       - 伪距观测值（米）
 *   sat      - 卫星ID
 *   navStore - 导航星历存储对象
 *   IF       - 无电离层组合标志
 * 返回值：Xvt - 卫星在发射时刻的位置、速度、钟差、钟漂、相对论改正
 * 说明：
 *   1. 初始发射时间 = 接收时间 - 光行时（pr/C_MPS）
 *   2. 迭代计算（2次迭代）：
 *      a. 获取卫星位置和钟差
 *      b. 应用TGD（群延迟）改正
 *      c. 修正发射时间：减去钟差和相对论效应
 *   3. 最终得到精确的发射时刻卫星状态
 */
Xvt computeAtTransmitTime(const CommonTime &tr,
                          const double &pr,
                          const SatID &sat,
                          RinexNavStore& navStore,int IF)
noexcept(false) {
    Xvt xvt;
    CommonTime tt;
    CommonTime transmit = tr;
    //cout<<"tr"<<tr<<endl;

    transmit -= pr / C_MPS;
    tt = transmit;
    //存储数据



    // 这里也可以用while循环来替换这里的迭代次数
    for (int i = 0; i < 2; i++) {
        xvt = navStore.getXvt(sat, tt);
        if (debug) {
            cout << "computeAtTransmitTime::xvt:" << endl;
            cout << xvt << endl;
            cout << "computeAtTransmitTime::tt:" << tt << endl;
        }
        //cout << xvt << endl;
        tt = transmit;
        //cout<<"tt:"<<tt<<endl;
        //correctTGD(xvt,sat,tt,IF,navStore);
        tt -= (xvt.clkbias + xvt.relcorr);

    }
    return xvt;
};

/**
 * 函数：codeSelectFrequency
 * 功能：根据观测码类型返回对应的频率值
 * 参数：
 *   code - 观测码类型（C1, C2, C5, C6, C7）
 * 返回值：double - 对应的频率值（Hz）
 * 说明：
 *   1. 用于无电离层组合计算中的频率权重
 *   2. 目前使用BDS频率定义，需根据实际系统扩展
 */
double codeSelectFrequency(const string& code) {
    static const std::map<std::string, double> freqMap = {
        {"C1", L1_FREQ_BDS},
        {"C2", L2_FREQ_BDS},
        {"C5", L5_FREQ_BDS},
        {"C6", L6_FREQ_BDS},
        {"C7", L7_FREQ_BDS}
    };
    
    auto it = freqMap.find(code);
    if (it != freqMap.end()) {
        return it->second;
    }
    
    if (debug) {
        cout << "No selective frequency for this code: " << code << endl;
    }
    throw InvalidRequest("Unknown code type in codeSelectFrequency: " + code);
}

/**
 * 函数：correctTGD
 * 功能：校正卫星钟群延迟（TGD）误差
 * 参数：
 *   xvt      - 输入/输出：卫星位置速度时间信息，将修正钟差
 *   sat      - 卫星ID
 *   epoch    - 当前历元时间
 *   IF       - 无电离层组合标志
 *   navStore - 导航星历存储对象
 * 说明：
 *   1. IF模式不支持TGD校正（应使用无电离层组合消除一阶电离层和TGD）
 *   2. GPS系统：直接使用TGD值
 *   3. BDS系统：使用TGD1和TGD2计算L1/L2频率的等效TGD
 *   4. 其他系统暂不支持
 */
void correctTGD(Xvt& xvt, SatID sat, CommonTime epoch,int IF,RinexNavStore& navStore) {
    if (IF) {
        if(debug)
            cout<<"correctTGD function don't support processing IF_group!!!"<<endl;
    }
    else {
        if (sat.system == "G") {
            NavEphGPS nav_eph_gps=navStore.findGPSEph(sat,epoch);
            xvt.clkbias-=nav_eph_gps.TGD;
            xvt.typeTGDData["C1"]=nav_eph_gps.TGD;
        }
        else if (sat.system == "C") {
            convertTimeSystem(epoch,TimeSystem::BDT);
            //cout<<"epoch:"<<epoch<<endl;
            NavEphBDS nav_eph_bds=navStore.findBDSEph(sat,epoch);
            double TGD1=nav_eph_bds.TGD1;
            //cout<<"TGD1:"<<TGD1<<endl;
            double TGD2=nav_eph_bds.TGD2;
            //cout<<"TGD2:"<<TGD2<<endl;

            double L1_2=pow(L1_FREQ_BDS,2);
            double L2_2=pow(L2_FREQ_BDS,2);
            double TGD=-L1_2*(TGD1-TGD2)/(L1_2-L2_2);
            xvt.clkbias-=TGD;
            xvt.typeTGDData["C2"]=TGD;
        }
        else {
            if(debug)
                cout<<"correctTGD function don't support processing other system except GPS and BDT!!!"<<endl;
        }
    }
}

/**
 * 函数：earthRotation
 * 功能：地球自转改正（将卫星位置从发射时刻旋转到接收时刻）
 * 参数：
 *   xyz              - 接收机位置（ECEF坐标系）
 *   satXvtTransTime  - 卫星在发射时刻的位置速度时间信息
 * 返回值：std::map<SatID, Xvt> - 旋转到接收时刻的卫星位置速度信息
 * 说明：
 *   1. 计算信号传播时间：卫星到接收机的距离 / 光速
 *   2. 计算地球自转角度：ω_earth * 传播时间
 *   3. 对卫星位置和速度进行Z轴旋转
 *   4. 保持Z坐标不变，仅旋转X-Y平面
 */
std::map<SatID, Xvt> earthRotation(Eigen::Vector3d &xyz,
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

/**
 * 函数：computeElevAzim
 * 功能：计算卫星相对于接收机的仰角和方位角
 * 参数：
 *   xyz           - 接收机位置（ECEF坐标系）
 *   satXvt        - 卫星位置速度信息
 *   tempElevData  - 输出：卫星仰角映射
 *   tempAzimData  - 输出：卫星方位角映射
 * 说明：
 *   1. 遍历所有卫星
 *   2. 调用elevation和azimuth函数计算仰角和方位角
 *   3. 结果存储在映射中供后续使用
 */
void computeElevAzim(Eigen::Vector3d& xyz,
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
/**
 * 函数：ionoDelay
 * 功能：计算所有卫星的电离层延迟
 * 参数：
 *   xyz           - 接收机位置（ECEF坐标系）
 *   epoch         - 当前历元时间
 *   satElevData   - 卫星仰角映射
 *   satAzimData   - 卫星方位角映射
 *   navStore      - 导航星历存储对象（包含电离层参数）
 *   sysTypes      - 可选参数，系统到观测类型的映射，用于确定实际使用的频率
 * 返回值：std::map<SatID, double> - 卫星电离层延迟映射（秒）
 * 说明：
 *   1. 遍历所有卫星
 *   2. 从导航星历获取Klobuchar模型参数（alpha, beta）
 *   3. 使用createWeekSecond创建对应系统的周秒对象
 *   4. 如果提供sysTypes，从其中提取观测类型（如C1、C2），并转换为频率编号获取频率
 *   5. 如果未提供sysTypes或获取频率失败，使用默认的L1频率
 *   6. 调用klobucharIonosphericCorrection计算延迟
 *   7. 支持GPS、BDS、Galileo、GLONASS、QZSS、IRNSS系统
 */
std::map<SatID, double> ionoDelay(Vector3d& xyz,
                                  CommonTime& epoch,
                                  std::map<SatID, double>& satElevData,
                                  std::map<SatID, double>& satAzimData,
                                  RinexNavStore& navStore,
                                  std::map<std::string, std::set<std::string>>* sysTypes)
{
    std::map<SatID, double> ionoDelayMap;

    // 卫星系统到时间系统的映射
    static const std::map<std::string, TimeSystem::SystemType> sysMap = {
        {"G", TimeSystem::GPS},
        {"C", TimeSystem::BDT},
        {"E", TimeSystem::GAL},
        {"R", TimeSystem::GLO},
        {"J", TimeSystem::QZS},
        {"I", TimeSystem::IRN}
    };

    for (auto& it : satElevData) {
        SatID sat = it.first;
        std::string sys = sat.system;
        
        // 获取时间系统
        auto sysIter = sysMap.find(sys);
        if (sysIter == sysMap.end()) {
            if(debug) cout << "Unknown system " << sys << endl;
            continue;
        }
        TimeSystem::SystemType tsType = sysIter->second;
        
        // 获取频率
        double freq = 0.0;
        if (sysTypes != nullptr) {
            auto typesIter = sysTypes->find(sys);
            if (typesIter != sysTypes->end()) {
                for (const std::string& type : typesIter->second) {
                    if (type.size() >= 2 && (type[0] == 'C' || type[0] == 'P')) {
                        int freqNum = std::stoi(type.substr(1));
                        freq = getFreq(sys, freqNum);
                        if (freq > 0) break;
                    }
                }
            }
        }
        if (freq <= 0) freq = getFreq(sys, 1);
        if (freq <= 0) {
            if(debug) cout << "Failed to get frequency for system " << sys << endl;
            continue;
        }
        
        // 创建周秒对象并转换时间
        std::unique_ptr<WeekSecond> ws(createWeekSecond(tsType));
        CommonTime epoch_converted = convertTimeSystem(epoch, TimeSystem(tsType));
        CommonTime2WeekSecond(epoch_converted, *ws);
        
        // 根据系统选择电离层模型和参数
        double ionodelay = 0.0;
        
        if (sys == "C") {
            // BDS: 使用Klobuchar模型，按SatID选择参数
            double alpha[4] = {0}, beta[4] = {0};
            auto itBDS = navStore.ionoCorrDataBDS.find(sat);
            
            // 如果当前卫星找不到参数，使用C02作为默认
            if (itBDS == navStore.ionoCorrDataBDS.end()) {
                SatID defaultSat("C02");
                itBDS = navStore.ionoCorrDataBDS.find(defaultSat);
            }
            
            if (itBDS != navStore.ionoCorrDataBDS.end()) {
                const auto& param = itBDS->second;
                if (param.hasAlpha) std::copy(param.alpha, param.alpha + 4, alpha);
                if (param.hasBeta) std::copy(param.beta, param.beta + 4, beta);
            }
            ionodelay = klobucharIonosphericCorrection(xyz, satElevData[sat], satAzimData[sat], alpha, beta, ws->getSOW(), sat, freq);
            
        } else if (sys == "E") {
            // Galileo: 使用NeQuick-G模型（3参数）
            double ai[3] = {0};
            auto galIter = navStore.ionoCorrData.find("GAL");
            if (galIter != navStore.ionoCorrData.end() && galIter->second.size() >= 3) {
                ai[0] = galIter->second[0];
                ai[1] = galIter->second[1];
                ai[2] = galIter->second[2];
            }
            // Galileo模型直接返回米，不需要乘以光速
            ionoDelayMap[sat] = GalileoIonosphericCorrection(xyz, satElevData[sat], satAzimData[sat], ai, freq);
            continue;  // 跳过后面的乘以光速操作
            
        } else {
            // GPS/QZSS/IRNSS/GLONASS: 使用Klobuchar模型，直接取第一个参数
            auto temp = navStore.ionoCorrData.begin();
            double alpha[4] = { temp->second[0], temp->second[1], temp->second[2], temp->second[3] };
            temp++;
            double beta[4]  = { temp->second[0], temp->second[1], temp->second[2], temp->second[3] };
            ionodelay = klobucharIonosphericCorrection(xyz, satElevData[sat], satAzimData[sat], alpha, beta, ws->getSOW(), sat, freq);
        }
        
        ionoDelayMap[sat] = ionodelay * C_MPS;
    }

    return ionoDelayMap;
}



/**
 * 函数：klobucharIonosphericCorrection
 * 功能：Klobuchar电离层模型校正计算
 * 参数：
 *   xyz    - 接收机位置（ECEF坐标系）
 *   elev   - 卫星仰角（度）
 *   azim   - 卫星方位角（度）
 *   alpha  - Klobuchar模型alpha参数数组[4]
 *   beta   - Klobuchar模型beta参数数组[4]
 *   tow    - 时间周内秒（GPS或BDS）
 *   sat    - 卫星ID（用于确定系统）
 *   freq   - 观测频率（Hz）
 * 返回值：double - 电离层延迟（秒）
 * 说明：
 *   1. 将接收机坐标转换为大地坐标
 *   2. 计算电离层穿刺点（IPP）位置
 *   3. 计算地磁纬度
 *   4. 计算当地时间
 *   5. 使用Klobuchar模型计算垂直延迟
 *   6. 应用倾斜因子和频率缩放
 *   7. 包含详细的调试输出（当debug=1且卫星为G10时）
 */
double klobucharIonosphericCorrection(Vector3d xyz,
                                      double elev,
                                      double azim,
                                      double alpha[4],
                                      double beta[4],
                                      double tow,
                                      SatID sat,
                                      double freq)
{
    XYZ coord = xyz;
    BLH blh;
    double RE;
    double h_ion;

    std::string sys = sat.system;

    // 使用工厂函数创建参考框架
    auto frame = ReferenceFrameFactory::create(sys);
    blh = xyz2blh(coord, *frame);
    RE = frame->getA();

    // 电离层高度（BDS使用375km，其他系统使用350km）
    h_ion = (sys == "C") ? 375000.0 : 350000.0;

    double latUser = blh(0);
    double lonUser = blh(1);
    double E = elev * PI / 180.0;
    double A = azim * PI / 180.0;

    // 地心角 ψ
    double psi = PI / 2.0 - E - asin((RE * cos(E)) / (RE + h_ion));

    // IPP 纬度
    double latIPP = asin(sin(latUser) * cos(psi) + cos(latUser) * sin(psi) * cos(A));

    // IPP 经度
    double lonIPP = lonUser + (sin(psi) * sin(A)) / cos(latIPP);

    // 地磁纬度
    const double phiP = 79.5 * PI/180.0;   // slightly north shift
    const double lamP = 288.0 * PI/180.0;  // slight west shift
    double latMag = asin(sin(latIPP) * sin(phiP) + cos(latIPP) * cos(phiP) * cos(lonIPP - lamP));
    double latMagDeg = latMag * 180.0 / PI;

    // 当地时间
    double t = 43200.0 * lonIPP / PI + tow;
    t = fmod(t, 86400.0);
    if (t < 0) t += 86400.0;

    // 幅度 AI
    double x = latMag / PI;
    double AI = alpha[0] + alpha[1] * x + alpha[2] * x * x + alpha[3] * x * x * x;
    if (AI < 0) AI = 0;

    // 周期
    double Pi = beta[0] + beta[1] * x + beta[2] * x * x + beta[3] * x * x * x;
    if (Pi < 72000.0) Pi = 72000.0;

    // 相位
    double XI = 2 * PI * (t - 50400.0) / Pi;

    // 倾斜因子
    double F = 1.0 / sqrt(1.0 - pow((RE * cos(E)) / (RE + h_ion), 2));

    // L1 电离层时间延迟
    double I1;
    if (fabs(XI) < PI / 2.0) {
        I1 = (5e-9 + AI * cos(XI)) * F;
    } else {
        I1 = 5e-9 * F;
    }

    // 频率缩放
    double f1;
    if (sys=="C")
        f1=L2_FREQ_BDS;
    else
        f1 = getFreq(sys, 1);

    double ionoDelay = I1 * (f1 * f1) / (freq * freq);

    // 调试输出（仅G10）
    if (debug && sat.toString() == "G10") {
        cout << "=============================================" << endl;
        cout << "              KLOBUCHAR  DEBUG                " << endl;
        cout << "=============================================" << endl;
        cout << left
             << "Sat          : " << sat << endl
             << "psi          : " << fixed << setprecision(8) << psi       << " rad" << endl
             << "latIPP       : " << fixed << setprecision(8) << latIPP    << " rad" << endl
             << "lonIPP       : " << fixed << setprecision(8) << lonIPP    << " rad" << endl
             << "latMagDeg    : " << fixed << setprecision(6) << latMagDeg << " deg" << endl
             << "t            : " << fixed << setprecision(6) << t         << " s"   << endl
             << "AI           : " << scientific << setprecision(10) << AI  << " s"   << endl
             << "Pi           : " << fixed << setprecision(6) << Pi        << " s"   << endl
             << "XI           : " << fixed << setprecision(8) << XI        << " rad" << endl
             << "F            : " << fixed << setprecision(8) << F         << endl
             << "I1           : " << scientific << setprecision(10) << I1  << " s"   << endl;
        cout << "=============================================" << endl;
    }

    return ionoDelay;
}

/**
 * 函数：GalileoIonosphericCorrection
 * 功能：Galileo NeQuick-G 电离层模型校正计算
 * 参数：
 *   xyz    - 接收机位置（ECEF坐标系）
 *   elev   - 卫星仰角（度）
 *   azim   - 卫星方位角（度）
 *   ai     - NeQuick-G模型参数数组[3]
 *   tow    - 时间周内秒
 *   sat    - 卫星ID
 *   freq   - 观测频率（Hz）
 * 返回值：
 *   电离层延迟（秒）
 */
double GalileoIonosphericCorrection(
    const Vector3d& xyz,
    double elev,
    double azim,
    double ai[3],
    double freq)
{
    XYZ coord = xyz;
    auto frame = ReferenceFrameFactory::create("E");

    BLH blh = xyz2blh(coord, *frame);

    const double RE   = frame->getA();
    const double HION = 450000.0;      // 450 km

    //--------------------------------------------------
    // User position
    //--------------------------------------------------
    double latUser = blh(0);           // rad
    double lonUser = blh(1);           // rad

    double E = elev * PI / 180.0;
    double A = azim * PI / 180.0;

    //--------------------------------------------------
    // IPP
    //--------------------------------------------------
    double psi =
        PI/2.0
      - E
      - asin(RE/(RE+HION)*cos(E));

    double latIPP =
        asin(
            sin(latUser)*cos(psi)
          + cos(latUser)*sin(psi)*cos(A)
        );

    double lonIPP =
        lonUser
      + sin(psi)*sin(A)/cos(latIPP);

    //--------------------------------------------------
    // Geomagnetic latitude
    //--------------------------------------------------
    const double PHI_P    = 78.3  * PI/180.0;
    const double LAMBDA_P = 291.0 * PI/180.0;

    double latMag =
        asin(
            sin(latIPP)*sin(PHI_P)
          + cos(latIPP)*cos(PHI_P)
          * cos(lonIPP - LAMBDA_P)
        );

    //--------------------------------------------------
    // Simplified Galileo Az
    //--------------------------------------------------
    double latMagDeg = latMag * 180.0 / PI;

    double Az =
          ai[0]
        + ai[1] * latMagDeg
        + ai[2] * latMagDeg * latMagDeg;

    if (Az < 0.0)
        Az = 0.0;

    //--------------------------------------------------
    // Simplified VTEC model
    //
    // Typical:
    // Az=50  -> 35 TECU
    // Az=100 -> 70 TECU
    // Az=150 ->105 TECU
    // Az=200 ->140 TECU
    //--------------------------------------------------
    double VTEC = std::max(5.0, 0.35* Az);

    //--------------------------------------------------
    // Mapping Function
    //--------------------------------------------------
    double MF =
        1.0 /
        sqrt(
            1.0 -
            pow(RE/(RE+HION)*cos(E), 2)
        );

    //--------------------------------------------------
    // Slant TEC
    //--------------------------------------------------
    double STEC = VTEC * MF;

    //--------------------------------------------------
    // Ionospheric delay
    //
    // 1 TECU =
    // 1e16 electrons/m²
    //--------------------------------------------------
    double ionoDelay =
        40.3e16 * STEC /
        (freq * freq);

    return ionoDelay;
}
/**
 * 函数：tropDelay
 * 功能：计算所有卫星的对流层延迟
 * 参数：
 *   xyz         - 接收机位置（ECEF坐标系）
 *   satElevData - 卫星仰角映射
 *   RH          - 相对湿度（百分比）
 * 返回值：std::map<SatID,double> - 卫星对流层延迟映射（米）
 * 说明：
 *   1. 遍历所有卫星
 *   2. 根据卫星系统选择对应的大地基准
 *   3. 调用saastamoinenTroposphericCorrection计算延迟
 *   4. 支持GPS、BDS、Galileo、GLONASS、QZSS、IRNSS系统
 */
std::map<SatID,double> tropDelay(Vector3d& xyz, std::map<SatID, double>&
satElevData,double RH)
{
    std::map<SatID,double> tropDelaymap;

    for (auto it : satElevData) {

        if (it.second < 15.0 ) {
            continue;
        }

        std::string sys = it.first.system;
        BLH blh;

        if (sys == "G") {
            GPSEllipsoid ell;
            blh = xyz2blh(xyz, ell);
        }
        else if (sys == "C") {
            BDSEllipsoid ell;
            blh = xyz2blh(xyz, ell);
        }
        else if (sys == "E") {
            GPSEllipsoid ell;  // Galileo使用WGS-84
            blh = xyz2blh(xyz, ell);
        }
        else if (sys == "R") {
            PZ90 ell;  // GLONASS使用PZ-90
            blh = xyz2blh(xyz, ell);
        }
        else if (sys == "J") {
            GPSEllipsoid ell;  // QZSS使用WGS-84
            blh = xyz2blh(xyz, ell);
        }
        else if (sys == "I") {
            WGS84 ell;  // IRNSS使用WGS-84
            blh = xyz2blh(xyz, ell);
        }
        else {
            if(debug)
                cout << "The tropDelay function don't support system " << sys << endl;
            continue;
        }

        double tropDelay = saastamoinenTroposphericCorrection(blh, it.second, it.first, RH);
        tropDelaymap[it.first] = tropDelay;
    }
    return tropDelaymap;
}


/**
 * 函数：saastamoinenTroposphericCorrection
 * 功能：Saastamoinen对流层模型校正计算
 * 参数：
 *   geoUser - 接收机大地坐标（纬度、经度、高程，度/米）
 *   elev    - 卫星仰角（度）
 *   sat_id  - 卫星ID（仅用于调试输出）
 *   RH      - 相对湿度（百分比）
 * 返回值：double - 对流层延迟（米）
 * 说明：
 *   1. 基于标准大气模型计算气压、温度、水汽压
 *   2. 计算天顶干延迟（ZHD）和天顶湿延迟（ZWD）
 *   3. 使用简单的映射函数（1/sin(elev)）
 *   4. 包含详细的调试输出（当debug=1且卫星为G10时）
 */
double saastamoinenTroposphericCorrection(
    const Vector3d& geoUser,
    double elev_deg,
    const SatID& sat_id,
    double RH_in)
{
    // ==============================
    // 1. 基本参数
    // ==============================
    double B   = geoUser(0);   // 纬度 (rad)
    double H_m = geoUser(2);   // 高程 (m)

    if (elev_deg < 5.0)   // cutoff（GNSS标准做法）
        return NAN;

    double E = elev_deg * PI / 180.0;

    // ==============================
    // 2. RH 统一到 0~1
    // ==============================
    double RH = RH_in;
    if (RH > 1.0) RH *= 0.01;   // 兼容输入 0~100

    RH = std::clamp(RH, 0.0, 1.0);

    // ==============================
    // 3. 标准大气模型
    // ==============================
    double H_km = H_m / 1000.0;

    double P = 1013.25 * pow(1.0 - 0.0000226 * H_m, 5.225);
    double T_c = 15.0 - 0.0065 * H_m;
    double T_k = T_c + 273.15;

    // 水汽压（Tetens）
    double es = 6.1078 * exp(17.27 * T_c / (T_c + 237.3));
    double e  = RH * es;

    // ==============================
    // 4. Saastamoinen ZHD / ZWD
    // ==============================
    double fBH = 1.0 - 0.00266 * cos(2.0 * B) - 0.00028 * H_km;
    double ZHD = 2.277e-3 * P / fBH;

    double ZWD = 0.002277 * (1255.0 / T_k + 0.05) * e;

    // ==============================
    // 5. 改进 mapping function（稳定版）
    // ==============================
    double sinE = sin(E);
    double tanE = tan(E);

    // 防止极低仰角数值爆炸
    sinE = std::max(sinE, 0.05);

    double mf_d = 1.0 / (sinE + 0.00143 / (tanE + 0.0445));
    double mf_w = 1.0 / (sinE + 0.00035 / (tanE + 0.017));

    // ==============================
    // 6. 总延迟
    // ==============================
    double tropoDelay = ZHD * mf_d + ZWD * mf_w;

    // ==============================
    // 7. 数值保护（不再“归零”！）
    // ==============================
    if (!std::isfinite(tropoDelay) || tropoDelay > 20.0)
        return NAN;

    // ==============================
    // 8. Debug
    // ==============================
    if (debug && sat_id.toString() == "G10")
    {
        cout << "sat: " << sat_id << endl;
        cout << "E (deg): " << elev_deg << endl;
        cout << "P: " << P << " hPa" << endl;
        cout << "T: " << T_c << " C" << endl;
        cout << "RH: " << RH << endl;
        cout << "ZHD: " << ZHD << endl;
        cout << "ZWD: " << ZWD << endl;
        cout << "Tropo: " << tropoDelay << endl;
    }

    return tropoDelay;
}
/**
 * 函数：wavelengthOfMW
 * 功能：计算MW（Melbourne-Wübbena）组合的波长
 * 参数：
 *   sys     - 卫星系统标识
 *   L1Type - L1观测类型标识
 *   L2Type - L2观测类型标识
 * 返回值：double - MW组合波长（米）
 * 说明：
 *   1. MW组合 = (f1*L1 - f2*L2)/(f1 - f2) - (f1*P1 + f2*P2)/(f1 + f2)
 *   2. 波长 = 光速 / (f1 - f2)
 *   3. 用于周跳探测中的阈值计算
 */
double wavelengthOfMW(string sys, string L1Type, string L2Type) {
    double f1 = getFreq(sys, L1Type);
    double f2 = getFreq(sys, L2Type);
    double wavelength = C_MPS / (f1 - f2);
    return wavelength;
};

/**
 * 函数：varOfMW
 * 功能：计算MW组合的初始方差（简化模型）
 * 参数：
 *   L1Type - L1观测类型标识
 *   L2Type - L2观测类型标识
 * 返回值：double - MW组合初始方差
 * 说明：
 *   1. 使用固定值：sqrt(2)/2 * 0.3
 *   2. 简化模型，实际应用中应根据观测噪声调整
 */
double varOfMW(string, string L1Type, string L2Type) {
    double var = sqrt(2.0) / 2 * 0.3;
    return var;
};

/**
 * 函数：detectCSMW
 * 功能：使用MW（Melbourne-Wübbena）组合进行周跳探测
 * 参数：
 *   obsData               - 输入/输出：观测数据，删除无法探测周跳的卫星
 *   csFlagData            - 输出：周跳标志映射（Variable -> 标志）
 *   satEpochMWData        - 输出：卫星历元MW值映射（用于绘图分析）
 *   satEpochMeanMWData    - 输出：卫星历元平均MW值映射
 *   satEpochCSFlagData    - 输出：卫星历元周跳标志映射（放大到MW值便于绘图）
 * 说明：
 *   1. MW组合 = (f1*L1 - f2*L2)/(f1 - f2) - (f1*P1 + f2*P2)/(f1 + f2)
 *   2. 使用滑动窗口统计计算MW值的均值和方差
 *   3. 周跳判断条件：
 *      a. 数据中断时间超过阈值（deltaTMax）
 *      b. 当前MW值与均值之差超过最小周期数（minCycles * 波长）
 *      c. 当前MW值与均值之差超过4倍标准差
 *   4. 检测到周跳时重置滑动窗口统计量
 *   5. 未检测到周跳时更新均值和方差
 *   6. 将周跳标志存储到模糊度变量中
 *   7. 目前仅支持GPS系统，需扩展BDS支持
 */
void detectCSMW(ObsData &obsData,
                std::map<Variable, int> &csFlagData,
                SatEpochValueMap &satEpochMWData,
                SatEpochValueMap &satEpochMeanMWData,
                SatEpochValueMap &satEpochCSFlagData) {
    //==================
    // 初始化常数和static变量
    //==================
    double deltaTMax(120.0);
    double minCycles(2.0);

    // A structure used to store filter data for a SV.
    struct MWData {
        // Default constructor initializing the data in the structure
        MWData()
                : formerEpoch(BEGINNING_OF_TIME), windowSize(0), meanMW(0.0), varMW(0.0) {};

        CommonTime formerEpoch; ///< The previous epoch time stamp.
        int windowSize;         ///< Size of current window, in samples.
        double meanMW;          ///< Accumulated mean value of combination.
        double varMW;           ///< Accumulated std value of combination.
    };

    // 这个数据在下次调用时需要用到，所以定位为static变量
    static std::map<SatID, MWData> satMWData;

    //==========================
    // 逐个卫星做周跳探测
    //==========================
    // Loop through all the satellites
    CommonTime currentEpoch = obsData.epoch;
    SatIDSet badSatSet;
    for (auto stv: obsData.satTypeValueData) {
        SatID sat = (stv).first;
        string L1Type, L2Type, C1Type, C2Type;
        if (sat.system == "G") {
            L1Type = "L1";
            L2Type = "L2";
            C1Type = "C1";
            C2Type = "C2";
        } else // 请增加bds的处理
        {
            badSatSet.insert(sat);
        }

        // wavelengthMW of MW-combination, see LinearCombination
        double wavelengthMW = wavelengthOfMW(sat.system, L1Type, L2Type);
        double varianceMW = varOfMW(sat.system, L1Type, L2Type);

        double f1 = getFreq(sat.system, L1Type);
        double f2 = getFreq(sat.system, L2Type);

        if (debug) {
            cout << "f1:" << f1 << "f2:" << f2 << endl;
        }

        double L1Value, L2Value, C1Value, C2Value, mwValue;

        try {
            L1Value = stv.second.at(L1Type);
            L2Value = stv.second.at(L2Type);
            C1Value = stv.second.at(C1Type);
            C2Value = stv.second.at(C2Type);

            mwValue
                    = (f1 * L1Value - f2 * L2Value) / (f1 - f2)
                      - (f1 * C1Value + f2 * C2Value) / (f1 + f2);
        } catch (std::out_of_range) {
            // 无法构成mw，这个卫星观测值周跳无法探测，删除这个卫星
            badSatSet.insert(sat);
            continue; // 继续处理下一个卫星
        }

        satEpochMWData[sat][currentEpoch] = mwValue;

        if (debugCSMW) {
            cout << "L1Value:" << L1Value << endl;
            cout << "L2Value:" << L2Value << endl;
            cout << "C1Value:" << C1Value << endl;
            cout << "C2Value:" << C2Value << endl;
            cout << "mwValue:" << mwValue << endl;
            cout << "wavelength:" << C_MPS / (f1 - f2) << endl;
        }

        //-------------------
        double currentDeltaT(0.0);
        double currentBias(0.0);
        int csFlag(0.0);

        currentDeltaT = (currentEpoch - satMWData[sat].formerEpoch);
        satMWData[sat].formerEpoch = currentEpoch;
        if (debugCSMW) {
            cout << "currentDeltaT:" << currentDeltaT << endl;
        }
        // Difference between current value of MW and average value
        currentBias = std::abs(mwValue - satMWData[sat].meanMW);
        if (debugCSMW) {
            cout << "currentBias:" << currentBias << endl;
        }

        // Increment window size
        satMWData[sat].windowSize++;

        /**
         * cycle-slip condition
         * 1. if data interrupt for a given time gap, then cyce slip should be set
         * 2. if current bias is greater than 1 cycle and greater than 4 sigma of mean mw.
         */
        double sigLimit = 4 * std::sqrt(satMWData[sat].varMW);

        if (debugCSMW) {
            cout << "deltaTMax:" << deltaTMax << endl;
            cout << "wavelengthMW:" << wavelengthMW << endl;
            cout << "sigLimit:" << sigLimit << endl;
            cout << "minCycles:" << minCycles * wavelengthMW << endl;
        }

        // 波长有可能为负值
        if (currentDeltaT > deltaTMax ||
            currentBias > std::abs(minCycles * wavelengthMW) ||
            currentBias > sigLimit) {

            // reset the filter window size/meanMW/InitialVarofMW
            satMWData[sat].meanMW = mwValue;
            satMWData[sat].varMW = varianceMW;
            satMWData[sat].windowSize = 1;

            if (debugCSMW) {
                cout << "* CS happened!" << endl;
            }
            csFlag = 1.0;
        } else {
            // MW bias from the mean value
            double mwBias(mwValue - satMWData[sat].meanMW);
            double size(static_cast<double>(satMWData[sat].windowSize));

            // Compute average
            satMWData[sat].meanMW += mwBias / size;

            // Compute variance
            // Var(i) = Var(i-1) + [ ( mw(i) - meanMW)^2/(i)- 1*Var(i-1) ]/(i);
            satMWData[sat].varMW += (mwBias * mwBias - satMWData[sat].varMW) / size;
        }

        // for print
        satEpochMeanMWData[sat][currentEpoch] = satMWData[sat].meanMW;

        // 放大到mw数值，以方便绘图
        satEpochCSFlagData[sat][currentEpoch] = csFlag * mwValue;

        // 将周跳探测标志存到模糊度变量中
        Variable amb1(obsData.station, sat, static_cast<Parameter>(Parameter::ambiguity), ObsID(sat.system, L1Type));
        Variable amb2(obsData.station, sat, static_cast<Parameter>(Parameter::ambiguity), ObsID(sat.system, L2Type));

        csFlagData[amb1] = csFlag;
        csFlagData[amb2] = csFlag;
    }

    // 删除坏卫星
    for (auto sat: badSatSet)
        obsData.satTypeValueData.erase(sat);

};

void differenceStation(EquSys& equSysRover,
                       EquSys& equSysBase,
                       EquSys& equSysSD)
{
    // 逐个观测值类型取出类型
    std::map<EquID, EquData> obsEquDataDiff;
    VariableSet varSetDiff;
    for(auto& oe: equSysRover.obsEquData)
    {
        // 在参考站中查找当前观测值，如果没有找到就跳过;
        // 需要注意测站名是不同的， 只需要查找卫星号和观测值
        if(equSysBase.obsEquData.find(oe.first)==equSysBase.obsEquData.end())
        {
            continue;
        }

        // 取出参考站的线性化残差观测值
        EquData oeBase = equSysBase.obsEquData.at(oe.first);

        // 在参考站中查找类型的观测值，找到了就计算站间差分观测值
        double diffPrefit;
        diffPrefit = oe.second.prefit - oeBase.prefit;

        if(debug)
        {
            cout << ">>>>>>>>> differenceStation" << endl;
            cout << oe.first
            << "rover: " << oe.second.prefit
            << "base:  " << oeBase.prefit
            << "diffPrefit:" << diffPrefit
            << endl;
        }

        // 把当前卫星的tvDiff数据插入到gDataDiff;
        obsEquDataDiff[oe.first].prefit = diffPrefit;

        //------------------------------------------------------------
        // 因为对于短基线来说，可以不用估计电离层和对流程，
        // 这里为了简单起见，直接将电离层和对流层参数从站间差分未知参数表中删除
        // todo:
        // 更优雅的处理方式是在参数估计时，对电离层和电离层进行约束，
        // 并根据基线长度对约束的方差进行动态调整。
        // 比如：
        // iono = 0, sigmaIono = 0.001*0.001*baseline
        // trop = 0, sigmaTrop = 0.0001*.0001*baseline
        // 通过增加电离层、对流层约束方程，实现通用rtk定位模型
        //------------------------------------------------------------
        // 未知参数与流动站的参数是相同的。
        std::map<Variable, double> vcDataTemp;
        for(auto vc: oe.second.varCoeffData)
        {
            if(vc.first.getParaType() != Parameter::iono)
            {
                vcDataTemp[vc.first] = vc.second;
                varSetDiff.insert(vc.first);
            }
        }
        obsEquDataDiff[oe.first].varCoeffData = vcDataTemp;

        // 权函数
        double weightRover = oe.second.weight;
        double weightBase = oeBase.weight;
        double varDiff = 1.0/weightRover + 1.0/weightBase;
        obsEquDataDiff[oe.first].weight = 1.0/varDiff;
    }

    equSysSD.obsEquData = obsEquDataDiff;
    equSysSD.varSet = varSetDiff;

};



SatID findDatumSat(bool& firstEpoch,
                   SatValueMap& satElevData) {
    // 确定基准卫星，必须是上一个历元已经固定的卫星才能选作基准
    SatID datumSat;
    auto maxIt = max_element(satElevData.begin(),satElevData.end(),
                              [](const auto& a, const auto& b){ return a.second < b.second; } );
    datumSat = maxIt->first;
    return datumSat;
}

void differenceSat( SatID& datumSat,
                    EquSys& equSysSD,
                    EquSys& equSysDD ) {
    //----------------------------------------------------
    // 根据基准卫星，选择每个观测类型的观测值，并将其他的与基准卫星对应观测值求差
    // warning:
    // 因为星间单差需要消除接收机钟差和接收机端硬件延迟，
    // 因此必须为每个类型独立构建星间单差观测方程，而不能混合在一起；
    // 因此，基准观测值的方程数据，应该存在以观测类型为key键值的map中，
    // 由于观测类型我们采用了C1，C2，L1，L2作为名字，
    // 当采用GPS+BDS时，两个系统均存在L2，无法有效区分，
    // 因此，这里需要创建一个独立的数据结构ObsID来管理观测类型ID，
    // 其由两个成员构成，一个是obsType；一个是卫星系统system
    // 另一个简单的处理：
    // string obsStr = obsType + system;
    //----------------------------------------------------

    std::map<ObsID, EquData> datumEquData;
    std::map<EquID, EquData> otherEquData;

    std::map<EquID, EquData> equData;
    equData = equSysSD.obsEquData;
    if (debug)
    {
        cout << "differenceSat:" << endl;
        cout << "datumSat:" << datumSat << endl;
    }

    for(auto ed: equData)
    {
        if(ed.first.sat == datumSat)
        {
            ObsID obsID(ed.first.sat.system, ed.first.obsType);
            if(debug)
                cout << "datum obsid:" << obsID << endl;

            datumEquData[obsID] = ed.second;
        }
        else
        {
            otherEquData[ed.first] = ed.second;
        }
    }

    // dd
    // 先验残差求差；
    // dx，dy，dz的系数求差；
    // 接收机钟差进一步差分掉了；
    // 模糊度除了基准卫星，其他卫星变成双差模式，系数不变
    std::map<EquID, EquData> equDataDD;
    VariableSet varSetDD;
    for(auto ed: otherEquData)
    {
        // 先验残差
        // 需要在基准ObsID里找EquData，来构成星间差分，
        // 如果找不到就剔除这个卫星；
        // 因此需要捕获异常，来处理找不到的情况；
        cout << "differenceSat:" << "sat:" << ed.first.sat << endl;
        double prefitDatum;
        ObsID currentObsID = ObsID(ed.first.sat.system, ed.first.obsType);
        try {
            prefitDatum= datumEquData.at(currentObsID).prefit;

            // dd prefit
            double prefitDD = ed.second.prefit - prefitDatum;
            equDataDD[ed.first].prefit = prefitDD;

            // 系数与未知参数
            VariableDataMap vcDatum = datumEquData.at(currentObsID).varCoeffData;

            // 接收机钟差消除了，只保留了坐标和模糊度参数
            for(auto vc: ed.second.varCoeffData)
            {
                if( vc.first.getParaType()==Parameter::dX ||
                    vc.first.getParaType()==Parameter::dY ||
                    vc.first.getParaType()==Parameter::dZ )
                {
                    double coeffDiff;
                    coeffDiff = vc.second - vcDatum.at(vc.first);
                    equDataDD[ed.first].varCoeffData[vc.first] = coeffDiff;
                    varSetDD.insert(vc.first);
                }
                else if(vc.first.getParaType()==Parameter::ambiguity)
                {
                    equDataDD[ed.first].varCoeffData[vc.first] = vc.second;
                    varSetDD.insert(vc.first);
                }
            }

            // 双差的方差近似等于单差观测值的方差的和；
            double weightCurrent = ed.second.weight;
            double weightDatum = datumEquData.at(currentObsID).weight;
            double varDiff = 1.0/weightCurrent + 1.0/weightDatum;
            equDataDD[ed.first].weight = 1.0/varDiff;

            //
            // todo
            // 构建完整的方差协方差阵，并比较定位结果的不同
        }
        catch(...)
        {
            continue;
        }
    }
    equSysDD.obsEquData = equDataDD;
    equSysDD.varSet = varSetDD;
};

// 对于Kalman滤波来说，需要对流动站和参考站周跳进行周跳标识符的合并，
// 只要流动站和参考站对应频率模糊度有一个发生了周跳就需要对周跳进行合并；
// 可以通过重载函数来实现对现有函数功能的复用。
void differenceStation(EquSys& equSysRover, VariableDataMap& csFlagRover,
                       EquSys& equSysBase, VariableDataMap& csFlagBase,
                       EquSys& equSysSD, VariableDataMap& csFlagSD)
{
    differenceStation(equSysRover,
                      equSysBase,
                      equSysSD);

    string roverStation = equSysRover.station;

    // todo:
    // 去掉流动站和参考站的站名，否则无法查找并匹配周跳
    VariableDataMap tempFlagRover;
    for(auto vd: csFlagRover)
    {
        Variable tempVar = vd.first;
        tempVar.station = std::string("");
        tempFlagRover[tempVar] = vd.second;
    }

    // 去掉基准站名字
    VariableDataMap tempFlagBase;
    for(auto vd: csFlagBase)
    {
        Variable tempVar = vd.first;
        tempVar.station = std::string("");
        tempFlagBase[tempVar] = vd.second;
    }

    // 现在，从基准站中寻找流动站模糊度，如果找到了，就把周跳标志合并
    // 如果没找到，就跳过，说明无法形成站间差分观测
    for(auto vd: tempFlagRover) {
        double flagRover = vd.second;
        if (tempFlagBase.find(vd.first) != tempFlagBase.end())
        {
            double flagBase = tempFlagBase.at(vd.first);
            double flagSD(0.0);
            // 基准站或者流动站一个发生周跳，就标志周跳
            if(flagRover|| flagBase)
            {
                flagSD = 1.0;
            }
            Variable varSD = vd.first; // 得到流动站模糊度变量
            varSD.station = roverStation; // 把流动站名站再次赋值进来
            csFlagSD[varSD] = flagSD;
        }
    }

    // 单差周跳
    if(debug) {
        cout << "differenceStation:" << "csFlagRover:" << endl;
        for(auto cd:csFlagRover)
        {
            cout << "cs:" << cd.first << " flag:" << cd.second;
        }

        cout << "differenceStation:" << "csFlagBase:" << endl;
        for(auto cd:csFlagBase)
        {
            cout << "cs:" << cd.first << " flag:" << cd.second;
        }

        cout << "differenceStation:" << "csFlagSD:" << endl;
        for(auto cd:csFlagSD)
        {
            cout << "cs:" << cd.first << " flag:" << cd.second;
        }
    }

};

void differenceSat( SatID& datumSat,
                    EquSys& equSysSD, VariableDataMap& csFlagSD,
                    EquSys& equSysDD, VariableDataMap& csFlagDD ) {
    //----------------------------------------------------
    // 根据基准卫星，选择每个观测类型的观测值，并将其他的与基准卫星对应观测值求差
    // warning:
    // 因为星间单差需要消除接收机钟差和接收机端硬件延迟，
    // 因此必须为每个类型独立构建星间单差观测方程，而不能混合在一起；
    // 因此，基准观测值的方程数据，应该存在以观测类型为key键值的map中，
    // 由于观测类型我们采用了C1，C2，L1，L2作为名字，
    // 当采用GPS+BDS时，两个系统均存在L2，无法有效区分，
    // 因此，这里需要创建一个独立的数据结构ObsID来管理观测类型ID，
    // 其由两个成员构成，一个是obsType；一个是卫星系统system
    // 另一个简单的处理：
    // string obsStr = obsType + system;
    //----------------------------------------------------

    std::map<ObsID, EquData> datumEquData;
    std::map<EquID, EquData> otherEquData;

    std::map<EquID, EquData> equData;
    equData = equSysSD.obsEquData;

    for(auto ed: equData)
    {
        if(ed.first.sat == datumSat)
        {
            ObsID obsID(ed.first.sat.system, ed.first.obsType);
            if(debug)
                cout << "datum obsid:" << obsID << endl;

            datumEquData[obsID] = ed.second;
        }
        else
        {
            otherEquData[ed.first] = ed.second;
        }
    }

    // dd
    // 先验残差求差；
    // dx，dy，dz的系数求差；
    // 接收机钟差进一步差分掉了；
    // 模糊度除了基准卫星，其他卫星变成双差模式，系数不变
    std::map<EquID, EquData> equDataDD;
    VariableSet varSetDD;
    for(auto ed: otherEquData)
    {
        // 先验残差
        // 需要在基准ObsID里找EquData，来构成星间差分，
        // 如果找不到就剔除这个卫星；
        // 因此需要捕获异常，来处理找不到的情况；
        cout << "differenceSat:" << "sat:" << ed.first.sat << endl;
        double prefitDatum;
        ObsID currentObsID = ObsID(ed.first.sat.system, ed.first.obsType);
        try {
            prefitDatum= datumEquData.at(currentObsID).prefit;

            // dd prefit
            double prefitDD = ed.second.prefit - prefitDatum;
            equDataDD[ed.first].prefit = prefitDD;

            // 系数与未知参数
            VariableDataMap vcDatum = datumEquData.at(currentObsID).varCoeffData;

            // 接收机钟差消除了，只保留了坐标和模糊度参数
            for(auto vc: ed.second.varCoeffData)
            {
                if( vc.first.getParaType()==Parameter::dX ||
                    vc.first.getParaType()==Parameter::dY ||
                    vc.first.getParaType()==Parameter::dZ )
                {
                    double coeffDiff;
                    coeffDiff = vc.second - vcDatum.at(vc.first);
                    equDataDD[ed.first].varCoeffData[vc.first] = coeffDiff;
                    varSetDD.insert(vc.first);
                }
                else if(vc.first.getParaType()==Parameter::ambiguity)
                {
                    std::pair<Variable, double> ambData;
                    for(auto vc2: vcDatum)
                    {
                        if(vc2.first.getParaType() == Parameter::ambiguity)
                        {
                            ambData.first = vc2.first;
                            ambData.second = vc2.second;
                        }
                    }
                    // 把基准模糊度插入到方程中，也就是估计基准模糊度，而不是合并成双差模糊度
                    // warning: 是负号
                    equDataDD[ed.first].varCoeffData[ambData.first] = -ambData.second;
                    equDataDD[ed.first].varCoeffData[vc.first] = vc.second;
                    varSetDD.insert(ambData.first);
                    varSetDD.insert(vc.first);
                }
            }

            // 双差的方差近似等于单差观测值的方差的和；
            double weightCurrent = ed.second.weight;
            double weightDatum = datumEquData.at(currentObsID).weight;
            double varDiff = 1.0/weightCurrent + 1.0/weightDatum;
            equDataDD[ed.first].weight = 1.0/varDiff;

            //
            // todo
            // 构建完整的方差协方差阵，并比较定位结果的不同
        }
        catch(...)
        {
            continue;
        }
    }
    equSysDD.obsEquData = equDataDD;
    equSysDD.varSet = varSetDD;

    // 直接把站间单差模糊度标志给双差即可，因为估计的模糊度仍然为站间单差模糊度
    csFlagDD = csFlagSD;

};

void fixSolution(VectorXd& stateVec,
                 MatrixXd& covMatrix,
                 VariableSet& varSet,
                 double& ratio,
                 Vector3d& dxyzFixed,
                 VariableDataMap& fixedAmbData)
{
    // 按照卫星把模糊度进行分类；
    VariableSet ambVarSet;
    for (auto var: varSet) {
        if(var.getParaType()==Parameter::ambiguity)
        {
            ambVarSet.insert(var);
        }
    };

    int numAmb = ambVarSet.size();
    int numXYZT = varSet.size() - numAmb;

    // 取出来星间差分模糊度ambVarSetSD的估值和方差，利用lambda方法固定
    VectorXd ambSol;
    MatrixXd ambCov;
    ambSol = stateVec.tail(numAmb);
    ambCov = covMatrix.block(numXYZT, numXYZT, numAmb, numAmb);

    if (debug) {
        cout << "ambVarSet" << endl;
        for (auto var: ambVarSet) {
            cout << var << " ";
        }
        cout << endl;

        cout << "ambSol" << endl;
        cout << ambSol.transpose() << endl;

        cout << "ambCov" << endl;
        cout << ambCov << endl;
    }

    // 如果ratio值大于3，则表明模糊度可以固定。
    ARLambda arLambda;
    VectorXd ambSolFixed = arLambda.resolve(ambSol, ambCov);

    int iamb=0;
    fixedAmbData.clear();
    for(auto var:ambVarSet)
    {
        fixedAmbData[var] = ambSolFixed(iamb);
        iamb++;
    }

    ratio = arLambda.squaredRatio;

    if (debug) {
        cout << "ratio:" << endl;
        cout << ratio << endl;
    }

    VectorXd xVecFixed;
    VectorXd xVec = VectorXd::Zero(numXYZT);

    // x/y/z
    xVec = stateVec.head(numXYZT);

    MatrixXd Qxx = covMatrix.block(0, 0, numXYZT, numXYZT);
    MatrixXd Qxb = covMatrix.block(0, numXYZT, numXYZT, numAmb);
    MatrixXd Qbb = covMatrix.block(numXYZT, numXYZT, numAmb, numAmb);

    if (debug) {
        cout << fixed << setprecision(5) << endl;

        cout << "covMatrix" << endl;
        cout << covMatrix << endl;

        cout << "Qxx" << endl;
        cout << Qxx << endl;

        cout << "Qxb" << endl;
        cout << Qxb << endl;

        cout << "Qbb" << endl;
        cout << Qbb << endl;
    }

    xVecFixed = xVec - Qxb * Qbb.inverse() * (ambSol - ambSolFixed);

    // return fixed solutions
    dxyzFixed = xVecFixed;

};



void ambiguityDatum(bool& firstEpoch,
                    SatID& datumSat,
                    VariableDataMap& fixedAmbData,
                    EquSys& equSysDD){

    // 对于第一个历元，直接将基准卫星模糊度固定为零即可。
    if(firstEpoch)
    {
        for(auto var:equSysDD.varSet)
        {
            if(var.getSat() == datumSat)
            {
                EquID equIDDatum;
                equIDDatum.sat = datumSat;
                equIDDatum.obsType = var.getParaType().toString()
                                     + var.getObsID().toString();

                EquData equDataDatum;
                equDataDatum.prefit = 0.0;
                equDataDatum.varCoeffData[var] = 1.0;
                equDataDatum.weight = 1.0E+8;

                // 将模糊度基准观测方程加入到观测系统中
                equSysDD.obsEquData[equIDDatum] = equDataDatum;
            }
        }
    }
    else
    {
        for(auto vd: fixedAmbData)
        {
            // 生成模糊度基准的观测方程
            if(vd.first.getSat()==datumSat)
            {
                EquID equIDDatum;
                equIDDatum.sat = datumSat;
                equIDDatum.obsType = vd.first.getParaType().toString()
                                     + vd.first.getObsID().toString();

                EquData equDataDatum;
                equDataDatum.prefit = vd.second;
                equDataDatum.varCoeffData[vd.first] = 1.0;
                equDataDatum.weight = 1.0E+8;

                // 将模糊度基准观测方程加入到观测系统中
                equSysDD.obsEquData[equIDDatum] = equDataDatum;
            }
        }
    }
};

// print solution to files
void printSolution(std::fstream & solStream,
                   CommonTime& ctTime,
                   Eigen::Vector3d& xyzRover,
                   Eigen::Vector3d& xyzRTKFloat,
                   double& ratio,
                   Eigen::Vector3d& xyzRTKFixed)
{
    YDSTime ydsTime = CommonTime2YDSTime(ctTime);
    solStream
            << ydsTime
            << fixed << setprecision(3)
            << "spp: " << xyzRover.transpose()
            << " float-rtk: " << xyzRTKFloat.transpose()
            << " ratio:" << ratio
            << " fixed-rtk:"<< xyzRTKFixed.transpose()
            << endl;
};

// print solution to files
void printSolution(std::fstream & solStream,
                   CommonTime& ctTime,
                   Eigen::Vector3d& xyzRover,
                   Eigen::Vector3d& xyzRTKFloat)
{
    YDSTime ydsTime = CommonTime2YDSTime(ctTime);
    solStream
    << ydsTime
    << fixed << setprecision(3)
    << "spp: " << xyzRover.transpose()
    << " rtk: " << xyzRTKFloat.transpose() << endl;
};

void printSolution(std::fstream & solStream,
                   CommonTime& ctTime,
                   Eigen::Vector3d& xyzRover)
{
    YDSTime ydsTime = CommonTime2YDSTime(ctTime);
    solStream
    << ydsTime
    << " "
    << fixed << setprecision(3)
    << xyzRover.transpose() << endl;
};

double getTGD(SatID sat_id,RinexNavStore navstore,std::string obsID,CommonTime& ctTime) {
    double tgd = 0.0;
    if (sat_id.system=="G") {
        NavEphGPS nav_eph_gps=navstore.findGPSEph(sat_id,ctTime);
        if (obsID=="C1") {
            double gamma=pow(L1_FREQ_GPS/L2_FREQ_GPS,2);
            tgd=nav_eph_gps.TGD*gamma;
        }
    }
    else if (sat_id.system=="C") {
        NavEphBDS nav_eph_bds=navstore.findBDSEph(sat_id,ctTime);
        if (obsID=="C2") {
            tgd=nav_eph_bds.TGD1;
        }
        else if (obsID=="C7") {
            tgd=nav_eph_bds.TGD2;
        }
    }
    return tgd;
}