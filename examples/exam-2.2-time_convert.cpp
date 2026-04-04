
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

#include <chrono>

#include "TimeStruct.h"
#include "TimeConvert.h"
#include"Time2020Convert.h"










int main() {
//civiltime是一个类，
    CivilTime civilTime(2025, 1, 4, 9, 0, 0.0);


    // convert yy/mm/dd to jd
    double jd;

    int yy = 2025;
    int month = 1;
    int day = 1;
    jd = convertYMD2JD(2025, 1, 1);

    cout << "jd for yy/mm/dd: " << yy << "/" << month << "/" << day << " is:" << fixed << jd << endl;

    JulianDate julianDate;
    CommonTime commonTime = CivilTime2CommonTime(civilTime);
    cout << "CommonTime is:" << commonTime << endl;


    julianDate = CommonTime2JulianDate(commonTime);

    cout << "julianData is:" << fixed << julianDate << endl;


    TimeSystem UTC("UTC");
    CommonTime test1(49169);
    std::cout << UTC.toString() << std::endl;
    std::cout <<"GPSToUTC:"<< convertTimeSystem(test1,UTC) << std::endl;


    BDTWeekSecond test2(2,800);
    CommonTime ct2;
    WeekSecond2CommonTime(test2,ct2);
    long test_day;
    double test_sow;
    TimeSystem time_system=TimeSystem::BDT;
    ct2.get(test_day,test_sow,time_system);
    std::cout << "day:"<<test_day<<".second:" <<test_sow<< std::endl;

    CommonTime ct2020(0);
    JulianDate jd1(0);
    jd1=CommonTime20202JulianDate(ct2020);
    std::cout << jd1 << std::endl;

    CivilTime gt2020(0,0,0);
    gt2020=CommonTime20202CivilTime(ct2020);
    std::cout << gt2020 << std::endl;

    CommonTime ct_mjd;
    ct_mjd=CommonTime20202CommonTime(ct2020);
    std::cout << ct_mjd << std::endl;

    return 0;
}