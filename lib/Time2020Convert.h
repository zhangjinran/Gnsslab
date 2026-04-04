//
// Created by zhang on 2026/3/14.
//


#pragma once

#include "TimeStruct.h"
#include "TimeConvert.h"






CommonTime CivilTime2CommonTime2020(const CivilTime &civilt);
CivilTime CommonTime20202CivilTime(const CommonTime &ct);
CommonTime JulianDate2CommonTime2020(JulianDate &jd);
JulianDate CommonTime20202JulianDate(CommonTime &ct);
void CommonTime20202JD2020(const CommonTime& ct, JD2020& jd2020 );
void JD20202CommonTime2020(JD2020& jd2020, CommonTime& ct);
CommonTime YDSTime2CommonTime2020(YDSTime &ydst);
YDSTime CommonTime20202YDSTime(const CommonTime &ct);
void CommonTime20202WeekSecond(const CommonTime& ct, WeekSecond& wk);
void WeekSecond2CommonTime2020(WeekSecond& wk, CommonTime& ct);
CommonTime CommonTime20202CommonTime(CommonTime &ct);
CommonTime CommonTime2CommonTime2020(CommonTime &ct) ;
