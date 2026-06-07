/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *
 * Author: Shoujian Zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * Description: 统一星历接口基类
 * 参考项目中现有的工厂模式设计（ReferenceFrameFactory、WeekSecond工厂）
 */

#ifndef NavEphBase_HPP
#define NavEphBase_HPP

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <functional>
#include "GnssStruct.h"
#include "TimeStruct.h"

class NavEphBase {
public:
    virtual ~NavEphBase() = default;

    virtual Xvt svXvt(const CommonTime& t) const = 0;
    virtual Xvt svXvt(const CommonTime& t, const SatID& sat) const {
        return svXvt(t);
    }
    virtual double svClockBias(const CommonTime& t) const = 0;
    virtual double svClockDrift(const CommonTime& t) const = 0;
    virtual long double svRelativity(const CommonTime& t) const = 0;
    virtual long double svRelativity(const CommonTime& t, Eigen::Vector3d r, Eigen::Vector3d v) const {
        // 默认实现：忽略位置速度参数，调用无参数版本
        return svRelativity(t);
    }
    virtual double svURA(const CommonTime& t) const = 0;
    virtual bool isValid(const CommonTime& ct) const = 0;
    virtual void printData() const = 0;

    virtual TimeSystem getTimeSystem() const = 0;
    virtual std::string getSystemCode() const = 0;

    CommonTime ctToc;
    CommonTime ctToe;
    CommonTime transmitTime;
    CommonTime beginValid;
    CommonTime endValid;
};

class NavEphFactory {
public:
    using CreatorFunc = std::function<std::unique_ptr<NavEphBase>()>;

    static std::unique_ptr<NavEphBase> create(const std::string& system);
    static void registerCreator(const std::string& system, CreatorFunc func);
    static std::vector<std::string> getSupportedSystems();
    static bool isSupported(const std::string& system);

private:
    static std::map<std::string, CreatorFunc>& getCreators();
};

#endif // NavEphBase_HPP