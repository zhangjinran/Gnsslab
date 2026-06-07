/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *
 * Author: Shoujian Zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 */

#include "NavEphBase.hpp"

std::map<std::string, NavEphFactory::CreatorFunc>& NavEphFactory::getCreators() {
    static std::map<std::string, CreatorFunc> creators;
    return creators;
}

std::unique_ptr<NavEphBase> NavEphFactory::create(const std::string& system) {
    auto& creators = getCreators();
    auto it = creators.find(system);
    if (it != creators.end()) {
        return it->second();
    }
    return nullptr;
}

void NavEphFactory::registerCreator(const std::string& system, CreatorFunc func) {
    getCreators()[system] = func;
}

std::vector<std::string> NavEphFactory::getSupportedSystems() {
    std::vector<std::string> systems;
    for (const auto& entry : getCreators()) {
        systems.push_back(entry.first);
    }
    return systems;
}

bool NavEphFactory::isSupported(const std::string& system) {
    return getCreators().find(system) != getCreators().end();
}

