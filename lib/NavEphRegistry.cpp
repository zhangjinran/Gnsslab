/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *
 * Author: Shoujian Zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 */

#include "NavEphBase.hpp"
#include "NavEphGPS.hpp"
#include "NavEphBDS.hpp"
#include "NavEphGLONASS.hpp"
#include "NavEphGalileo.hpp"
#include "NavEphQZSS.hpp"
#include "NavEphIRNSS.hpp"

namespace {
    struct Registry {
        Registry() {
            NavEphFactory::registerCreator("G", []() {
                return std::make_unique<NavEphGPS>();
            });
            
            NavEphFactory::registerCreator("C", []() {
                return std::make_unique<NavEphBDS>();
            });
            
            NavEphFactory::registerCreator("R", []() {
                return std::make_unique<NavEphGLONASS>();
            });
            
            NavEphFactory::registerCreator("E", []() {
                return std::make_unique<NavEphGalileo>();
            });
            
            NavEphFactory::registerCreator("J", []() {
                return std::make_unique<NavEphQZSS>();
            });
            
            NavEphFactory::registerCreator("I", []() {
                return std::make_unique<NavEphIRNSS>();
            });
        }
    };
    
    static Registry registry;
}

void ensureNavEphRegistered() {
}