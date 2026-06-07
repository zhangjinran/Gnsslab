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

#include <iostream>
#include <string>
#include <stdexcept>
#include "GnssStruct.h"

using namespace std;

int main() {
    // Test normal cases
    SatID sat1("G15");
    SatID sat2("C03");

    cout << "Satellite ID: " << sat1 << endl;
    cout << "Satellite ID: " << sat2 << endl;

    if (sat1 < sat2) {
        cout << "sat1 is less than sat2" << endl;
    } else {
        cout << "sat1 is not less than sat2" << endl;
    }

    if (sat1 == sat2) {
        cout << "sat1 and sat2 are equal" << endl;
    } else {
        cout << "sat1 and sat2 are not equal" << endl;
    }

    // Test exception cases
    cout << "\n--- Testing exception handling ---" << endl;

    // Test 1: Invalid format (too short)
    try {
        SatID sat3("G");
        cout << "ERROR: Should have thrown exception for 'G'" << endl;
    } catch (const invalid_argument& e) {
        cout << "Caught expected exception: " << e.what() << endl;
    }

    // Test 2: Invalid system identifier
    try {
        SatID sat4("X15");
        cout << "ERROR: Should have thrown exception for 'X15'" << endl;
    } catch (const invalid_argument& e) {
        cout << "Caught expected exception: " << e.what() << endl;
    }

    // Test 3: Invalid satellite ID (non-numeric)
    try {
        SatID sat5("GXX");
        cout << "ERROR: Should have thrown exception for 'GXX'" << endl;
    } catch (const invalid_argument& e) {
        cout << "Caught expected exception: " << e.what() << endl;
    }

    // Test 4: Satellite ID out of range
    try {
        SatID sat6("G100");
        cout << "ERROR: Should have thrown exception for 'G100'" << endl;
    } catch (const invalid_argument& e) {
        cout << "Caught expected exception: " << e.what() << endl;
    }

    // Test 5: Negative satellite ID
    try {
        SatID sat7("G-1");
        cout << "ERROR: Should have thrown exception for 'G-1'" << endl;
    } catch (const invalid_argument& e) {
        cout << "Caught expected exception: " << e.what() << endl;
    }

    // Test BDS generation field
    cout << "\n--- Testing BDS Generation ---" << endl;
    
    // BDS-2 satellites (C01-C14)
    SatID bds2("C03");
    cout << "C03 generation: " << bds2.generation << " (expected: 1 for BDS-2)" << endl;
    
    SatID bds2_14("C14");
    cout << "C14 generation: " << bds2_14.generation << " (expected: 1 for BDS-2)" << endl;
    
    // BDS-3 satellites (C19+)
    SatID bds3("C20");
    cout << "C20 generation: " << bds3.generation << " (expected: 2 for BDS-3)" << endl;
    
    SatID bds3_19("C19");
    cout << "C19 generation: " << bds3_19.generation << " (expected: 2 for BDS-3)" << endl;
    
    // Reserved satellites (C15-C18)
    SatID reserved("C15");
    cout << "C15 generation: " << reserved.generation << " (expected: 0 for unknown)" << endl;
    
    // Non-BDS satellite
    cout << "G15 generation: " << sat1.generation << " (expected: 0 for non-BDS)" << endl;

    // Test assignment operator
    cout << "\n--- Testing Assignment Operator ---" << endl;
    SatID sat_assign("C20");
    SatID sat_target;
    
    cout << "Before assignment: sat_target = " << sat_target << ", generation = " << sat_target.generation << endl;
    sat_target = sat_assign;
    cout << "After assignment:  sat_target = " << sat_target << ", generation = " << sat_target.generation << endl;
    
    if (sat_target == sat_assign) {
        cout << "Assignment successful: sat_target == sat_assign" << endl;
    } else {
        cout << "ERROR: Assignment failed" << endl;
    }

    return 0;
}