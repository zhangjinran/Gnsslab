//
// Created by zhang on 2026/3/14.
//
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>
#include "CoordStruct.h"
#include "CoordConvert.h"


int main() {

        // 示例 XYZ 坐标
        XYZ xyz_north_pole(0.0, 0.0, 6356752.314);  // 北极点
        XYZ xyz_south_pole(0.0, 0.0, -6356752.314); // 南极点
        XYZ xyz_normal(4081945.67, 2187689.34, 4767321.89); // 正常点
        PZ90 pz90;
        BLH blh_north_pole=xyz2blh(xyz_north_pole,pz90);
        std::cout << blh_north_pole << std::endl<<std::endl;
        BLH blh_south_pole=xyz2blh(xyz_south_pole,pz90);
        std::cout << blh_south_pole << std::endl<<std::endl;
        BLH blh_normal=xyz2blh(xyz_normal,pz90);
        std::cout << blh_normal << std::endl<<std::endl;
        XYZ xyz_enu_north_pole=blh2ENU(blh_north_pole,pz90,xyz_south_pole);
        std::cout << xyz_enu_north_pole << std::endl<<std::endl;
        std::cout << ENU2BLH(xyz_enu_north_pole,pz90,xyz_south_pole) << std::endl<<std::endl;
        return 0;
}