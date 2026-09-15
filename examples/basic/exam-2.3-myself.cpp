//
// Created by zhang on 2026/3/14.
//
#include <iostream>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <iomanip>
#include <Eigen/Dense>
#include <gnsslab/CoordStruct.h>
#include <gnsslab/CoordConvert.h>
#include <gnsslab/DataExporter.h>

int main() {
    std::cout << std::fixed << std::setprecision(6);

    // 示例 XYZ 坐标
    XYZ xyz_north_pole(0.0, 0.0, 6356752.314);  // 北极点
    XYZ xyz_south_pole(0.0, 0.0, -6356752.314); // 南极点
    XYZ xyz_normal(4081945.67, 2187689.34, 4767321.89); // 正常点
    PZ90 pz90;

    // ========== 测试1: XYZ -> BLH 转换 ==========
    std::cout << "===== XYZ -> BLH 转换测试 =====" << std::endl;
    BLH blh_north_pole = xyz2blh(xyz_north_pole, pz90);
    std::cout << "北极点 (弧度): B=" << blh_north_pole.B() 
              << ", L=" << blh_north_pole.L() 
              << ", H=" << blh_north_pole.H() << "m" << std::endl;
    std::cout << "北极点 (角度): B=" << rad2deg(blh_north_pole.B()) << "°" 
              << ", L=" << rad2deg(blh_north_pole.L()) << "°" 
              << ", H=" << blh_north_pole.H() << "m" << std::endl << std::endl;

    BLH blh_south_pole = xyz2blh(xyz_south_pole, pz90);
    std::cout << "南极点 (弧度): B=" << blh_south_pole.B() 
              << ", L=" << blh_south_pole.L() 
              << ", H=" << blh_south_pole.H() << "m" << std::endl;
    std::cout << "南极点 (角度): B=" << rad2deg(blh_south_pole.B()) << "°" 
              << ", L=" << rad2deg(blh_south_pole.L()) << "°" 
              << ", H=" << blh_south_pole.H() << "m" << std::endl << std::endl;

    BLH blh_normal = xyz2blh(xyz_normal, pz90);
    std::cout << "正常点 (弧度): B=" << blh_normal.B() 
              << ", L=" << blh_normal.L() 
              << ", H=" << blh_normal.H() << "m" << std::endl;
    std::cout << "正常点 (角度): B=" << rad2deg(blh_normal.B()) << "°" 
              << ", L=" << rad2deg(blh_normal.L()) << "°" 
              << ", H=" << blh_normal.H() << "m" << std::endl << std::endl;

    // ========== 测试2: BLH -> XYZ 反向转换 ==========
    std::cout << "===== BLH -> XYZ 反向转换测试 =====" << std::endl;
    XYZ xyz_back = blh2xyz(blh_normal, pz90);
    std::cout << "原始XYZ: (" << xyz_normal.X() << ", " << xyz_normal.Y() << ", " << xyz_normal.Z() << ")" << std::endl;
    std::cout << "转换后XYZ: (" << xyz_back.X() << ", " << xyz_back.Y() << ", " << xyz_back.Z() << ")" << std::endl;
    std::cout << "差异: (" << xyz_normal.X() - xyz_back.X() << ", " 
              << xyz_normal.Y() - xyz_back.Y() << ", " 
              << xyz_normal.Z() - xyz_back.Z() << ")" << std::endl << std::endl;

    // ========== 测试3: ENU 转换 ==========
    std::cout << "===== ENU 转换测试 =====" << std::endl;
    XYZ xyz_enu_north_pole = blh2ENU(blh_north_pole, pz90, xyz_south_pole);
    std::cout << "北极点 ENU(以南极点为参考): E=" << xyz_enu_north_pole.X() 
              << ", N=" << xyz_enu_north_pole.Y() 
              << ", U=" << xyz_enu_north_pole.Z() << std::endl;

    BLH blh_enu_back = ENU2BLH(xyz_enu_north_pole, pz90, xyz_south_pole);
    std::cout << "ENU 转回 BLH (角度): B=" << rad2deg(blh_enu_back.B()) << "°" 
              << ", L=" << rad2deg(blh_enu_back.L()) << "°" 
              << ", H=" << blh_enu_back.H() << "m" << std::endl << std::endl;

    // ========== 测试4: 弧度/角度转换函数 ==========
    std::cout << "===== 弧度/角度转换函数测试 =====" << std::endl;
    double rad = std::numbers::pi / 4; // 45度
    double deg = rad2deg(rad);
    std::cout << rad << " 弧度 = " << deg << " 度" << std::endl;
    
    double deg_test = 180.0;
    double rad_test = deg2rad(deg_test);
    std::cout << deg_test << " 度 = " << rad_test << " 弧度" << std::endl;

    // 验证转换可逆性
    double rad_back = deg2rad(rad2deg(rad));
    std::cout << "转换可逆性验证: " << rad << " -> deg -> rad = " << rad_back 
              << ", 误差 = " << fabs(rad - rad_back) << std::endl << std::endl;

    // ========== 测试5: 多导航系统参考框架测试 ==========
    std::cout << "===== 多导航系统参考框架测试 =====" << std::endl;
    
    WGS84 wgs84;
    GPSEllipsoid gps;
    BDSEllipsoid bds;
    PZ90 pz90_new;
    Galileo galileo;
    GPSEllipsoid qzss;
    IRNSS irnss;

    std::cout << "各系统椭球参数对比:" << std::endl;
    std::cout << "WGS84:    a=" << wgs84.getA() << "m, f=" << wgs84.getF() << ", GM=" << wgs84.getGM() << std::endl;
    std::cout << "GPS:      a=" << gps.getA() << "m, f=" << gps.getF() << ", GM=" << gps.gm() << std::endl;
    std::cout << "BDS:      a=" << bds.getA() << "m, f=" << bds.getF() << ", GM=" << bds.gm() << std::endl;
    std::cout << "GLONASS:  a=" << pz90_new.getA() << "m, f=" << pz90_new.getF() << ", GM=" << pz90_new.getGM() << std::endl;
    std::cout << "Galileo:  a=" << galileo.getA() << "m, f=" << galileo.getF() << ", GM=" << galileo.getGM() << std::endl;
    std::cout << "QZSS:     a=" << qzss.getA() << "m, f=" << qzss.getF() << ", GM=" << qzss.getGM() << std::endl;
    std::cout << "IRNSS:    a=" << irnss.getA() << "m, f=" << irnss.getF() << ", GM=" << irnss.getGM() << std::endl << std::endl;

    // 使用不同系统进行坐标转换
    std::cout << "不同参考框架下的坐标转换结果:" << std::endl;
    
    BLH blh_wgs84 = xyz2blh(xyz_normal, wgs84);
    std::cout << "WGS84    : B=" << rad2deg(blh_wgs84.B()) << "°, L=" << rad2deg(blh_wgs84.L()) << "°, H=" << blh_wgs84.H() << "m" << std::endl;
    
    BLH blh_gps = xyz2blh(xyz_normal, gps);
    std::cout << "GPS      : B=" << rad2deg(blh_gps.B()) << "°, L=" << rad2deg(blh_gps.L()) << "°, H=" << blh_gps.H() << "m" << std::endl;
    
    BLH blh_bds = xyz2blh(xyz_normal, bds);
    std::cout << "BDS      : B=" << rad2deg(blh_bds.B()) << "°, L=" << rad2deg(blh_bds.L()) << "°, H=" << blh_bds.H() << "m" << std::endl;
    
    BLH blh_glonass = xyz2blh(xyz_normal, pz90_new);
    std::cout << "GLONASS  : B=" << rad2deg(blh_glonass.B()) << "°, L=" << rad2deg(blh_glonass.L()) << "°, H=" << blh_glonass.H() << "m" << std::endl;
    
    BLH blh_galileo = xyz2blh(xyz_normal, galileo);
    std::cout << "Galileo  : B=" << rad2deg(blh_galileo.B()) << "°, L=" << rad2deg(blh_galileo.L()) << "°, H=" << blh_galileo.H() << "m" << std::endl;
    
    BLH blh_qzss = xyz2blh(xyz_normal, qzss);
    std::cout << "QZSS     : B=" << rad2deg(blh_qzss.B()) << "°, L=" << rad2deg(blh_qzss.L()) << "°, H=" << blh_qzss.H() << "m" << std::endl;
    
    BLH blh_irnss = xyz2blh(xyz_normal, irnss);
    std::cout << "IRNSS    : B=" << rad2deg(blh_irnss.B()) << "°, L=" << rad2deg(blh_irnss.L()) << "°, H=" << blh_irnss.H() << "m" << std::endl << std::endl;

    // ========== 测试6: 参考框架工厂模式测试 ==========
    std::cout << "===== 参考框架工厂模式测试 =====" << std::endl;

    // 获取支持的系统列表
    std::vector<std::string> systems = ReferenceFrameFactory::getSupportedSystems();
    std::cout << "支持的系统列表: ";
    for (const auto& sys : systems) {
        std::cout << sys << " ";
    }
    std::cout << std::endl << std::endl;

    // 测试工厂创建函数
    std::cout << "使用工厂创建参考框架并进行转换:" << std::endl;
    
    std::vector<std::string> testSystems = {"GPS", "BDS", "GLONASS", "Galileo", "QZSS", "IRNSS", "WGS84", "unknown"};
    for (const auto& sys : testSystems) {
        auto frame = ReferenceFrameFactory::create(sys);
        BLH blh = xyz2blh(xyz_normal, *frame);
        std::cout << sys << "     : B=" << rad2deg(blh.B()) << "°, L=" << rad2deg(blh.L()) << "°, H=" << blh.H() << "m";
        if (!ReferenceFrameFactory::isSupported(sys)) {
            std::cout << " (使用默认WGS84)";
        }
        std::cout << std::endl;
    }

    // 测试大小写不敏感
    std::cout << std::endl << "测试大小写不敏感:" << std::endl;
    auto frame_lower = ReferenceFrameFactory::create("gps");
    auto frame_upper = ReferenceFrameFactory::create("GPS");
    BLH blh_lower = xyz2blh(xyz_normal, *frame_lower);
    BLH blh_upper = xyz2blh(xyz_normal, *frame_upper);
    std::cout << "gps (小写): H=" << blh_lower.H() << "m" << std::endl;
    std::cout << "GPS (大写): H=" << blh_upper.H() << "m" << std::endl;

    // ========== 测试7: 坐标转换互逆性验证（关键测试）==========
    std::cout << std::endl << "===== 坐标转换互逆性验证 =====" << std::endl;
    
    // 使用用户提到的参考坐标 (-2267750.275, 5009154.471, 3221294.345)
    XYZ refXYZ(-2267750.275, 5009154.471, 3221294.345);
    std::cout << "测试坐标 (接收机参考位置):" << std::endl;
    std::cout << "  X = " << refXYZ.X() << " m" << std::endl;
    std::cout << "  Y = " << refXYZ.Y() << " m" << std::endl;
    std::cout << "  Z = " << refXYZ.Z() << " m" << std::endl << std::endl;
    
    // 测试不同参考框架下的互逆性
    std::vector<std::string> sysNames = {"GPS", "BDS", "GLONASS", "Galileo", "QZSS", "IRNSS", "WGS84"};
    std::cout << "=== xyz2blh -> blh2xyz 互逆性测试 ===" << std::endl;
    std::cout << "系统        X差异(m)    Y差异(m)    Z差异(m)    范数(m)" << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;
    
    for (const auto& sys : sysNames) {
        auto frame = ReferenceFrameFactory::create(sys);
        
        // XYZ -> BLH -> XYZ
        BLH blh = xyz2blh(refXYZ, *frame);
        XYZ xyzBack = blh2xyz(blh, *frame);
        
        // 计算差异
        double dx = xyzBack.X() - refXYZ.X();
        double dy = xyzBack.Y() - refXYZ.Y();
        double dz = xyzBack.Z() - refXYZ.Z();
        double norm = sqrt(dx*dx + dy*dy + dz*dz);
        
        std::cout << std::left << std::setw(10) << sys 
                  << std::fixed << std::setprecision(10) 
                  << std::setw(12) << dx 
                  << std::setw(12) << dy 
                  << std::setw(12) << dz 
                  << std::setw(12) << norm << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "=== blh2xyz -> xyz2blh 互逆性测试 ===" << std::endl;
    std::cout << "系统        B差异(rad)  L差异(rad)  H差异(m)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    BLH blhTest(deg2rad(30.5), deg2rad(114.3), 50.0); // 武汉附近典型坐标
    for (const auto& sys : sysNames) {
        auto frame = ReferenceFrameFactory::create(sys);
        
        // BLH -> XYZ -> BLH
        XYZ xyz = blh2xyz(blhTest, *frame);
        BLH blhBack = xyz2blh(xyz, *frame);
        
        // 计算差异
        double dB = blhBack.B() - blhTest.B();
        double dL = blhBack.L() - blhTest.L();
        double dH = blhBack.H() - blhTest.H();
        
        std::cout << std::left << std::setw(10) << sys 
                  << std::fixed << std::setprecision(15) 
                  << std::setw(12) << dB 
                  << std::setw(12) << dL 
                  << std::setw(12) << dH << std::endl;
    }
    
    // ========== 测试8: ENU计算链路验证 ==========
    std::cout << std::endl << "===== ENU计算链路验证 =====" << std::endl;
    
    // 模拟一个真实的接收机位置（武汉大学附近）
    XYZ roverXYZ(-2267750.275, 5009154.471, 3221294.345);
    XYZ baseXYZ(-2267750.0, 5009154.0, 3221294.0); // 参考点
    
    std::cout << "流动站坐标: X=" << roverXYZ.X() << ", Y=" << roverXYZ.Y() << ", Z=" << roverXYZ.Z() << std::endl;
    std::cout << "基准站坐标: X=" << baseXYZ.X() << ", Y=" << baseXYZ.Y() << ", Z=" << baseXYZ.Z() << std::endl;
    
    // 测试不同框架下的ENU计算
    std::cout << std::endl << "=== ENU计算结果对比 ===" << std::endl;
    std::cout << "系统        E(m)       N(m)       U(m)" << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    
    for (const auto& sys : sysNames) {
        auto frame = ReferenceFrameFactory::create(sys);
        
        // 方法1: 直接使用xyz计算ENU（理论正确方法）
        BLH blhRover = xyz2blh(roverXYZ, *frame);
        XYZ enu1 = blh2ENU(blhRover, *frame, baseXYZ);
        
        // 方法2: 先转BLH再转回XYZ再计算（模拟可能的误差路径）
        BLH blh = xyz2blh(roverXYZ, *frame);
        XYZ xyzBack = blh2xyz(blh, *frame);
        BLH blhBack = xyz2blh(xyzBack, *frame);
        XYZ enu2 = blh2ENU(blhBack, *frame, baseXYZ);
        
        // 计算两种方法的差异
        double dE = enu2.X() - enu1.X();
        double dN = enu2.Y() - enu1.Y();
        double dU = enu2.Z() - enu1.Z();
        
        std::cout << std::left << std::setw(10) << sys 
                  << std::fixed << std::setprecision(6) 
                  << std::setw(10) << enu1.X() 
                  << std::setw(10) << enu1.Y() 
                  << std::setw(10) << enu1.Z() << std::endl;
    }
    
    // 检查关键的互逆性误差
    std::cout << std::endl << "=== 关键检查: 参考点自转换误差 ===" << std::endl;
    auto gpsFrame = ReferenceFrameFactory::create("GPS");
    BLH blhRef = xyz2blh(refXYZ, *gpsFrame);
    XYZ xyzBackRef = blh2xyz(blhRef, *gpsFrame);
    
    double dxRef = xyzBackRef.X() - refXYZ.X();
    double dyRef = xyzBackRef.Y() - refXYZ.Y();
    double dzRef = xyzBackRef.Z() - refXYZ.Z();
    double normRef = sqrt(dxRef*dxRef + dyRef*dyRef + dzRef*dzRef);
    
    std::cout << "参考坐标: (" << refXYZ.X() << ", " << refXYZ.Y() << ", " << refXYZ.Z() << ")" << std::endl;
    std::cout << "转换回XYZ: (" << xyzBackRef.X() << ", " << xyzBackRef.Y() << ", " << xyzBackRef.Z() << ")" << std::endl;
    std::cout << "差异: dx=" << dxRef << "m, dy=" << dyRef << "m, dz=" << dzRef << "m" << std::endl;
    std::cout << "差异范数: " << normRef << " m" << std::endl;
    std::cout << "精度评估: ";
    if (normRef < 1e-4) {
        std::cout << "✓ 毫米级精度，符合预期" << std::endl;
    } else if (normRef < 0.1) {
        std::cout << "⚠ 厘米级误差，可能存在问题" << std::endl;
    } else if (normRef < 1.0) {
        std::cout << "⚠ 分米级误差，需要检查" << std::endl;
    } else {
        std::cout << "✗ 米级误差，存在严重问题!" << std::endl;
    }
    
    // ========== 测试9: 导出坐标系统数据 ==========
    std::cout << std::endl << "===== 导出坐标系统数据 =====" << std::endl;
    
    std::vector<ReferenceFrame*> frames;
    frames.push_back(new WGS84());
    frames.push_back(new GPSEllipsoid());
    frames.push_back(new BDSEllipsoid());
    frames.push_back(new PZ90());
    frames.push_back(new Galileo());
    frames.push_back(new GPSEllipsoid());
    frames.push_back(new IRNSS());
    
    if (gnss::DataExporter::exportCoordConversionData(xyz_normal, frames)) {
        std::cout << "✓ 坐标转换数据导出成功" << std::endl;
    } else {
        std::cout << "✗ 坐标转换数据导出失败" << std::endl;
    }
    
    if (gnss::DataExporter::exportEllipsoidParams()) {
        std::cout << "✓ 椭球参数导出成功" << std::endl;
    } else {
        std::cout << "✗ 椭球参数导出失败" << std::endl;
    }
    
    if (gnss::DataExporter::exportCoordConversionErrors(xyz_normal)) {
        std::cout << "✓ 坐标转换误差数据导出成功" << std::endl;
    } else {
        std::cout << "✗ 坐标转换误差数据导出失败" << std::endl;
    }
    
    if (gnss::DataExporter::exportFrameDifferenceMatrix(xyz_normal)) {
        std::cout << "✓ 框架差异矩阵导出成功" << std::endl;
    } else {
        std::cout << "✗ 框架差异矩阵导出失败" << std::endl;
    }
    
    // 导出ENU坐标转换数据
    XYZ refXYZ_export(4081945.67, 2187689.34, 4767321.89);  // 接收机位置
    XYZ targetXYZ(4100000.00, 2200000.00, 4780000.00);  // 卫星位置或目标点
    
    if (gnss::DataExporter::exportENUConversionData(targetXYZ, refXYZ_export, frames)) {
        std::cout << "✓ ENU坐标转换数据导出成功" << std::endl;
    } else {
        std::cout << "✗ ENU坐标转换数据导出失败" << std::endl;
    }
    
    // 清理动态分配的内存
    for (auto frame : frames) {
        delete frame;
    }

    return 0;
}
