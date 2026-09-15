/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *
 * Author: shoujian zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * Description: 
 *  配置文件读取演示程序
 *  展示如何使用 ConfigReader 读取不同类型的配置值
 */

#include <gnsslab/ConfigReader.h>
#include <iostream>
#include <stdexcept>
#include <string>

// 调试模式开关
#define DEBUG_MODE 1

// 配置结构体
struct GNSSConfig {
    // 系统开关
    int GPS = 0;
    int BD2 = 0;
    int GLO = 0;
    
    // 文件路径
    std::string navFile = "";
    std::string obsFile = "";
    
    // 噪声参数
    double noiseGPSCode = 0.0;
    double noiseGPSPhase = 0.0;
    double noiseBDSCode = 0.0;
    double noiseBDSPhase = 0.0;
    double noiseGLO = 0.0;
    
    // 定位参数
    int maxIter = 0;
    double threshold = 0.0;
    
    // 验证配置有效性
    void validate() const {
        if (navFile.empty()) {
            throw std::invalid_argument("导航文件路径(navFile)不能为空");
        }
        if (obsFile.empty()) {
            throw std::invalid_argument("观测文件路径(obsFile)不能为空");
        }
        if (maxIter <= 0) {
            throw std::invalid_argument("最大迭代次数(maxIter)必须大于0");
        }
        if (threshold <= 0) {
            throw std::invalid_argument("收敛阈值(threshold)必须大于0");
        }
        if (GPS + BD2 + GLO == 0) {
            throw std::invalid_argument("至少需要启用一个卫星系统(GPS/BD2/GLO)");
        }
    }
    
    // 打印配置信息
    void print() const {
        std::cout << "\n" << "=" << std::string(50, '=') << "=" << std::endl;
        std::cout << "              GNSS 配置参数" << std::endl;
        std::cout << "=" << std::string(50, '=') << "=" << std::endl;
        
        std::cout << "\n【卫星系统配置】" << std::endl;
        std::cout << "  GPS 启用: " << (GPS ? "是" : "否") << std::endl;
        std::cout << "  BD2 启用: " << (BD2 ? "是" : "否") << std::endl;
        std::cout << "  GLO 启用: " << (GLO ? "是" : "否") << std::endl;
        
        std::cout << "\n【文件路径】" << std::endl;
        std::cout << "  导航文件: " << navFile << std::endl;
        std::cout << "  观测文件: " << obsFile << std::endl;
        
        std::cout << "\n【噪声参数 (m)】" << std::endl;
        std::cout << "  GPS 伪距噪声: " << noiseGPSCode << std::endl;
        std::cout << "  GPS 载波噪声: " << noiseGPSPhase << std::endl;
        std::cout << "  BDS 伪距噪声: " << noiseBDSCode << std::endl;
        std::cout << "  BDS 载波噪声: " << noiseBDSPhase << std::endl;
        std::cout << "  GLO 噪声: " << noiseGLO << std::endl;
        
        std::cout << "\n【定位参数】" << std::endl;
        std::cout << "  最大迭代次数: " << maxIter << std::endl;
        std::cout << "  收敛阈值: " << threshold << " m" << std::endl;
        
        std::cout << "=" << std::string(50, '=') << "=" << std::endl;
    }
};

// 从配置文件加载配置
GNSSConfig loadConfig(const std::string& configFile) {
    #if DEBUG_MODE
    std::cout << "[DEBUG] 正在读取配置文件: " << configFile << std::endl;
    #endif
    
    ConfigReader configReader(configFile);
    GNSSConfig config;
    
    try {
        // 读取整数类型
        config.GPS = configReader.getValueAsInt("GPS");
        config.BD2 = configReader.getValueAsInt("BD2");
        config.GLO = configReader.getValueAsInt("GLO");
        config.maxIter = configReader.getValueAsInt("maxIter");
        
        // 读取字符串类型
        config.navFile = configReader.getValueAsString("navFile");
        config.obsFile = configReader.getValueAsString("obsFile");
        
        // 读取双精度浮点类型
        config.noiseGPSCode = configReader.getValueAsDouble("noiseGPSCode");
        config.noiseGPSPhase = configReader.getValueAsDouble("noiseGPSPhase");
        config.noiseBDSCode = configReader.getValueAsDouble("noiseBDSCode");
        config.noiseBDSPhase = configReader.getValueAsDouble("noiseBDSPhase");
        config.noiseGLO = configReader.getValueAsDouble("noiseGLO");
        config.threshold = configReader.getValueAsDouble("threshold");
        
    } catch (const std::exception& e) {
        throw std::runtime_error("配置文件读取失败: " + std::string(e.what()));
    }
    
    return config;
}

// 打印帮助信息
void printHelp(const std::string& programName) {
    std::cout << "==========================================" << std::endl;
    std::cout << "        配置文件读取演示程序" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "使用方式:" << std::endl;
    std::cout << "  " << programName << " <配置文件路径>" << std::endl;
    std::cout << "  " << programName << " -h | --help" << std::endl;
    std::cout << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << programName << " spp.ini" << std::endl;
    std::cout << "  " << programName << " ../examples/spp.ini" << std::endl;
    std::cout << std::endl;
    std::cout << "配置文件格式:" << std::endl;
    std::cout << "  # 卫星系统配置 (0=禁用, 1=启用)" << std::endl;
    std::cout << "  GPS=1" << std::endl;
    std::cout << "  BD2=1" << std::endl;
    std::cout << "  GLO=0" << std::endl;
    std::cout << std::endl;
    std::cout << "  # 文件路径" << std::endl;
    std::cout << "  navFile=./data/nav.24n" << std::endl;
    std::cout << "  obsFile=./data/obs.24o" << std::endl;
    std::cout << std::endl;
    std::cout << "  # 噪声参数 (单位: 米)" << std::endl;
    std::cout << "  noiseGPSCode=0.3" << std::endl;
    std::cout << "  noiseGPSPhase=0.001" << std::endl;
    std::cout << "  noiseBDSCode=0.35" << std::endl;
    std::cout << "  noiseBDSPhase=0.0015" << std::endl;
    std::cout << "  noiseGLO=0.4" << std::endl;
    std::cout << std::endl;
    std::cout << "  # 定位参数" << std::endl;
    std::cout << "  maxIter=20" << std::endl;
    std::cout << "  threshold=0.001" << std::endl;
    std::cout << "==========================================" << std::endl;
}

int main(int argc, char *argv[]) {
    const std::string programName = argv[0];
    
    try {
        // 参数检查
        if (argc != 2) {
            std::cerr << "错误: 参数数量不正确！" << std::endl;
            printHelp(programName);
            return 1;
        }
        
        // 帮助信息
        std::string arg = argv[1];
        if (arg == "-h" || arg == "--help") {
            printHelp(programName);
            return 0;
        }
        
        // 加载配置文件
        GNSSConfig config = loadConfig(arg);
        
        // 验证配置
        config.validate();
        
        // 打印配置信息
        config.print();
        
        // 调试输出
        #if DEBUG_MODE
        std::cout << "\n[DEBUG] 配置读取成功！" << std::endl;
        #endif
        
    } catch (const std::exception &e) {
        std::cerr << "\n错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}