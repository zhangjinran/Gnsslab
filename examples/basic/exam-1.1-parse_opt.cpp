/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *
 * Author: shoujian zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 */

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <string>
#include <algorithm>
#include <gnsslab/ConfigReader.h>

// 配置结构体（替代全局变量，更好的封装）
struct CalculatorConfig {
    double num1{0.0};
    double num2{0.0};
    std::string operation;
    
    // 验证配置有效性
    void validate() const {
        if (!std::isfinite(num1)) {
            throw std::invalid_argument("num1 不是有效数字");
        }
        if (!std::isfinite(num2)) {
            throw std::invalid_argument("num2 不是有效数字");
        }
        if (operation != "+" && operation != "-" && operation != "*" && operation != "/") {
            throw std::invalid_argument("不支持的操作符: " + operation);
        }
        if (operation == "/" && num2 == 0) {
            throw std::invalid_argument("除数不能为零");
        }
    }
};

// 函数声明
double calculate(const CalculatorConfig& config);
CalculatorConfig loadConfig(const std::string& configFile);
CalculatorConfig parseCommandLine(int argc, char* argv[]);
void printHelp(const std::string& programName);

// 调试模式开关
#define DEBUG_MODE 1

int main(int argc, char* argv[]) {
    const std::string programName = argv[0];
    
    try {
        CalculatorConfig config;
        
        // 参数处理逻辑
        if (argc == 2) {
            std::string arg = argv[1];
            if (arg == "-h" || arg == "--help") {
                printHelp(programName);
                return 0;
            }
            // 尝试从配置文件读取
            config = loadConfig(arg);
        } else if (argc == 4) {
            // 从命令行参数读取
            config = parseCommandLine(argc, argv);
        } else {
            std::cerr << "错误: 参数数量不正确！" << std::endl;
            printHelp(programName);
            return 1;
        }
        
        // 调试输出：显示配置信息
        #if DEBUG_MODE
        std::cout << "[DEBUG] 配置信息:" << std::endl;
        std::cout << "[DEBUG] num1 = " << config.num1 << std::endl;
        std::cout << "[DEBUG] operation = " << config.operation << std::endl;
        std::cout << "[DEBUG] num2 = " << config.num2 << std::endl;
        #endif
        
        // 验证配置
        config.validate();
        
        // 执行计算
        double result = calculate(config);
        
        // 输出结果
        std::cout << "计算结果: " << result << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

// 从配置文件加载配置
CalculatorConfig loadConfig(const std::string& configFile) {
    #if DEBUG_MODE
    std::cout << "[DEBUG] 正在读取配置文件: " << configFile << std::endl;
    #endif
    
    ConfigReader configReader(configFile);
    CalculatorConfig config;
    
    try {
        // 使用正确的类型转换
        config.num1 = configReader.getValueAsDouble("num1");
        config.num2 = configReader.getValueAsDouble("num2");
        config.operation = configReader.getValueAsString("operation");
        
        // 检查配置项是否为空
        if (config.operation.empty()) {
            throw std::runtime_error("配置项 operation 不能为空");
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("配置文件读取失败: " + std::string(e.what()));
    }
    
    return config;
}

// 从命令行参数解析配置
CalculatorConfig parseCommandLine(int argc, char* argv[]) {
    #if DEBUG_MODE
    std::cout << "[DEBUG] 正在解析命令行参数" << std::endl;
    #endif
    
    CalculatorConfig config;
    
    // 使用 atof 进行类型转换
    config.num1 = std::atof(argv[1]);
    config.operation = argv[2];
    config.num2 = std::atof(argv[3]);
    
    // 检查转换是否成功
    if (config.num1 == 0 && argv[1] != std::string("0")) {
        throw std::invalid_argument("num1 不是有效数字: " + std::string(argv[1]));
    }
    if (config.num2 == 0 && argv[3] != std::string("0")) {
        throw std::invalid_argument("num2 不是有效数字: " + std::string(argv[3]));
    }
    
    return config;
}

// 执行计算
double calculate(const CalculatorConfig& config) {
    #if DEBUG_MODE
    std::cout << "[DEBUG] 正在执行计算: " << config.num1 << " " 
              << config.operation << " " << config.num2 << std::endl;
    #endif
    
    if (config.operation == "+") {
        return config.num1 + config.num2;
    } else if (config.operation == "-") {
        return config.num1 - config.num2;
    } else if (config.operation == "*") {
        return config.num1 * config.num2;
    } else if (config.operation == "/") {
        return config.num1 / config.num2;
    }
    
    throw std::invalid_argument("未知操作符: " + config.operation);
}

// 打印帮助信息
void printHelp(const std::string& programName) {
    std::cout << "==========================================" << std::endl;
    std::cout << "          计算器程序使用说明" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "使用方式:" << std::endl;
    std::cout << "  1. 命令行直接输入:" << std::endl;
    std::cout << "     " << programName << " <num1> <operation> <num2>" << std::endl;
    std::cout << std::endl;
    std::cout << "  2. 使用配置文件:" << std::endl;
    std::cout << "     " << programName << " <配置文件路径>" << std::endl;
    std::cout << std::endl;
    std::cout << "  3. 显示帮助:" << std::endl;
    std::cout << "     " << programName << " -h 或 --help" << std::endl;
    std::cout << std::endl;
    std::cout << "支持的操作符:" << std::endl;
    std::cout << "  +   加法" << std::endl;
    std::cout << "  -   减法" << std::endl;
    std::cout << "  *   乘法" << std::endl;
    std::cout << "  /   除法" << std::endl;
    std::cout << std::endl;
    std::cout << "配置文件格式:" << std::endl;
    std::cout << "  num1=10.5" << std::endl;
    std::cout << "  operation=+" << std::endl;
    std::cout << "  num2=3.2" << std::endl;
    std::cout << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << programName << " 10 + 5" << std::endl;
    std::cout << "  " << programName << " 20 / 4" << std::endl;
    std::cout << "  " << programName << " config.ini" << std::endl;
    std::cout << "==========================================" << std::endl;
}