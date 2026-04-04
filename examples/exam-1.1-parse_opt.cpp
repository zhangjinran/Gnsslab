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
#include <cstdlib> // 用于 std::exit
#include <stdexcept> // 用于 std::invalid_argument
#include <cstring> // Required for strcmp
#include <ConfigReader.h>//用于配置文件读取

using namespace std;
// 函数声明
double add(double a, double b);

double subtract(double a, double b);

double multiply(double a, double b);

double divide(double a, double b);

//提前声明避免报错，全局变量
double num1;
double num2;
std::string operation;
double result;


int main(int argc, char *argv[]) {

    string helpStr//帮助字符创，就是用来存储说明信息的
    = "Usage: "
      "  calculator <num1> <operation> <num2>\n"
      "  Operations only support: +, -, *, /\n"
      "warning:\n"
      "  whitespace must be given between num1 operation and num2!\n"
      "examples:\n"
      "  calculator 2 * 4 \n"
      "  calculator 2 / 4 \n"
      "author:"
      "     shjzhang, shjzhang@sgg.whu.edu.cn";




    //如果argc为2就有两种选择请求帮助，或者采用配置文件形式。
    if (argc == 2)
    {
        cout << argv[0] << endl;
        cout << argv[1] << endl;
        //help格式
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {//strcmp比较两者是否相等，相等就为0.
            cout << helpStr << endl;
            return 1;
        }
        //配置文件读取内容。
        else {
            ConfigReader configReader(argv[1]); // Replace with your config file path
            try {
                num1 = configReader.getValueAsInt("num1");
                operation = configReader.getValueAsString("operation");
                num2 = configReader.getValueAsInt("num2");

                std::cout << "num1: " << num1 << std::endl;
                std::cout << "operation: " << operation << std::endl;
                std::cout << "num2: " << num2 << std::endl;


            }
            catch (std::runtime_error &e) {
                cerr << e.what() << endl;
                exit(-1);
            }

        }

    }

    // 检查参数数量，不符合要求直接报错
    else if (argc == 3||argc>4) {
        std::cerr << "Usage: calculator <num1> <operation> <num2>\n";
        std::cerr << "Operations: +, -, *, /\n";
        std::cerr << "Usage2:calculator <config filename>\n";
        return 1;
    }

    // 解析输入参数
    else if (argc==4) {
         num1 = std::atof(argv[1]);//把命令行格式转化为double格式。
         operation = argv[2];
         num2 = std::atof(argv[3]);

    }

    try {
        if (operation == "+") {
            result = add(num1, num2);
        } else if (operation == "-") {
            result = subtract(num1, num2);
        } else if (operation == "*") {
            result = multiply(num1, num2);
        } else if (operation == "/") {
            if (num2 == 0) {
                throw std::invalid_argument("Division by zero is not allowed.");
            }
            result = divide(num1, num2);
        } else {
            std::cerr << "Invalid operation: " << operation << "\n";
            return 1;
        }

        std::cout << "Result: " << result << "\n";
    } catch (const std::invalid_argument &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

// 函数定义
double add(double a, double b) {
    return a + b;
}

double subtract(double a, double b) {
    return a - b;
}

double multiply(double a, double b) {
    return a * b;
}

double divide(double a, double b) {
    return a / b;
}