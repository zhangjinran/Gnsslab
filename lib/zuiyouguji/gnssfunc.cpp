//
// Created by zhang on 2026/5/8.
//
#include "gnssfunc.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include<Eigen/Dense>
#include<boost/math/distributions/chi_squared.hpp>
#include"SolverLSQ.h"


#include "GnssStruct.h"
#define debug 0

std::vector<EpochData> readData(const std::string& filename)
{
    std::ifstream fin(filename);

    if (!fin.is_open())
    {
        throw std::runtime_error("cannot open file");
    }

    std::vector<EpochData> epochs;

    std::string line;

    while (std::getline(fin, line))
    {
        if (line.empty())
        {
            continue;
        }

        std::stringstream ss(line);

        EpochData epoch;

        ss >> epoch.gpsWeek
           >> epoch.gpsSec
           >> epoch.satNum;

        epoch.sats.reserve(epoch.satNum);

        for (int i = 0; i < epoch.satNum; ++i)
        {
            std::getline(fin, line);

            std::stringstream satss(line);

            SatData sat;

            satss >> sat.sat
                  >> sat.pseudorange
                  >> sat.x
                  >> sat.y
                  >> sat.z
                  >> sat.elevation;
            if (sat.sat=="C32")
                continue;
            epoch.sats.push_back(sat);
        }

        epochs.push_back(epoch);
    }

    return epochs;
}

SolverLSQ sovleLsQ(EpochData& epoch_data,SatID sat) {
    Vector3d xyz(0, 0, 0);
    Vector3d dxyz(0, 0, 0);
    SolverLSQ sovleLsq_final;
    int iter(0);
    while (true) {

        EquSys equSys;
        equSys = linearize(xyz, epoch_data,sat);

        if(debug)
            cout << "afte linearize:" << endl;

        // 如果是基准站，完成线性化后就退出
        // 因为基准站位置是准确的
        SolverLSQ solverLsq;

        solverLsq.solve(equSys);



        dxyz = solverLsq.getdxyz();

        xyz += dxyz;
        solverLsq.setxyz(xyz);
        if (debug) {
            cout << "iteration:" << iter<<" "
            << "dxyz:" << dxyz.transpose()<<" "
            << "xyz:" << xyz.transpose() << endl;
        }

        // convergence threshold

        if (dxyz.norm() < 0.1) {
            break;
        }

        if (iter > 10) {
            InvalidSolver e("too many iterations");
            throw(e);
        }
        iter++;
        sovleLsq_final= solverLsq;

}

return sovleLsq_final;
}



EquSys linearize(Vector3d xyz,EpochData& epoch_data,SatID test_sat) {
     EquSys equSysTemp;
    equSysTemp.station="moren";
    // 自动收集当前观测中的所有系统
    std::set<std::string> availableSystems;
    double cutOffElev = 5;

    for (auto& st:epoch_data.sats) {
        SatID sat = st.sat;
        availableSystems.insert(sat.system);
    }


    VariableSet varSetTemp;
    for (auto st:epoch_data.sats) {
        SatID sat = st.sat;

        double elev = st.elevation;
        double elevRad = elev*DEG_TO_RAD;

        // 跳过这颗卫星，不形成观测方程和未知参数数据
        if(elev < cutOffElev)
        {
            continue;
        }

        // 这里卫星的位置，应该是地球自转以后的卫星位置
        Vector3d satxyz(st.x, st.y, st.z);
        XYZ satXYZ;

        satXYZ=satxyz;

        // rho
        double rho(0.0);
        rho = ( satXYZ - xyz).norm();
        if(debug)
        {
            cout << "Sat:" << sat << endl;
            cout << "xyz:" << xyz << endl;
            cout << "satXYZ:" << satXYZ << endl;
        }


        // partials
        Eigen::Vector3d cosines;
        cosines[0] = (xyz.x() - satXYZ[0]) / rho;
        cosines[1] = (xyz.y() - satXYZ[1]) / rho;
        cosines[2] = (xyz.z() - satXYZ[2]) / rho;

        // todo
        // 请补充rhoDot，用于后续的单点测速

        // 首先定义所有可能的未知参数
        // 位置参数 3 个（永远共用）
        Variable dx(equSysTemp.station, Parameter::dX);
        Variable dy(equSysTemp.station, Parameter::dY);
        Variable dz(equSysTemp.station, Parameter::dZ);
        Variable bias(equSysTemp.station, Parameter::bias);
        bool istestSat=(sat==test_sat);
        // 钟差参数：根据系统自动生成
        std::map<std::string, Variable> clockParams;

        for (auto& sys : availableSystems) {
            if (sys == "G") {
                clockParams[sys] = Variable(equSysTemp.station, Parameter::cdt);
            } else if (sys == "C") {
                clockParams[sys] = Variable(equSysTemp.station, Parameter::cdt_BDS);
            }
            // 可无限扩展
        }

        // 对每个观测值，都需要存储对应的未知参数及其偏导数

                EquID equID = EquID(sat, "moren");


                //>> 先验残差
                double prefit;
                double computedObs = rho ;
                prefit = st.pseudorange - computedObs;

                if (debug) {
                    cout<<"prefit:"<<prefit<<endl;
                    cout << "obs:" << st.pseudorange << endl;
                    cout << "rho:" << rho << endl;
                    cout<<endl;
                }

                equSysTemp.obsEquData[equID].prefit = prefit;
                equSysTemp.obsEquData[equID].varCoeffData[dx] = cosines[0];
                equSysTemp.obsEquData[equID].varCoeffData[dy] = cosines[1];
                equSysTemp.obsEquData[equID].varCoeffData[dz] = cosines[2];
                std::string sys = sat.system;
                // 自动给对应系统的钟差赋系数 1.0
                equSysTemp.obsEquData[equID].varCoeffData[ clockParams[sys] ] = 1.0;

                if (istestSat) {
                    equSysTemp.obsEquData[equID].varCoeffData[bias]= 1.0;
                }



                double sigma1_2=pow(2.4,2);
                double temp=0.12*1.001/sqrt(0.002001+pow(sin(elev*PI/180),2));
                double sigma2_2=pow(temp,2);
                double temp1=0.13+0.53*exp(-elev/10);
                double temp2=0.15+0.43*exp(-elev/6.9);
                double sigma3_2=pow(3.09*sqrt(pow(temp1,2)+pow(temp2,2)),2);
                // Compute the weight according to elevation
                double sigCode=sqrt(sigma1_2+sigma2_2+sigma3_2);
                double weight;

                weight = 1.0 / (sigCode*sigCode);

                equSysTemp.obsEquData[equID].weight = weight; // IF组合方差为1.0m

                // 把当前观测方程未知参数插入到总体的未知参数
                varSetTemp.insert(dx);
                varSetTemp.insert(dy);
                varSetTemp.insert(dz);
                varSetTemp.insert(clockParams[sys]);
        if (istestSat)
            varSetTemp.insert(bias);
                equSysTemp.satList.push_back(sat);
            }
    equSysTemp.varSet = varSetTemp;
    if (debug)
        cout<<equSysTemp.satList.size()<<endl;

    return equSysTemp;
};
bool inspect(SolverLSQ solver_lsq,double df,EpochData& epoch_data,double alpha) {
    VectorXd residual=solver_lsq.getResiduals();
    MatrixXd covariance=solver_lsq.getcov_r();
    MatrixXd covariance2=solver_lsq.getw();
    double T0=residual.transpose()*covariance2*residual;
    epoch_data.T0=T0;
    epoch_data.result=solver_lsq.getxyz();
    if(debug)
        cout<<"T0:"<<T0<<endl;
    return detect_model(T0, df,epoch_data, alpha);

}


bool inspect_bias(SolverLSQ solver_lsq,EpochData& epoch_data,double alpha) {

    VectorXd residual=solver_lsq.getResiduals();
    MatrixXd covariance=solver_lsq.getcov_r();
    MatrixXd covariance2=solver_lsq.getw();
    double Ta=epoch_data.T0-residual.transpose()*covariance2*residual;
    epoch_data.Ta=Ta;
    epoch_data.result=solver_lsq.getxyz();
    alpha=0.01/epoch_data.satNum;

    return detect_model(Ta, 1,epoch_data, alpha);
}

// 计算卡方分布上侧 alpha 分位数（即讲义里的 χ²_α(df, 0)）
double chi2_critical_value(int df, double alpha)
{
    // df: 自由度（ℓ - n）
    // alpha: 显著性水平（如 0.05，对应 95% 置信度）
    boost::math::chi_squared dist(df);
    // 上侧 alpha 分位数 = 下侧 (1-alpha) 分位数
    double critical = boost::math::quantile(complement(dist, alpha));
    return critical;
}

// 模型探测的判断逻辑（对应讲义 7.4.1）
bool detect_model(double T0, int df, EpochData& epoch_data,double alpha)
{
    double chi2_crit = chi2_critical_value(df, alpha);
    epoch_data.thrd=chi2_crit;
    if (debug) {
        std::cout << "T0: " << T0 << ", 临界值 χ²(" << df << ", " << alpha << "): " << chi2_crit << std::endl;
    }
    if (T0 <= chi2_crit)
    {
        if (debug) {
            std::cout << "接受原假设 H0，模型通过检验" << std::endl;
        }
        return true; // 接受 Ω0
    }
    else
    {
        if (debug) {
            std::cout << "拒绝原假设 H0，模型存在异常" << std::endl;
        }
        return false; // 拒绝 Ω0，需识别/适应
    }
}

void writeData(const vector<EpochData>epoch_datas, map<double,SolverLSQ>& solver_lsqs,string name) {
    ofstream outFile("/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/examples/zuiyouguji/result/data"+name+".txt");
    // 固定浮点格式，保留2位小数
    outFile << fixed << setprecision(10);


    // 第一行：表头 空格分隔
    outFile << "time x y z T0 thrd " ;
    vector<string> sats;
    for (auto epoch:epoch_datas) {
        for (auto sat:epoch.sats) {
            if (find(sats.begin(),sats.end(),sat.sat)==sats.end()) {
                sats.push_back(sat.sat);
                outFile<<sat.sat<<" ";
            }
        }
    }
    outFile <<endl;

    // 多行数据，每个字段空格隔开，数字固定格式
    for (auto epoch: epoch_datas) {
        SolverLSQ solver_lsq=solver_lsqs[epoch.gpsSec];
        VectorXd risudals=solver_lsq.getResiduals();
        outFile << epoch.gpsSec << " " << epoch.result[0] << " " << epoch.result[1] << " " << epoch.result[2] << " " ;
        if (name=="")
            outFile<< epoch.T0;
        else
            outFile<< epoch.Ta;
        outFile<<" "<<epoch.thrd<<" ";


        vector<string> sat_temp;
        for (auto satdata:epoch.sats) {
            sat_temp.push_back(satdata.sat);
        }
        vector<double> sat_list(sats.size(),0);
        for (int i=0;i<sats.size();i++) {
            for (int j=0; j<sat_temp.size();j++) {
                if (sats[i] == sat_temp[j]) {
                    sat_list[i]=risudals[j];
                    break;
                }
            }
        }
        for (int i=0;i<sat_list.size();i++) {
            outFile << sat_list[i] << " ";
        }
        outFile << endl;

    }

    outFile.close();

}