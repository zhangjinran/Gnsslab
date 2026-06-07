//
// Created by zhang on 2026/5/8.
//

#include "zuiyouguji/gnssfunc.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include<Eigen/Dense>
#include<boost/math/distributions/chi_squared.hpp>
#include"SolverLSQ.h"
#include <algorithm>


#include "GnssStruct.h"

int main()
{
    std::string filename="/home/zhang/Documents/大学课程/大二第二学期课程/卫星算法/gnssLab-2.4/examples/zuiyouguji/CUSV_2026001_1.txt";
    auto epochs = readData(filename);
    map<double,SolverLSQ> solver_lsqs;
    vector<string> sats;
    for (auto epoch:epochs) {
        for (auto sat:epoch.sats) {
            if (find(sats.begin(),sats.end(),sat.sat)==sats.end())
                sats.push_back(sat.sat);
        }
    }

    for (auto& st : epochs) {
        cout<<"sats NUm:"<<st.sats.size()<<endl;
        SolverLSQ solver_lsq=sovleLsQ(st);
        cout<<"epoch"<<st.gpsSec<<endl;
        cout<<"xyz:"<<solver_lsq.getxyz()<<endl;
        double df=solver_lsq.getResiduals().size()-4;
        inspect(solver_lsq,df,st,0.01);
        solver_lsqs[st.gpsSec]=solver_lsq;
    }
    writeData(epochs,solver_lsqs);




    for (auto sat:sats) {
        map<double,SolverLSQ> solver_lsqs;
        for (auto& st : epochs) {
            vector<string> sats_temp;
            for (auto sat:st.sats) {

                sats_temp.push_back(sat.sat);
            }
            cout<<"sats NUm:"<<st.sats.size()<<endl;
            SolverLSQ solver_lsq=sovleLsQ(st,sat);
            cout<<"epoch"<<st.gpsSec<<endl;
            cout<<"xyz:"<<solver_lsq.getxyz()<<endl;
            if (find(sats_temp.begin(),sats_temp.end(),sat)!=sats_temp.end()) {
                inspect_bias(solver_lsq,st);
            }
            solver_lsqs[st.gpsSec]=solver_lsq;
        }
        writeData(epochs,solver_lsqs,sat);

    }

    for (auto &st: epochs) {
        SolverLSQ solver_lsq ;


        while (true) {
            vector<string> sats_delete;
            vector<string> sats_temp;
            double df;
            for (auto sat: st.sats) {
                sats_temp.push_back(sat.sat);
            }

            solver_lsq=sovleLsQ(st);
            df= solver_lsq.getResiduals().size() - 4;
            if (!inspect(solver_lsq, df, st, 0.01)) {
                for (auto sat: sats) {


                        if (find(sats_temp.begin(), sats_temp.end(), sat) != sats_temp.end()) {
                            if (sat=="C60")
                                cout<<endl;
                            SolverLSQ solver_temp = sovleLsQ(st, sat);
                            if (!inspect_bias(solver_temp, st)) {
                                sats_delete.push_back(sat);
                            }
                        }

                }

                if (sats_delete.empty())
                    break;
                for (auto &sat: sats_delete) {
                    st.sats.erase(remove_if(st.sats.begin(), st.sats.end(),[&](auto &s) { return s.sat == sat; }    ),    st.sats.end());
                }
            }
            else
                break;
        }
        solver_lsqs[st.gpsSec]=solver_lsq;

    }
    writeData(epochs, solver_lsqs, "final");


    return 0;
}