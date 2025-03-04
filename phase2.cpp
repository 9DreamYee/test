#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <cassert>
#include "gurobi_c++.h"
#include "ParseNetInfoToGurobi.h"

void exportToCSV(std::vector<commonBoundary> &boundaries,
                 const std::vector<double> &actualAreaVector,
                 const std::vector<double> &ratioVector)
{
    std::ofstream csvFile("net_area_data.csv");
    csvFile << "BoundayID,ShiftAmount,ActualArea,Ratio\n";
    for(size_t i = 0; i < boundaries.size(); i++){
        csvFile << boundaries[i].boundaryID << ","
                << boundaries[i].shiftAmount << ","
                << actualAreaVector[i] << ","
                << (ratioVector[i] >= 0 ? ratioVector[i] : 0)
                << "\n";
    }
    csvFile.close();
}

int main(int argc, char* argv[])
{
    // 1. 讀取 commonBoundary
    std::ifstream file(argv[1]);
    auto nets       = parseNetsInfo(file);
    auto boundaries = buildCommonBoundaries(nets);
    Rectangle innerRect;
    Rectangle outterRect =Rectangle(-1.4e+08,-1.4e+08,2.8e+08,2.8e+08);
    sortBoundaryClockwise(boundaries,innerRect);
    //同時特例處利Boundary 0 
    UpdateCommonBoundaryInfo(boundaries,nets,innerRect,outterRect);

    std::vector<double> actualAreaVector;
    std::vector<double> ratioVector;

    try {
        // 初始化 Gurobi
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "phase2_cornerMILP.log");
        env.start();
        GRBModel model = GRBModel(env);

        const int E = boundaries.size();

        // =============== 定義變數 ===============
        // normal net: delta1[e], u_e
        // corner net: bVar1,e, bVar2,e, delta1,e, delta2,e + Lin1,e, Lin2,e, u_e
        std::vector<GRBVar> bVar1(E), bVar2(E);
        std::vector<GRBVar> delta1(E), delta2(E);
        std::vector<GRBVar> Lin1(E), Lin2(E); // for linearization bVar * delta
        std::vector<GRBVar> uVars(E);

        for(int e=0; e<E; e++){
            auto &b = boundaries[e];

            // 誤差變數
            std::string uName = "u_"+std::to_string(b.boundaryID);
            uVars[e] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, uName);

            if(b.shift_Direction == 2) {
                // corner net
                // binary var
                std::string b1Name = "b1_"+std::to_string(b.boundaryID);
                std::string b2Name = "b2_"+std::to_string(b.boundaryID);
                bVar1[e] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, b1Name);
                bVar2[e] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, b2Name);

                // define delta1, delta2
                // 這裡以 b.shiftMax 為上下界, 也可使用 corner_area / alpha 之類
                double M1 = b.shiftMin;
                double M2 = b.shiftMax;
                // alpha=0 => 原邊界線不可用 => M1=0
                if(b.alpha < 1e-9) {
                    M1 = 0;
                }

                std::string dName1 = "delta1_"+std::to_string(b.boundaryID);
                std::string dName2 = "delta2_"+std::to_string(b.boundaryID);
                delta1[e] = model.addVar(0.0, M1, 0.0, GRB_CONTINUOUS, dName1);
                delta2[e] = model.addVar(0.0, M2, 0.0, GRB_CONTINUOUS, dName2);

                // 定義 Lin1, Lin2 => bVar*delta => 純線性表示
                std::string lx1Name = "Lin1_"+std::to_string(b.boundaryID);
                std::string lx2Name = "Lin2_"+std::to_string(b.boundaryID);
                Lin1[e] = model.addVar(0.0, M1, 0.0, GRB_CONTINUOUS, lx1Name);
                Lin2[e] = model.addVar(0.0, M2, 0.0, GRB_CONTINUOUS, lx2Name);

            } else {
                // normal net
                bVar1[e] = GRBVar(); // dummy
                bVar2[e] = GRBVar(); // dummy

                // define single delta
                std::string dName = "delta_"+std::to_string(b.boundaryID);
                delta1[e] = model.addVar(b.shiftMin, b.shiftMax, 0.0, GRB_CONTINUOUS, dName);
                delta2[e] = GRBVar(); // unused

                Lin1[e] = GRBVar(); // unused
                Lin2[e] = GRBVar(); // unused
            }
        }
        model.update();

        // =============== 加入約束 ===============
        for(int e=0; e<E; e++){
            auto &b = boundaries[e];
            if(b.shift_Direction == 2) {
                // corner net => 互斥: bVar1 + bVar2 = 1
                model.addConstr(bVar1[e] + bVar2[e] <= 1,
                                "choose_"+std::to_string(b.boundaryID));

                // Big-M: 
                double M1 = delta1[e].get(GRB_DoubleAttr_UB);
                double M2 = delta2[e].get(GRB_DoubleAttr_UB);

                // delta1 <= bVar1*M1
                model.addConstr(delta1[e] <= bVar1[e]*M1,
                                "useLine1_"+std::to_string(b.boundaryID));
                // delta2 <= bVar2*M2
                model.addConstr(delta2[e] <= bVar2[e]*M2,
                                "useLine2_"+std::to_string(b.boundaryID));

                // corner_area: alpha*delta1 <= corner_area (if alpha>0)
                if(b.alpha > 1e-9) {
                    GRBLinExpr lhs = b.alpha * delta1[e];
                    model.addConstr(lhs <= bVar1[e]* b.cornerArea,
                                    "cornerLimit_"+std::to_string(b.boundaryID));
                }

                // =========== Linearization for bVar*delta => Lin1,Lin2 ===========

                // Lin1[e] = bVar1[e]* delta1[e]
                // bigM constraints:
                model.addConstr(Lin1[e] <= delta1[e],  "Lin1a_"+std::to_string(e));
                model.addConstr(Lin1[e] <= bVar1[e]*M1,"Lin1b_"+std::to_string(e));
                model.addConstr(Lin1[e] >= delta1[e] - M1*(1-bVar1[e]),
                                "Lin1c_"+std::to_string(e));
                model.addConstr(Lin1[e] >= 0,          "Lin1d_"+std::to_string(e));

                // Lin2[e] = bVar2[e]* delta2[e]
                model.addConstr(Lin2[e] <= delta2[e],  "Lin2a_"+std::to_string(e));
                model.addConstr(Lin2[e] <= bVar2[e]*M2,"Lin2b_"+std::to_string(e));
                model.addConstr(Lin2[e] >= delta2[e] - M2*(1-bVar2[e]),
                                "Lin2c_"+std::to_string(e));
                model.addConstr(Lin2[e] >= 0,          "Lin2d_"+std::to_string(e));
            }
        }

        // actualArea & 誤差
        std::vector<GRBLinExpr> actualAreaExpr(E);
        for(int e=0; e<E; e++){
            auto &b = boundaries[e];
            if(b.shift_Direction == 2) {
                // actualArea = bVar1*(alpha*delta1) + bVar2*(corner_area + alpha_corner*delta2)
                // => 改為 = alpha*Lin1[e] + bVar2[e]* corner_area + alpha_corner*Lin2[e]
                GRBLinExpr expr = 0.0;

                expr += b.alpha * Lin1[e];   // bVar1[e]* alpha*delta1
                expr += b.alpha_corner * Lin2[e];  // bVar2[e]* alpha_corner*delta2
                // corner_area*bVar2 => (const * binary) 是線性的 => OK
                expr += b.cornerArea * bVar2[e];

                actualAreaExpr[e] = expr;
            } else {
                // normal net => actualArea = alpha * delta1
                GRBLinExpr expr = 0.0;
                expr += b.alpha * delta1[e];
                actualAreaExpr[e] = expr;
            }
        }

        // 誤差約束
        std::vector<GRBVar> residual(E); // optional
        for(int e=0; e<E; e++){
            auto &b = boundaries[e];
            auto &Aexpr = actualAreaExpr[e];

            // u_e >= Aexpr - shiftAmount
            GRBLinExpr lhs1 = Aexpr;
            lhs1 -= b.shiftAmount;
            std::string cPos = "absPos_"+std::to_string(e);
            model.addConstr(lhs1 <=  uVars[e], cPos);

            // u_e >= shiftAmount - Aexpr
            GRBLinExpr lhs2 = b.shiftAmount;
            lhs2 -= Aexpr;
            std::string cNeg = "absNeg_"+std::to_string(e);
            model.addConstr(lhs2 <= uVars[e], cNeg);
        }

        // 目標函數
        GRBLinExpr obj = 0.0;
        for(int e=0; e<E; e++){
            obj += uVars[e];
        }
        model.setObjective(obj, GRB_MINIMIZE);

        // solve
        model.optimize();
        int status = model.get(GRB_IntAttr_Status);

        if(status == GRB_OPTIMAL){
            std::cout << "Optimal solution found.\n";
            std::cout << "ObjVal = " << model.get(GRB_DoubleAttr_ObjVal) << "\n";
	    std::cout << "Number of Variables: "<< model.get(GRB_IntAttr_NumVars) << std::endl;
	    std::cout << "Number of Constraints: "<< model.get(GRB_IntAttr_NumConstrs) << std::endl;
	    std::cout << "Solve time from Gurobi: "<< model.get(GRB_DoubleAttr_Runtime)<< std::endl;

            // 輸出解
            for(int e=0; e<E; e++){
                auto &b = boundaries[e];
                double uVal   = uVars[e].get(GRB_DoubleAttr_X);
                double aArea  = actualAreaExpr[e].getValue(); // getValue() => evaluate
                double ratio  = (std::fabs(b.shiftAmount)>1e-9)? (aArea/b.shiftAmount) : 0.0;
		double b1Val = 0.0, b2Val = 0.0, d1Val = 0.0, d2Val = 0.0;
                if(b.shift_Direction == 2){
                    // corner net => check bVar1, bVar2
                    b1Val = bVar1[e].get(GRB_DoubleAttr_X);
                    b2Val = bVar2[e].get(GRB_DoubleAttr_X);
                    d1Val = delta1[e].get(GRB_DoubleAttr_X);
                    d2Val = delta2[e].get(GRB_DoubleAttr_X);
                    //std::cout << "  (b1="<<b1Val<<", b2="<<b2Val << ", delta1="<<d1Val<<", delta2="<<d2Val<<")";
                } else {
                    d1Val = delta1[e].get(GRB_DoubleAttr_X);
                    //std::cout << "  (delta=" << dVal << ")";
                }
                std::cout<<"Boundary "<<b.boundaryID<<" (shiftDir="<<b.shift_Direction
                         <<"): b1= "<<b1Val<<", b2= "<<b2Val<< ", delta1= "<<d1Val<<", delta2="<<d2Val
                         <<", shiftAmt="<<b.shiftAmount
                         <<", actualArea="<<aArea
                         <<", ratio="<<ratio
                         <<", u="<<uVal<<"\n";

                std::cout << "\n";
            }
	    //normal net平移後的結果更新到netInfo & commonBoundary
	    //Phase2UpdateAllInfo_normal_nets(deltaVector, boundaries, nets);
	    //outputNetsInfo(nets);
	    outputCommonBoundaries(boundaries);
	    //outputNetsInfo_drawing(nets);
	    //exportToCSV(boundaries,actualAreaVector,ratioVector);

        } else {
            std::cout<<"No optimal solution. status="<<status<<"\n";
        }

    } catch(GRBException &e){
        std::cerr<<"GRB Error code="<< e.getErrorCode()<<"\n"<< e.getMessage()<<"\n";
    } catch(...){
        std::cerr<<"Exception\n";
    }

    return 0;
}
