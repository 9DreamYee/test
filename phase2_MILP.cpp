#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include "gurobi_c++.h"
#include "ParseNetInfoToGurobi.h"

void exportToCSV(std::vector<commonBoundary> &boundaries, std::vector<double> actualAreaVector, std::vector<double> ratioVector){
	std::ofstream csvFile("net_area_data.csv");
	csvFile << "BoundayID,ShiftAmount,ActualArea,Ratio\n";
	for(int i = 0; i < boundaries.size(); i++){
		csvFile << boundaries[i].boundaryID<<","
		        << boundaries[i].shiftAmount<<","
			<< actualAreaVector[i]<<","
			<< (ratioVector[i] >= 0 ? ratioVector[i] : 0)
			<<"\n";
	}
	csvFile.close();
}
int main(int argc, char* argv[])
{
    // 1. 讀取 commonBoundary
    std::ifstream file(argv[1]);
    auto nets = parseNetsInfo(file);
    auto boundaries = buildCommonBoundaries(nets);
    Rectangle innerRect;
    Rectangle outterRect =Rectangle(-1.4e+08,-1.4e+08,2.8e+08,2.8e+08);
    sortBoundaryClockwise(boundaries,innerRect);
    //同時特例處利Boundary 0 
    UpdateCommonBoundaryInfo(boundaries,nets,innerRect,outterRect);
    try {
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "phase2_cornerMILP.log");
        env.start();
        GRBModel model = GRBModel(env);

        int E = boundaries.size();

        // -- 變數定義 --
        // For corner net: we define bVar1, bVar2 (binary) + delta1, delta2
        // For normal net: we define a single delta1, plus a dummy bVar or skip it
        std::vector<GRBVar> bVar1(E), bVar2(E);   // 0/1
        std::vector<GRBVar> delta1(E), delta2(E);
        std::vector<GRBVar> uVars(E);            // 誤差

        // 設定個人認定合適的Big-M值，對應原線 or斜線最大可平移量
        // 可根據 b.shiftMax、 b.corner_area 等計算
        // 這裡簡單假設 M1= b.shiftMax, M2 = b.shiftMax
        // (若更嚴謹可考慮 alpha, corner_area)
        for(int e=0; e<E; e++){
            auto &b = boundaries[e];

            // 誤差變數
            std::string uName = "u_"+std::to_string(b.boundaryID);
            uVars[e] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, uName);

            double M1 = b.shiftMin;  // or something else
            double M2 = b.shiftMax;  // or something else

            if(b.shift_Direction == 2) {
                // corner net
                // Binary var
                std::string b1Name = "b1_"+std::to_string(b.boundaryID);
                bVar1[e] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, b1Name);

                std::string b2Name = "b2_"+std::to_string(b.boundaryID);
                bVar2[e] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, b2Name);

                // delta1, delta2
                std::string dName1 = "delta1_"+std::to_string(b.boundaryID);
                std::string dName2 = "delta2_"+std::to_string(b.boundaryID);

                // 若 alpha=0 => 原線不可用 => M1=0 or forced bVar1=0
                if(b.alpha <= 1e-9) {
                    // 直接令 M1=0 => delta1=0, solver只能用 bVar2
                    M1 = 0;
                }
                delta1[e] = model.addVar(0.0, M1, 0.0, GRB_CONTINUOUS, dName1);
                delta2[e] = model.addVar(0.0, M2, 0.0, GRB_CONTINUOUS, dName2);

            } 
	    
	    else {
                // normal net
                // 單一 delta => 不需要 bVar2
                bVar1[e] = GRBVar(); // dummy
                bVar2[e] = GRBVar(); // dummy

                std::string dName = "delta_"+std::to_string(b.boundaryID);
                delta1[e] = model.addVar(b.shiftMin, b.shiftMax, 0.0, GRB_CONTINUOUS, dName);

                // dummy delta2
                delta2[e] = GRBVar(); // not used
            }
        }
        model.update();

        // -- 加入約束 --
        for(int e=0; e<E; e++){
            auto &b = boundaries[e];

            if(b.shift_Direction == 2) {
                // 1) 互斥: bVar1[e] + bVar2[e] <= 1
                model.addConstr(bVar1[e] + bVar2[e] == 1, 
                                "chooseLine_"+std::to_string(b.boundaryID));

                // 2) Big-M => delta1[e] <= bVar1[e]*M1
                //            delta2[e] <= bVar2[e]*M2
                double M1 = delta1[e].get(GRB_DoubleAttr_UB); 
                double M2 = delta2[e].get(GRB_DoubleAttr_UB);
                model.addConstr(delta1[e] <= bVar1[e]*M1, 
                                "useLine1_"+std::to_string(b.boundaryID));
                model.addConstr(delta2[e] <= bVar2[e]*M2,
                                "useLine2_"+std::to_string(b.boundaryID));

                // 3) corner_area: alpha*delta1 <= corner_area
                if(b.alpha > 1e-9) {
                    GRBLinExpr lhs = b.alpha * delta1[e];
                    model.addConstr(lhs <= b.cornerArea * bVar1[e],
                                    "cornerLimit_"+std::to_string(b.boundaryID));
                }
            }
            // if normal net => do nothing special here
        }

        // -- ActualArea & 誤差 --
        // actualArea_e = alpha*delta1 + alpha_corner*delta2
        // u_e >= actualArea - shiftAmount
        // u_e >= shiftAmount - actualArea
        for(int e=0; e<E; e++){
            auto &b = boundaries[e];
	    //GRBQuadExpr tempArea;
            GRBLinExpr actualArea = 0.0;
            if(b.shift_Direction == 2) {
                // corner net => two segments
		/*
		tempArea += bVar1[e] * (b.alpha * delta1[e]);
		tempArea += bVar2[e] * (b.alpha_corner * delta2[e] + b.cornerArea);
		actualArea = GRBLinExpr (tempArea);
		*/
                actualArea += (b.alpha * delta1[e]);
                actualArea += (b.alpha_corner * delta2[e] );
		
                //int b2 = delta1[e].get(GRB_DoubleAttr_X);
            } 
	    
	    else {
                // normal net => single direction
                actualArea += b.alpha * delta1[e];
            }

            //誤差約束:  u_e >= alpha_e * delta_e - shiftAmount[e]
            //           u_e >= shiftAmount[e] - alpha_e * delta_e
            {
                GRBLinExpr lhs1 = actualArea; 
                lhs1 -= b.shiftAmount;
                model.addConstr(uVars[e] >= lhs1, "absPos_"+std::to_string(e));

                GRBLinExpr lhs2 = b.shiftAmount; 
                lhs2 -= actualArea;
                model.addConstr(uVars[e] >= lhs2, "absNeg_"+std::to_string(e));
            }
        }

        // -- 目標函數: minimize sum of u_e
        GRBLinExpr obj = 0.0;
        for(int e=0; e<E; e++){
            obj += uVars[e];
        }
        model.setObjective(obj, GRB_MINIMIZE);

        // -- solve --
        model.optimize();
        int status = model.get(GRB_IntAttr_Status);
        if(status == GRB_OPTIMAL) {
            std::cout<<"Optimal solution found.\nObjVal= "<< model.get(GRB_DoubleAttr_ObjVal)<<"\n";
	    std::cout << "Number of Variables: "<< model.get(GRB_IntAttr_NumVars) << std::endl;
	    std::cout << "Number of Constraints: "<< model.get(GRB_IntAttr_NumConstrs) << std::endl;
	    std::cout << "Solve time from Gurobi: "<< model.get(GRB_DoubleAttr_Runtime)<< std::endl;
            for(int e=0; e<E; e++){
                auto &b = boundaries[e];
                double uVal = uVars[e].get(GRB_DoubleAttr_X);
                double d1=0.0, d2=0.0, actualArea=0.0, ratio=0.0;

                if(b.shift_Direction == 2) {
                    d1 = delta1[e].get(GRB_DoubleAttr_X);
                    d2 = delta2[e].get(GRB_DoubleAttr_X);
                    actualArea = b.alpha*d1 + b.alpha_corner*d2;
                } else {
                    d1 = delta1[e].get(GRB_DoubleAttr_X);
                    actualArea = b.alpha*d1;
                }
                if(std::fabs(b.shiftAmount)>1e-9){
                    ratio = actualArea / b.shiftAmount;
                }
                std::cout<<"Boundary "<<b.boundaryID<<" (shiftDir="<<b.shift_Direction
                         <<"): delta1="<<d1<<", delta2="<<d2
                         <<", shiftAmt="<<b.shiftAmount
                         <<", actualArea="<<actualArea
                         <<", ratio="<<ratio
                         <<", u="<<uVal<<"\n";
            }
	    //normal net平移後的結果更新到netInfo & commonBoundary
	    //Phase2UpdateAllInfo_normal_nets(deltaVector, boundaries, nets);
	    //outputNetsInfo(nets);
	    /*
	    outputCommonBoundaries(boundaries);
	    outputNetsInfo_drawing(nets);
	    */
	    //exportToCSV(boundaries,actualAreaVector,ratioVector);
        } 
	
	else {
            std::cout << "No optimal solution, status= " << status << "\n";
        }

    } catch(GRBException e){
        std::cerr<<"GRB Error code="<< e.getErrorCode()<<"\n"<< e.getMessage()<<"\n";
    } catch(...){
        std::cerr<<"Exception\n";
    }

    return 0;
}
