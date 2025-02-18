#include <iostream>
#include <vector>
#include "gurobi_c++.h"
#include "ParseNetInfoToGurobi.h"
/*
struct BoundaryInfo {
    int    boundaryID;
    double alpha;
    double shiftAmount;
    double shiftMin;
    double shiftMax;
};
*/
int main(int argc, char* argv[]) 
{
    // 0. 建立netInfo & commonBoundary資料結構
    std::vector<double> deltaVector;
    std::ifstream file(argv[1]);
    auto nets = parseNetsInfo(file);
    auto boundaries = buildCommonBoundaries(nets);
    //同時特例處利Boundary 1
    UpdateCommonBoundaryInfo(boundaries,nets);

    try {
        // 1. 初始化 Gurobi
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "phase2.log");
        env.start();
        GRBModel model = GRBModel(env);

        // -------------------------------------------
        // 2. 假設已有BoundaryInfo(由phase1決定shiftAmount)
        // -------------------------------------------
        //std::vector<commonBoundary> boundaries;
        int E = boundaries.size();

        // -------------------------------------------
        // 3. 定義變數: delta_e 與 u_e
        // -------------------------------------------
        std::vector<GRBVar> deltaVars(E), uVars(E);
        for(int e=0; e<E; e++){
            auto &b = boundaries[e];
            // delta_e
            std::string dName = "delta_"+std::to_string(b.boundaryID);
            deltaVars[e] = model.addVar(b.shiftMin, b.shiftMax, 0.0, GRB_CONTINUOUS, dName);

            // u_e
            std::string uName = "u_"+std::to_string(b.boundaryID);
            uVars[e] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, uName);
        }
        model.update();

        // -------------------------------------------
        // 4. 誤差約束:  u_e >= alpha_e * delta_e - shiftAmount[e]
        //               u_e >= shiftAmount[e] - alpha_e * delta_e
        // -------------------------------------------
        for(int e=0; e<E; e++){
            auto &b = boundaries[e];
            GRBLinExpr lhs1 = b.alpha * deltaVars[e] - b.shiftAmount;
            model.addConstr(uVars[e] >= lhs1, "absPos_"+std::to_string(e));

            GRBLinExpr lhs2 = b.shiftAmount - b.alpha * deltaVars[e];
            model.addConstr(uVars[e] >= lhs2, "absNeg_"+std::to_string(e));
        }

        // -------------------------------------------
        // 5. 目標函數: min sum_e u_e
        // -------------------------------------------
        GRBLinExpr obj = 0.0;
        for(int e=0; e<E; e++){
            obj += uVars[e];
        }
        model.setObjective(obj, GRB_MINIMIZE);

        // -------------------------------------------
        // 6. optimize
        // -------------------------------------------
        model.optimize();

        int status = model.get(GRB_IntAttr_Status);
        //找到最佳解
	if(status == GRB_OPTIMAL){
            std::cout << "Optimal solution found.\n";
            std::cout << "ObjVal = " << model.get(GRB_DoubleAttr_ObjVal) << "\n";

            // 輸出解
            for(int e=0; e<E; e++){
                double deltaVal = deltaVars[e].get(GRB_DoubleAttr_X);
                double uVal     = uVars[e].get(GRB_DoubleAttr_X);
                auto &b = boundaries[e];
                // 實際對應的面積移動 = alpha_e * deltaVal
                double actualArea = b.alpha * deltaVal;
		deltaVector.push_back(deltaVal);
                std::cout 
                    << "Boundary " << b.boundaryID 
                    << ": delta= " << deltaVal
                    << ", shiftAmount= " << b.shiftAmount
                    << ", actualArea= " << actualArea
                    << ", u= " << uVal
                    << "\n";
            }
	    //normal net平移後的結果更新到netInfo & commonBoundary
	    Phase2UpdateAllInfo_normal_nets(deltaVector, boundaries, nets);
	    outputNetsInfo(nets);
	    outputCommonBoundaries(boundaries);
	    //outputNetsInfo_drawing(nets);
        } 
	//找不到最佳解
	else {
            std::cout << "No optimal solution. status= " << status << "\n";
        }

    }//end of try

    catch(GRBException e) {
        std::cerr << "Error code= " << e.getErrorCode() << "\n"
                  << e.getMessage() << "\n";
    } 
    catch(...) {
        std::cerr << "Exception during optimization\n";
    }

    return 0;
}
