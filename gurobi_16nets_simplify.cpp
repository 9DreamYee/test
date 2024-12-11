#include <iostream>
#include <vector>
#include <cmath>
#include "gurobi_c++.h"
using namespace std;

int main(int argc, char *argv[]) {
    try {
        // 初始化 Gurobi 環境
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "ring_area_allocation.log");
        env.start();

        // 假設參數 (請依實際問題修改)
        int N = 16;
	//int N = 4;
	//vector<double> A = {170,10,10,10};
	//vector<double> A = {25,10,5,10,50};
        vector<double> A = {427,81,204,53.5,70.5,80,369.5,213.5,155.5,128.5,196,159.5,255.5,120.5,254.5,489};
	double sumA = 0.0;
        for (int i = 0; i < N; i++) {
            sumA += A[i];
        }
        double Aavg = sumA / N;

        // 建立模型
        GRBModel model = GRBModel(env);

        // 定義變數:
        // xForward[i] = x_{i,i+1}, xBackward[i] = x_{i,i-1}
        vector<GRBVar> xForward(N);
        vector<GRBVar> xBackward(N);
	//vector<GRBVar> zVars(N);
        for (int i = 0; i < N; i++) {
            xForward[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xForward_"+to_string(i));
            xBackward[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "xBackward_"+to_string(i));
	    //zVars[i] = model.addVar(0.0,GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z_"+to_string(i));
        }

        model.update();

        auto modIdx = [&](int idx) {return (idx + N) % N;};
        // 加入面積平衡約束
        for (int i = 0; i < N; i++) {
            int ip1 = modIdx(i+1); // i+1 backward
            int im1 = modIdx(i-1); // i-1 forward

            // y_i = A[i] + xForward[im1] + xBackward[ip1] - (xForward[i] + xBackward[i])
            // 要 y_i = Aavg
            GRBLinExpr y_i = A[i] 
                             + xForward[im1]   // 從P_(i-1)流入i
                             + xBackward[ip1]  // 從P_(i+1)流入i
                             - xForward[i]     // i流出到i+1
                             - xBackward[i];    // i流出到i-1

            model.addConstr(y_i == Aavg, "balance_"+to_string(i));
	/*    
	// 面積偏差輔助變數(最終面積與原始面積的偏差)
	GRBLinExpr diff = (xForward[im1] + xBackward[ip1] - xForward[i]- xBackward[i]);
	model.addConstr(zVars[i] >= diff, "z_pos_"+to_string(i));
	model.addConstr(zVars[i] >= - diff, "z_neg_"+to_string(i));
        */
	}

        // 設定目標函數: 最小化總流量
        GRBLinExpr obj = 0.0;
	double alpha = 1.0; 
	double beta = 0.0;
        for (int i = 0; i < N; i++) {
            obj += alpha * (xForward[i] + xBackward[i]);
	    //obj += beta * zVars[i];
        }
        model.setObjective(obj, GRB_MINIMIZE);

        // 求解
        model.optimize();

        int status = model.get(GRB_IntAttr_Status);
	int output_i = 0,output_i_forward = 0,output_i_backward = 0;
        if (status == GRB_OPTIMAL) {
            cout << "Optimal solution found." << endl;
            cout << "Objective Value: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
	    cout << "Number of Variables: "<< model.get(GRB_IntAttr_NumVars) << endl;
	    cout << "Number of Constraints: "<< model.get(GRB_IntAttr_NumConstrs) << endl;
            // 輸出解
            cout << "----- Variable Solutions -----" << endl;
            for (int i = 0; i < N; i++) {
		output_i = i+1;
		output_i_forward = modIdx(i+1)+1;
		output_i_backward = modIdx(i-1)+1;
                cout<< "Area from Polygon["<<output_i<<"] forward to Ploygon["<<output_i_forward<<"]: "<<xForward[i].get(GRB_DoubleAttr_X)<< endl;
                cout<< "Area from Polygon["<<output_i<<"] backward to Ploygon["<<output_i_backward<<"]: "<<xBackward[i].get(GRB_DoubleAttr_X)<< endl;
		if (i < N-1)
		cout<< "---Next Polygon---"<<endl;
		/*
		cout << "xForward[" << i << "] = " << xForward[i].get(GRB_DoubleAttr_X) << endl;
                cout << "xBackward[" << i << "] = " << xBackward[i].get(GRB_DoubleAttr_X) << endl;
		*/
            }

            // 輸出每個多邊形最終分配的面積 y_i
            cout << "----- Final Assigned Areas (y_i) -----" << endl;
            for (int i = 0; i < N; i++) {
                int ip1 = modIdx(i+1);
                int im1 = modIdx(i-1);
		output_i = i+1;
                double y_i = A[i] 
                             + xForward[im1].get(GRB_DoubleAttr_X)  // 從P_(i-1)流入i的量
                             + xBackward[ip1].get(GRB_DoubleAttr_X) // 從P_(i+1)流入i的量
                             - xForward[i].get(GRB_DoubleAttr_X)    // i流出到i+1
                             - xBackward[i].get(GRB_DoubleAttr_X);   // i流出到i-1

                cout << "A[" << output_i << "] (Total area for Polygon "<<output_i<<"): " << y_i << endl;
            }
        } else {
            cout << "No optimal solution. Status = " << status << endl;
        }

    } catch (GRBException e) {
        cerr << "Error code = " << e.getErrorCode() << endl;
        cerr << e.getMessage() << endl;
    } catch (...) {
        cerr << "Exception during optimization" << endl;
    }

    return 0;
}
