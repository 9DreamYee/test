#include <vector>
#include <fstream>
#include <cmath>
#include "gurobi_c++.h"
#include "ParseNetInfoToGurobi.h"
using namespace std;
int main(int argc, char *argv[]) {
    std::ifstream file(argv[1]);
    auto nets = parseNetsInfo(file);
    try {
        // 初始化 Gurobi 環境
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "ring_area_allocation.log");
        env.start();
	
	/* 
	//手key面積資料
	//256io_52 nets
	int N = 52;
	vector<double> A = {1.61055e+07,1.38474e+07,7.61658e+06,1.30587e+07,6.15807e+06,8.42197e+06,2.77238e+07,8.34062e+06,8.42294e+06,8.57898e+06,8.58015e+06,1.11709e+07,9.42195e+06,3.82470e+07,1.89281e+07,5.98329e+06,8.44603e+06,9.41745e+06,7.56782e+06,2.78106e+07,8.37709e+06,8.46303e+06,8.58963e+06,8.64232e+06,6.77133e+06,1.86072e+07,3.60425e+07,5.08707e+06,1.80850e+07,7.75023e+06,8.25137e+06,8.15294e+06,2.79092e+07,8.42829e+06,8.50245e+06,8.44091e+06,8.86571e+06,9.78540e+06,1.00892e+07,4.03501e+07,8.48133e+06,1.54182e+07,8.49221e+06,7.84779e+06,8.98102e+06,2.75579e+07,8.37381e+06,8.45271e+06,8.60359e+06,8.61971e+06,8.53785e+06,3.55951e+07};
	*/
	/*
	//16 nets  one side shorter
	//精度可能不足
	int N = 16;
	vector<double> A = { 1.9816e+07 , 4.31725e+07,1.44806e+07,1.40609e+07 , 1.68449e+07,7.91898e+07 ,  4.52008e+07, 3.26731e+07,2.75996e+07 , 4.17588e+07, 3.30298e+07, 5.41461e+07, 2.64275e+07,
	5.17217e+07,9.81793e+07,8.56985e+07 };
	*/

	double sumA = 0.0;
	int N = nets.size();
        for (int i = 0; i < nets.size(); i++) {
            sumA += nets[i].areaInitial;
	    std::cout<<nets[i].areaInitial<<std::endl;
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
            GRBLinExpr y_i = nets[i].areaInitial 
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
            //cout << "----- Variable Solutions -----" << endl;
	    //output_to_MILP_paraser
	    int rows = N, cols = N;
	    //vector<vector<int>> matrix(rows,vector<int>(cols));
	    vector<vector<int>> matrix(rows,vector<int>(2));
	    for(int i = 0;i < rows; i++){
	    	for(int j = 0; j < 2; j++){
			matrix[i][j] = 0;
		}
	    }
	    for (int i = 0; i < N; i++) {
		int ip1 = modIdx(i+1);
		int im1 = modIdx(i-1);
		// forward
		if(xForward[i].get(GRB_DoubleAttr_X)!=0){
			//cout<< "Area from Polygon["<<i<<"] forward to Ploygon["<<ip1<<"]: "<<xForward[i].get(GRB_DoubleAttr_X)<<endl;
			matrix[i][0] = xForward[i].get(GRB_DoubleAttr_X);
			//cout<<"net"<<i<<" to net "<<ip1<<" shiftamount: "<<xForward[i].get(GRB_DoubleAttr_X)<<endl;
		}
		// backward
		if(xBackward[i].get(GRB_DoubleAttr_X)!=0){
                	//cout<< "Area from Polygon["<<i<<"] backward to Ploygon["<<im1<<"]: "<<xBackward[i].get(GRB_DoubleAttr_X)<<endl;
			matrix[i][1] = 0-xBackward[i].get(GRB_DoubleAttr_X);
			//cout<<"net"<<i<<" to net "<<im1<<" shiftamount: "<< 0 - xBackward[i].get(GRB_DoubleAttr_X)<<endl;
		}
            }
	    //輸出格式:以net為單位 i為net總數 j = 2 [i][0]代表該條net forward的面積值 [i][1]代表該條net backward的面積值
	    ofstream outfile("Gurobi_area_assignment_result.txt");
	    for(int i = 0;i < rows; i++){
	    	for(int j = 0; j < 2; j++){
		    outfile<< matrix[i][j]<<" ";
		}
		outfile<<endl;
	    }
	   
	    /*
	    // output_readable
            for (int i = 0; i < N; i++) {
		output_i = i+1;
		output_i_forward = modIdx(i+1)+1;
		output_i_backward = modIdx(i-1)+1;
                cout<< "Area from Polygon["<<output_i<<"] forward to Ploygon["<<output_i_forward<<"]: "<<xForward[i].get(GRB_DoubleAttr_X)<< endl;
                cout<< "Area from Polygon["<<output_i<<"] backward to Ploygon["<<output_i_backward<<"]: "<<xBackward[i].get(GRB_DoubleAttr_X)<< endl;
		if (i < N-1)
		cout<< "---Next Polygon---"<<endl;
            }
	    */
	    
            // 輸出每個多邊形最終分配的面積 y_i
            cout << "----- Final Assigned Areas (y_i) -----" << endl;
            for (int i = 0; i < N; i++) {
                int ip1 = modIdx(i+1);
                int im1 = modIdx(i-1);
		output_i = i+1;
                double y_i = nets[i].areaInitial 
                             + xForward[im1].get(GRB_DoubleAttr_X)  // 從P_(i-1)流入i的量
                             + xBackward[ip1].get(GRB_DoubleAttr_X) // 從P_(i+1)流入i的量
                             - xForward[i].get(GRB_DoubleAttr_X)    // i流出到i+1
                             - xBackward[i].get(GRB_DoubleAttr_X);   // i流出到i-1

                cout << "A[" << output_i << "] (Total area for Polygon "<<output_i<<"): " << y_i << endl;
            }
        } 
	else {
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
