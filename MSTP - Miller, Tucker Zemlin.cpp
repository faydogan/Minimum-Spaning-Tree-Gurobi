/*
	MIT License

	Copyright (c) 2018 Furkan AYDOGAN

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

	Minimum Spanning Tree Problem with Conflicts - Miller, Tucker and Zemlin Formulation
*/

#include "gurobi_c++.h"
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

// Converter For Integer to String
string itos(int i) { stringstream s; s << i; return s.str(); }

int main() {
	//Read Edge Values From "data.txt" file
	ifstream inF;
	inF.open("data.txt");
	vector <vector <int>> C;
	int num;
	int i, j, k, n, e;
	inF >> n;
	inF >> e;
	for (i = 0; i < e; i++) {
		vector<int> row;
		for (k = 0; k < 4; k++) {
			inF >> num;
			row.push_back(num);
		}
		C.push_back(row);
	}
	inF.close();
	//Put All Edge Values are in C Vector Now
	
	//Edges with Conflict, read from "conflicts.txt" as pairs such as: {X10,X23} 1 0 2 3.
	vector <vector <int>> constraints;
	ifstream inF2;
	inF2.open("constraints.txt");
	int num_const;
	inF2 >> num_const;
	for (i = 0; i < num_const; i++) {
		vector<int> row2;
		for (k = 0; k < 4; k++) {
			inF2 >> num;
			row2.push_back(num);
		}
		constraints.push_back(row2);
	}
	inF2.close();
	//All conflicting edges are in "constraints" vector
	
	//n:(node) and e:(edge) values will be printed for information
	cout << "n : " << n << " e  :  " << e << endl;
	
	//Reseting Gurobi Envoriment, It will provide us brand new env. for our run
	GRBEnv resetParams();
	
	//Define X ve U variables on Gurobi Env.
	GRBEnv env = GRBEnv();
	GRBVar **X = NULL;
	GRBVar *U = NULL;

	//Entering dimention and size
	X = new GRBVar*[n];
	U = new GRBVar[n];
	for (i = 0; i < n; i++) {
		X[i] = new GRBVar[n];
	}

	try {
		//Clear Model Cache on Gurobi
		GRBModel reset();
		GRBModel model = GRBModel(env);

		//Creation Xij, Ui Variables on GUROBI
		int **Costs = new int*[n];
		U[0] = model.addVar(0.0, (n - 1), 0.0, GRB_CONTINUOUS, "U_" + itos(0));
		for (i = 0; i < n; i++) {
			Costs[i] = new int[n];
			if(i != 0)
				U[i] = model.addVar(1.0, (n - 1), 0.0, GRB_CONTINUOUS, "U_" + itos(i));

		}

		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++) {
				for (k = 0; k < C.size(); k++) {
					if ((i == C[k][0] && j == C[k][1]) || (j == C[k][0] && i == C[k][1])) {
						X[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "X_" + itos(i) + "_" + itos(j));
						Costs[i][j] = C[k][3];
						break;
					}
					else {
						Costs[i][j] = 0;
					}
				}
			}
		}
		
		//Xij, Ui GUROBI variables are created, 
		//Inside Costs(ij) Objective Function Coefficients Will Be Stored
		
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++)
				cout << "Costs_" << i << "_" << j << " : " << Costs[i][j] << endl;
			cout << endl;
		}
		
		/*--------------------------------------------------------------------
			ADDING OBJ FUNCT and CONSTRAINTS INTO MODEL
		----------------------------------------------------------------------*/
		
		//ASSIGN OBJ FUNCT
		GRBLinExpr obj = 0;
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++) {
				if (Costs[i][j] != 0)
					obj += (Costs[i][j] * X[i][j]);
			}
		}
		model.setObjective(obj, GRB_MINIMIZE);
		
		//ASSIGN CONSTRAINTS
		GRBLinExpr objs = 0;
		for (i = 1; i < n; i++) {
			objs = 0;
			for (j = 0; j < n; j++) {
					if (Costs[i][j] != 0)
						objs += X[j][i];
			}
			if(i != 0)
			model.addConstr(objs == 1, "A_" + itos(i));
		}

		GRBLinExpr objs2 = 0;
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++) {
				if (Costs[i][j] != 0)
					objs2 += X[i][j];
			}
		}
		model.addConstr(objs2 == n-1, "TOT");
		
		//AS CONSTRAINT (6) U1 WILL BE 1
		model.addConstr(U[0] == 0.0, "root");

	
		//Miller Tucker Zemlin CONSTRAINT
		//(4) Ui - Uj + (n * Xij) <= n - 1 for each i,j pairs ;
		for (i = 1; i < n; i++) {
			for (j = 1; j < n; j++) {
				if (Costs[i][j] != 0 && i != j) {
					model.addConstr(U[i] - U[j] + (n * X[i][j]) <= n - 1, "mtz_" + itos(i) + "_" + itos(j));
				}
			}
		}


		/*--------------------------------------------------------------------
			SOLVE MODEL ON GUROBI (WITHPUT CONFLICTS)
		----------------------------------------------------------------------*/

		cout << "-----------------------------------------------" << endl << "SOLVING WITHOUT CONFLICT" << endl << "----------------------------------------------" << endl;
		model.optimize();
		model.write("out.lp");
		//If you need result details in a seperate file, erase comment slashes below two lines
		//model.computeIIS();
		//model.write("MSTP_model.ilp"); 

		/*--------------------------------------------------------------------
			Print Out The Results
		----------------------------------------------------------------------*/
		//Xij ve Cij Results
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++) {
				if (Costs[i][j] != 0)
					if(X[i][j].get(GRB_DoubleAttr_X) != 0)
						cout << X[i][j].get(GRB_StringAttr_VarName) << " :" << X[i][j].get(GRB_DoubleAttr_X) << "   " << "C for" << X[i][j].get(GRB_StringAttr_VarName) << " :" << X[i][j].get(GRB_DoubleAttr_Obj) << endl;
			}
			cout << endl;
		}
		//Ui Results
		for (i = 1; i < n; i++) {
			cout << "U_" << i << " : " << U[i].get(GRB_DoubleAttr_X) << endl;
		}
		

		/*--------------------------------------------------------------------
			CHECK IF ANY CONFLICT EXISTS IN CURRENT SOLUTION
		----------------------------------------------------------------------*/

		cout << "----------------------------------------------------" << endl << "CHECKING CONFLICTS IN SOLUTION" << endl << "----------------------------------------------------" << endl;
		string checkConflict = "None";
		for (i = 0; i < num_const; i++) {
			if (X[constraints[i][0]][constraints[i][1]].get(GRB_DoubleAttr_X) == 1 && X[constraints[i][2]][constraints[i][3]].get(GRB_DoubleAttr_X) == 1) {
				checkConflict = "Exists";
				cout << "Conflict Violation : X_" << constraints[i][0] << "_" << constraints[i][1] << " and " << "X_" << constraints[i][2] << "_" << constraints[i][3] << " are both in solution." << endl;
			}
		}


		/*--------------------------------------------------------------------
			ADD FOUND CONFLICTS INTO MODEL AND SOLVE IT AGAIN
		----------------------------------------------------------------------*/

		if (checkConflict == "Exists") {
			cout << "----------------------------------------------------" << endl << "TRY TO SOLVE MODEL WITH CONFLICT CONSTRAINTS" << endl << "----------------------------------------------------" << endl;
			for (i = 0; i < num_const; i++) {
				model.addConstr(X[constraints[i][0]][constraints[i][1]] + X[constraints[i][2]][constraints[i][3]] == 1.0, "CFLC_" + itos(i) + "_" + itos(j));
			}

			model.optimize();
			model.write("out_with_conflicts.lp");
			
			//Xij ve Cij Results
			for (i = 0; i < n; i++) {
				for (j = 0; j < n; j++) {
					if (Costs[i][j] != 0)
						if (X[i][j].get(GRB_DoubleAttr_X) != 0)
							cout << X[i][j].get(GRB_StringAttr_VarName) << " :" << X[i][j].get(GRB_DoubleAttr_X) << "   " << "C for" << X[i][j].get(GRB_StringAttr_VarName) << " :" << X[i][j].get(GRB_DoubleAttr_Obj) << endl;
				}
				cout << endl;
			}
			
			//Ui Results
			for (i = 1; i < n; i++) {
				cout << "U_" << i << " : " << U[i].get(GRB_DoubleAttr_X) << endl;
			}
		}
		else {
			cout << endl << "THERE IS NO CONFLICT IN SOLUTION!" << endl;
		}




	}
	catch (GRBException e) {
		cout << "Error number: " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}

	catch (...) {
		cout << "Error during optimization" << endl;
	}


	system("pause");

	return 0;
}
