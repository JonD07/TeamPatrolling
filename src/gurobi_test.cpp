#include "PatrollingInput.h"
#include "Solution.h"
#include "defines.h"
#include <tuple>
#include <queue>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solver.h"
#include "KMeansSolver.h"
#include "VRPSolver.h"
#include "LaunchOptimizerOBS.h"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "PatrollingInput.h"


/* Copyright 2024, Gurobi Optimization, LLC */

/* This example formulates and solves the following simple MIP model:

     maximize    x + y
     subject to  y <= 0.108x 
                y >= 0.108x + -8 
                y <= 0.139x - 12.721 
                y >= 0.139x - 20.721
*/


using namespace std;

void runOptimization() {
  try {
    GRBEnv env = GRBEnv(true);
    env.set("LogFile", "mip1.log");
    env.start();

    GRBModel model = GRBModel(env);
    UGVAction action1 = UGVAction(E_UGVActionTypes::e_MoveToPosition, 0, 0, 0, 0);
    UGVAction action2 = UGVAction(E_UGVActionTypes::e_MoveToPosition, 414.721, 44.925, 0, 0);
    UGVAction action3 = UGVAction(E_UGVActionTypes::e_MoveToPosition, 1681.2, 221, 0, 0);
    Location loc; 
    loc.x = 52.4;
    loc.y = 400; 
    Obstacle o1("1", "cirlce", loc, 9);
    std::string id = "1"; 
    GRBVar x = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x_" + id);
    GRBVar y = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_" + id);
    std::vector<GRBVar> coord;
    coord.push_back(x);
    coord.push_back(y);
    std::vector<std::vector<GRBVar>> act_pos_var;
    act_pos_var.push_back(coord);
    LaunchOptimizerOBS::addCorridorConstraints(&model, &act_pos_var, action1, action2, action3, o1);


    model.setObjective(x + y, GRB_MAXIMIZE);
    model.write("model.lp");      // Human-readable LP format
    model.optimize();



  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }
}
