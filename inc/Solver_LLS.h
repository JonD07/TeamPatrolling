/*
 * Solver_LLS.h
 * LLS -> Launch, Land, Switch 
 * Created by:	Jack "Siggy" Sigler 
 * On: 			Jan 28, 2025
 *
 * Description: This solver builds off of the ILO solver and specifally aims to enchance the case where the 
 * optimizer places a launch and land action right on top of each other but actually wants them to be switched. 
 * This solver will attempt to switch Launch/Land actions that fall within a certain distance tolerance of each other 
 * as long as a switch makes a improvement  
 */

#pragma once

#include <tuple>
#include <queue>
#include <cmath> 
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solver.h"
#include "KMeansSolver.h"
#include "VRPSolver.h"
#include "LaunchOptimizer.h"
#include "gurobi_c++.h"

#define DEBUG_ILO	DEBUG || 0


class Solver_LLS : public Solver {
public:
	Solver_LLS();
	~Solver_LLS();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
private:
	LaunchOptimizer optimizer;
	void LLSRelaxAll(int ugv_num, std::vector<std::vector<int>>& drones_to_UGV, PatrollingInput* input, Solution* sol_current);
	bool areActionsOverlapping(const UGVAction& action1, const UGVAction& action2); 
	bool updateSubtours(int drone_id, Solution* sol_final);
};
