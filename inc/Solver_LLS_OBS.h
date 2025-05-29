/*
 * Solver_LLS_OBS.h
 * LLS -> Launch, Land, Switch 
 * Created by:	Jonathan Diller
 * On: 			May 27, 2025
 *
 * Description: This solver builds off of the original LLS solver, but now with obstacles.
 * The general idea is to find an initial solution, optimize the solution while ignoring
 * obstacles, swap launch/land actions that all on top of one another, then run some basic
 * obstacle avoidance techniques.
 */

#pragma once

#include <tuple>
#include <queue>
#include <cmath> 
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solver_LLS.h"
#include "KMeansSolver.h"
#include "VRPSolver.h"
#include "LaunchOptimizer.h"
#include "gurobi_c++.h"

#define DEBUG_LLS_OBS	DEBUG || 0


class Solver_LLS_OBS : public Solver_LLS {
public:
	Solver_LLS_OBS();
	~Solver_LLS_OBS();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
	/*
	*  Perform the basic LLS, but don't worry about improvements. Swap all actions that are on top of
	*  each other and then optimize. This returns an optimized (consistent) solution.
	*/
	void LazyLLS(int ugv_num, std::vector<std::vector<int>>& drones_to_UGV, PatrollingInput* input, Solution* sol_current);
private:
};
