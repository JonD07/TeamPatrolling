/*
 * LaunchOptimizer.h
 *
 * Created by:	Jonathan Diller
 * On: 			Aug 7, 2024
 *
 * Description: This class optimizes the launch and receive points of each drone for a give set
 * of drones and an accompanying UGV. It does this via a second order cone program using Gurobi.
 */

#pragma once

#include <tuple>
#include <queue>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solution.h"
#include "gurobi_c++.h"

#define DEBUG_LAUNCHOPT	DEBUG || 0

#define CONST_RELAXATION(X)		X,X+0.1



class LaunchOptimizer {
public:
	LaunchOptimizer();
	~LaunchOptimizer();

	// Attempts to optimize the location/time of all launch/land actions assigned to this UGV. Returns true if Gurobi finds a valid solution, false otherwise.
	bool OptLaunching(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_final);

protected:
};
