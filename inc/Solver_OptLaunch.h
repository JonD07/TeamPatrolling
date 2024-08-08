/*
 * Solver_OptLaunch.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jul 18, 2024
 *
 * Description: This solver takes the initial solution found using the baseline
 * algorithm and then optimizes the launch and receive locations of each drone
 * for each UGV stop. This is done via a convex NLP, which can be solved to
 * optimality in polynomial time.
 */

#pragma once

#include <tuple>
#include <queue>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solver.h"
#include "KMeansSolver.h"
#include "VRPSolver.h"
#include "LaunchOptimizer.h"
#include "gurobi_c++.h"

#define DEBUG_OPTLAUNCH	DEBUG || 1


class OptLaunchSolver : public Solver {
public:
	OptLaunchSolver();
	~OptLaunchSolver();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
private:
	LaunchOptimizer optimizer;
};
