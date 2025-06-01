/*
 * Solver_LOIRS.h
 * Created by:	Jonathan Diller
 * On: 			May 30, 2025
 *
 * Description: Launch-Optimizer with Iterative Swapping + Replanning
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
#include "LaunchOptimizerOBS.h"
#include "ActionSwapper.h"

#define DEBUG_LOS_IR	DEBUG || 0


class Solver_LOIRS : public Solver {
public:
	Solver_LOIRS(bool swapping = true, bool replanning = true);
	~Solver_LOIRS();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
private:
	bool bSwapping;
	bool bReplanning;

	LaunchOptimizer optimizer;
	LaunchOptimizerOBS optimizerOBS;
	ActionSwapper swapper;
};
