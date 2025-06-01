/*
 * Solver_LORS.h
 * LOSR..?
 * Created by:	Jonathan Diller
 * On: 			May 30, 2025
 *
 * Description: Launch-Optimizer with Replanning & Swapping
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

#define DEBUG_LORS	DEBUG || 0


class Solver_LORS : public Solver {
public:
	Solver_LORS(bool swapping = true, bool replanning = true);
	~Solver_LORS();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
private:
	bool bSwapping;
	bool bReplanning;

	LaunchOptimizer optimizer;
	LaunchOptimizerOBS optimizerOBS;
	ActionSwapper swapper;
};
