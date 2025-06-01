/*
 * Solver_BaselineOBS.h
 *
 * Created by:	Jonathan Diller
 * On: 			May 29, 2025
 *
 * Description: Runs the standard baseline algorithm, but also avoids obstacles.
 */

#pragma once

#include <tuple>
#include <queue>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solver.h"

#define DEBUG_GREEDY	DEBUG || 0


class Solver_Baseline_OBS : public Solver {
public:
	Solver_Baseline_OBS();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
private:
};
