/*
 * Solver_Baseline.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 25, 2024
 *
 * Description:
 */

#pragma once

#include <tuple>
#include <queue>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solver.h"

#define DEBUG_GREEDY	DEBUG || 1


class BaselineSolver : public Solver {
public:
	BaselineSolver();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
private:
};
