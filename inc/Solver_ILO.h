/*
 * Solver_ILO.h
 *
 * Created by:	Jonathan Diller
 * On: 			Aug 7, 2024
 *
 * Description: This solver finds an initial solution using the baseline method, then
 * iteratively optimizes launch/receive points and re-solves TSP instances for each
 * drone sub-tour. This continues until no more improvement is found.
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

#define DEBUG_ILO	DEBUG || 0


class Solver_ILO : public Solver {
public:
	Solver_ILO();
	~Solver_ILO();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
private:
	LaunchOptimizer optimizer;
};
