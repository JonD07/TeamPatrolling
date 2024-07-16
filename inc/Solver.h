/*
 * Solver.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 25, 2024
 *
 * Description: Parent class for all solvers
 */

#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <cmath>
#include <complex>

#include "Solution.h"
#include "PatrollingInput.h"
#include "Utilities.h"
#include "Roots.h"

#define DEBUG_SOLVER	0 || DEBUG

class Solver {
public:
	Solver();
	virtual ~Solver();

	virtual void Solve(PatrollingInput* input, Solution* sol_final) = 0;

protected:
	double calcChargeTime(double J);
};
