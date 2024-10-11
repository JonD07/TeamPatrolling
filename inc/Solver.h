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
#include "KMeansSolver.h"
#include "VRPSolver.h"

#define DEBUG_SOLVER	DEBUG || 0


// Data structure for when a drone arrives at the UGV
struct Arrival {
	double time;
	int ID;

	// Constructor
	Arrival(double t, int id) : time(t), ID(id) {}
};

// Comparison function to order Arrivals by time
struct CompareArrival {
	bool operator()(const Arrival& a1, const Arrival& a2) {
		return a1.time > a2.time;
	}
};

struct TSPVertex {
	int nID;
	double x;
	double y;
};


class Solver {
public:
	Solver();
	virtual ~Solver();

	virtual void Solve(PatrollingInput* input, Solution* sol_final) = 0;
	// Runs the baseline solver, setting an initial solution to build off of
	void RunBaseline(PatrollingInput* input, Solution* sol_final, std::vector<std::vector<int>>& drones_to_UGV);

protected:
	/*
	 * Solves TSP on on vertices held in lst and stores found ordering in result. The multiplier
	 * variable can be set to force the solver to solver a fixed-HPP (forcing the first and last
	 * vertices in lst to be connected in the TSP solution).
	 */
	void solverTSP_LKH(std::vector<TSPVertex>& lst, std::vector<TSPVertex>& result, double multiplier);
private:
	KMeansSolver mKMeansSolver;
	VRPSolver mVRPSolver;
};
