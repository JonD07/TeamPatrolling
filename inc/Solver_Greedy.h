/*
 * GreedySolver.h
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
#include "KMeansSolver.h"
#include "VRPSolver.h"

#define DEBUG_GREEDY	DEBUG || 1


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


class GreedySolver : public Solver {
public:
	GreedySolver();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
private:
	KMeansSolver mKMeansSolver;
	VRPSolver mVRPSolver;
};
