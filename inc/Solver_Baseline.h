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


class BaselineSolver : public Solver {
public:
	BaselineSolver();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
private:
	KMeansSolver mKMeansSolver;
	VRPSolver mVRPSolver;

	// Forms K clusters
	void formClusters(std::vector<ClusteringPoint>& vctrPoIPoint, std::vector<std::vector<ClusteringPoint>>& clusters, int K);
	// Solve VRP on centroids of each cluster with depot and m_g vehicles
	void solveCentroidsVRP(int K, int m_g, std::vector<std::vector<ClusteringPoint>>& clusters,
			std::vector<VRPPoint>& depots, std::vector<std::vector<int>>& depot_order, double depot_x, double depot_y);
	// Solve VRP on cluster with |D_j| vehicles
	void solveClusterVRP();
};
