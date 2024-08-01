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
#include "gurobi_c++.h"

#define DEBUG_OPTLAUNCH	DEBUG || 1

#define CONST_RELAXATION(X)		X,X+0.1

enum E_SOCActionType {
	e_LaunchDrone,
	e_ReceiveDrone,
	e_BaseStation,
};

// Data structure for a drone sub-tour
struct SubTour {
	// Fixed distance of inner tour
	double tour_dist;
	int ID;
	int launch_ID;
	int land_ID;
	double start_x;
	double start_y;
	double end_x;
	double end_y;

	// Constructor
	SubTour(double dist, int id, double sx, double sy, double ex, double ey) :
			tour_dist(dist), ID(id), start_x(sx), start_y(sy), end_x(ex), end_y(ey) {
		launch_ID = -1;
		land_ID = -1;
	}
	SubTour(const SubTour& other) : tour_dist(other.tour_dist), ID(other.ID), launch_ID(other.launch_ID), land_ID(other.land_ID) {}
};

// Data structure for actions in the SOC program
struct SOCAction {
	double time; // Time that the action is completed
	int ID; // Which drone?
	E_SOCActionType action_type;
	int subtour_index; // Which sub-tour?

	// Constructor
	SOCAction(double t, int id, E_SOCActionType type, int subtour) : time(t), ID(id), action_type(type), subtour_index(subtour) {}
	SOCAction(const SOCAction& other) : time(other.time), ID(other.ID), action_type(other.action_type), subtour_index(other.subtour_index) {}
};

// Comparison function to SOC Actions by time
struct CompareSOCAction {
	bool operator()(const SOCAction& a1, const SOCAction& a2) {
		return a1.time > a2.time;
	}
};


class OptLaunchSolver : public Solver {
public:
	OptLaunchSolver();
	~OptLaunchSolver();

	void Solve(PatrollingInput* input, Solution* sol_final);

protected:
private:
	KMeansSolver mKMeansSolver;
	VRPSolver mVRPSolver;
};
