/*
 * LaunchOptimizer.h
 *
 * Created by:	Jonathan Diller
 * On: 			Aug 7, 2024
 *
 * Description: This class optimizes the launch and receive points of each drone for a give set
 * of drones and an accompanying UGV. It does this via a second order cone program using Gurobi.
 */

#pragma once

#include <tuple>
#include <queue>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solution.h"
#include "gurobi_c++.h"

#define DEBUG_LAUNCHOPT	DEBUG || 0

#define CONST_RELAXATION(X)		X,X+0.1

enum E_SOCActionType {
	e_LaunchDrone=0,
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
	SubTour(const SubTour& other) {
		tour_dist = other.tour_dist;
		ID = other.ID;
		launch_ID = other.launch_ID;
		land_ID = other.land_ID;
		start_x = other.start_x;
		start_y = other.start_y;
		end_x = other.end_x;
		end_y =other.end_y;
	}


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


class LaunchOptimizer {
public:
	LaunchOptimizer();
	~LaunchOptimizer();

	void OptLaunching(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_final);

protected:
};
