/*
 * I_solution.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 25, 2024
 *
 * Description: Solution class to hold a solution to the given input.
 */

#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <queue>

#include "PatrollingInput.h"

#define DEBUG_SOL	DEBUG || 0


enum class E_DroneActionTypes {
	e_LaunchFromUGV,	// 0
	e_LandOnUGV,		// 1
	e_MoveToNode,		// 2
	e_MoveToUGV,		// 3
	e_AtUGV,			// 4
	e_KernelEnd			// 5
};

enum class E_UGVActionTypes {
	e_LaunchDrone,		// 0
	e_ReceiveDrone,		// 1
	e_MoveToNode,		// 2
	e_MoveToWaypoint,	// 3
	e_MoveToDepot,		// 4
	e_AtDepot,			// 5
	e_KernelEnd			// 6
};

struct DroneAction {
	int mActionID;
	E_DroneActionTypes mActionType;
	double fX, fY;
	double fCompletionTime;
	// When applicable, this field holds the node ID
	int mDetails;

	DroneAction(E_DroneActionTypes actionType, double x, double y, double t, int details = -1) {
		// Track how many actions have been created
		static int action_count = 0;

		mActionID = action_count++;
		mActionType = actionType;
		fX = x;
		fY = y;
		fCompletionTime = t;
		mDetails = details;
	}

	DroneAction(const DroneAction& other) {
		mActionID = other.mActionID;
		mActionType = other.mActionType;
		fX = other.fX;
		fY = other.fY;
		fCompletionTime = other.fCompletionTime;
		mDetails = other.mDetails;
	}
};

struct UGVAction {
	int mActionID;
	E_UGVActionTypes mActionType;
	double fX, fY;
	double fCompletionTime;
	// When applicable, this field holds the node ID
	int mDetails;

	UGVAction(E_UGVActionTypes actionType, double x, double y, double t, int details = -1) {
		// Track how many actions have been created
		static int action_count = 0;

		mActionID = action_count++;
		mActionType = actionType;
		fX = x;
		fY = y;
		fCompletionTime = t;
		mDetails = details;
	}

	UGVAction(const UGVAction& other) {
		mActionID = other.mActionID;
		fX = other.fX;
		fY = other.fY;
		mActionType = other.mActionType;
		fCompletionTime = other.fCompletionTime;
		mDetails = other.mDetails;
	}
};

// Data structure for when a PoI is visited
struct NodeService {
	double time;
	int ActionID;

	// Constructor
	NodeService(double t, int id) : time(t), ActionID(id) {}
};

// Comparison function to order NodeServices by time
struct CompareNoderService {
	bool operator()(const NodeService& a1, const NodeService& a2) {
		return a1.time > a2.time;
	}
};


class Solution {
public:
	Solution(PatrollingInput* input);
	virtual ~Solution();
	Solution(const Solution &other);

	// Prints this solution
	double CalculatePar();

	// Prints this solution
	void PrintSolution();
	// Calculates the Penalty Accumulation Rate of the current solution stored in this solution object.
	double Benchmark();
	// Determines if this is a valid solution (doesn't break constraints)
	bool ValidSolution();
	// Pushes action onto drone j's action list
	void PushDroneAction(int j, const DroneAction& action);
	// Pushes action onto UGV j's action list
	void PushUGVAction(int j, const UGVAction& action);
	// Get the last action for drone j
	const DroneAction& GetLastDroneAction(int j);
	// Get the last action for UGV j
	const UGVAction& GetLastUGVAction(int j);
	// Get the current action list of drone j
	void GetDroneActionList(int j, std::vector<DroneAction>& lst);
	// Get the current action list of UGV j
	void GetUGVActionList(int j, std::vector<UGVAction>& lst);
	// Completely clears the current solution (deletes all actions and completion times)
	void ClearSolution();
	// Deletes the current plan (actions and completion times) for drone j
	void ClearDroneSolution(int j);
	// Deletes the current plan (actions and completion times) for UGV j
	void ClearUGVSolution(int j);

private:
	PatrollingInput* m_input;

	// Lists of actions for each robot
	std::vector<std::vector<DroneAction>> m_Aa;
	std::vector<std::vector<UGVAction>> m_Ag;
};
