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
#include <iomanip>
#include <sstream>

#include "Utilities.h"
#include "PatrollingInput.h"

#define DEBUG_SOL	DEBUG || 0

//#define DRONE_BATTERY_ZERO	-.005 * droneA.battery_state.max_battery_energy
#define DRONE_BATTERY_ZERO	-5000.0


enum class E_DroneActionTypes {
	e_LaunchFromUGV=0,	// 0
	e_LandOnUGV,		// 1
	e_MoveToNode,		// 2
	e_MoveToUGV,		// 3
	e_AtUGV,			// 4
	e_KernelEnd			// 5
};

enum class E_UGVActionTypes {
	e_LaunchDrone=0,	// 0
	e_ReceiveDrone,		// 1
	e_MoveToNode,		// 2
	e_MoveToWaypoint,	// 3
	e_MoveToDepot,		// 4
	e_AtDepot,			// 5
	e_KernelEnd, 		// 6
	e_MoveToPosition 	// 7 
};

inline std::string ugvActionTypeToString(E_UGVActionTypes type) {
	switch (type) {
		case E_UGVActionTypes::e_AtDepot:         return "AtDepot";
		case E_UGVActionTypes::e_MoveToWaypoint:  return "MoveToWaypoint";
		case E_UGVActionTypes::e_LaunchDrone:     return "LaunchDrone";
		case E_UGVActionTypes::e_ReceiveDrone:    return "ReceiveDrone";
		case E_UGVActionTypes::e_KernelEnd:       return "KernelEnd";
		case E_UGVActionTypes::e_MoveToPosition:  return "MoveToPosition";
		default:                                  return "Unknown";
	}
}

inline std::string uavActionTypeToString(E_DroneActionTypes type) {
	switch (type) {
		case E_DroneActionTypes::e_LaunchFromUGV: return "LaunchFromUGV";
		case E_DroneActionTypes::e_LandOnUGV:     return "LandOnUGV";
		case E_DroneActionTypes::e_MoveToNode:    return "MoveToNode";
		case E_DroneActionTypes::e_MoveToUGV:     return "MoveToUGV";
		case E_DroneActionTypes::e_AtUGV:         return "AtUGV";
		case E_DroneActionTypes::e_KernelEnd:     return "KernelEnd";
		default:                                  return "Unknown";
	}
}

struct DroneAction {
	int mActionID;
	E_DroneActionTypes mActionType;
	double fX, fY;
	double fCompletionTime;
	// When applicable, this field holds the node ID or drones
	int mDetails;
	int mObstacle;

	DroneAction(E_DroneActionTypes actionType, double x, double y, double t, int details = -1, int obstacle = -1) {
		// Track how many actions have been created
		static int action_count = 0;

		mActionID = action_count++;
		mActionType = actionType;
		fX = x;
		fY = y;
		fCompletionTime = t;
		mDetails = details;
		mObstacle = obstacle;
	}
	DroneAction(const DroneAction& other) {
		mActionID = other.mActionID;
		mActionType = other.mActionType;
		fX = other.fX;
		fY = other.fY;
		fCompletionTime = other.fCompletionTime;
		mDetails = other.mDetails;
		mObstacle = other.mObstacle;
	}

	DroneAction& operator=(const DroneAction& other) {
		mActionID = other.mActionID;
		mActionType = other.mActionType;
		fX = other.fX;
		fY = other.fY;
		fCompletionTime = other.fCompletionTime;
		mDetails = other.mDetails;
		mObstacle = other.mObstacle;
		return *this;
	}

	void print() const {
		std::cout << "UAVAction #" << mActionID << "\n";
		std::cout << "  Type: " << uavActionTypeToString(mActionType) << "\n";
		std::cout << "  Location: (" << fX << ", " << fY << ")\n";
		std::cout << "  Completion Time: " << fCompletionTime << "\n";
		std::cout << "  Details: " << mDetails << "\n";
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

	UGVAction& operator=(const UGVAction& other) {
		mActionID = other.mActionID;
		fX = other.fX;
		fY = other.fY;
		mActionType = other.mActionType;
		fCompletionTime = other.fCompletionTime;
		mDetails = other.mDetails;
		return *this;
	}

	void print() const {
		std::cout << "UGVAction #" << mActionID << "\n";
		std::cout << "  Type: " << ugvActionTypeToString(mActionType) << "\n";
		std::cout << "  Location: (" << fX << ", " << fY << ")\n";
		std::cout << "  Completion Time: " << fCompletionTime << "\n";
		std::cout << "  Details: " << mDetails << "\n";
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
	Solution(const Solution &other);
	Solution& operator=(const Solution &other);
	virtual ~Solution();

	// Takes in an existing solution and generates a runtime version (clears any existing solution stored here)
	void CreateRuntimeSolution(const Solution &other);

	// Calculates the Penalty Accumulation Rate metric
	double CalculatePar();
	// Calculates the worst-latency metric
	double CalculateWorstLatency();

	// Prints this solution
	void PrintSolution();
	// Prints the current solution into a yaml file (for testing with ARL)
	void GenerateYAML(const std::string& filename);
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
	// Returns the time of the last action of UGV j
	double GetTotalTourTime(int j);
	// Completely clears the current solution (deletes all actions and completion times)
	void ClearSolution();
	// Deletes the current plan (actions and completion times) for drone j
	void ClearDroneSolution(int j);
	// Deletes the current plan (actions and completion times) for UGV j
	void ClearUGVSolution(int j);
	// Helper function to convert DroneActionType enum to string
	std::string droneActionTypeToString(E_DroneActionTypes actionType);
	// Helper function to convert UGVActionType enum to string
	std::string ugvActionTypeToString(E_UGVActionTypes actionType);
	// Function to swap an Entire UGV Action List out
	void swapUGVActionList(int UGVId, std::vector<UGVAction>& newActionList);
	// Function to swap an Entire Drone Action List out
	void swapDroneActionLists(int DroneId, const std::vector<DroneAction>& newActionList); 
	// Function to swap two actions in a particular UGVAction List
	void swapUGVActions(int ugv_num, int index1, int index2); 
	// Checks a solution for anything that would make it invalid, (can't quarentee its valid but can prove its invalid)
	bool is_valid(PatrollingInput* input, int algorithm);
	// Checks if a single action pair intercepts any two obstacles 
	bool collisionsPresent(const UGVAction actionA,const UGVAction actionB, const std::vector<Obstacle>& obstacles);
private:
	// Checks to see if UGV - Drone corresponding action is at the same time and at the same place
	bool syncUGVDroneAction(UGVAction& UGV_action, DroneAction& drone_action);
	// Charges drone A for some time
	void chargeDrone(UAV& drone, int drone_ID, double time_on_UGV, PatrollingInput* input);
	// Equation 8 
	double calcEnergyFromTime(UAV drone, double t);
	// Equation 10
	double calcChargedEnergy(UAV drone, int drone_id, double charge_duration, PatrollingInput* input);
	// Starts from a drone being launched, checks to make sure it has enough battery
	bool validateDroneTrip(UAV& droneA, const std::vector<DroneAction>& action_list, int& list_index);
	// Finds the moving energy from 2 UGV actions
	double calcUGVMovingEnergy(UGVAction& UGV_last, UGVAction& UGV_current,UGV& UGV_current_object); 
	// For a UGV does the time and distance of its move make sense
	bool validateMovementAndTiming(const UGVAction& prev, const UGVAction& next, const UGV& ugv, double overhead_time);
	PatrollingInput* m_input;

	// Lists of actions for each robot
	std::vector<std::vector<DroneAction>> m_Aa;
	std::vector<std::vector<UGVAction>> m_Ag;

	// Reduce precision of floating point number and convert to string
	std::string floatingPointToString(double val);
};
