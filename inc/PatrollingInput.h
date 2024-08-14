/*
 * MASInput.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 25, 2024
 *
 * Description: Patrolling problem input class
 *
 */

#pragma once

#include <vector>
#include <iostream>
#include <boost/numeric/conversion/cast.hpp>
// External library for YAML files
#include <yaml-cpp/yaml.h>

#include "Input.h"
#include "Roots.h"


#define DEBUG_PATROLINPUT	DEBUG || 0


// Struct definitions
struct Location {
	double x;
	double y;
};

struct BatteryState {
	double max_battery_energy;
	double current_battery_energy;
};

struct ChargingPad {
	std::string ID;
	std::string mode;
	std::string UAV_ID;
	bool is_charging;
};

struct Agent {
	std::string ID;
	std::string type;
	std::string subtype;
	Location location;
	BatteryState battery_state;
	std::string stratum;
	std::string charging_pad_ID;
	std::vector<ChargingPad> charging_pads;
};

struct Node {
	std::string ID;
	std::string type;
	Location location;
	double time_last_service;
};


class PatrollingInput : public Input {
public:
	/*
	 * Sets:
	 * i in I, set of PoI, |I| = N
	 * j in R^a, set of drones, |R^a| = Ma
	 * j in R^g, set of UGVs, |R^g| = Mg
	 */

	PatrollingInput(std::string input_path);
	virtual ~PatrollingInput();

	/// Getters
	// Number of agents
	int GetN() {return nodes.size();}
	// Number of drones
	int GetMa() {return mRa.size();}
	// Number of UGVs
	int GetMg() {return mRg.size();}
	// Get list of nodes
	const std::vector<Node>& GetNodes() {return nodes;}
	// Get the location of the depot (we assume a single depot)
	void GetDepot(int j, double* x, double* y);
	// Get the initial location of drone j
	void GetDroneInitLocal(int j, double* x, double* y);
	// Get the initial location of UGV j
	void GetUGVInitLocal(int j, double* x, double* y);
	// Get the designated name for node i
	std::string GetNodeID(int i) {return nodes.at(i).ID;}
	// Get the designated name for drone j
	std::string GetDroneID(int j) {return mRa.at(j).ID;}
	// Get the designated name for UGV j
	std::string GetUGVID(int j) {return mRg.at(j).ID;}
	// Get the max battery capacity of drone j
	double GetDroneBatCap(int j) {return mRa.at(j).battery_state.max_battery_energy;}
	// Get the max battery capacity of drone j
	double GetUGVBatCap(int j) {return mRg.at(j).battery_state.max_battery_energy;}
	// Get the max range of drone j (on a full charge)
	double GetDroneMaxDist(int j);
	// Get the max range of drone j (on a full charge)
	double GetUGVMaxDist(int j);
	// Determines the time required to charge drone j for J jules
	double calcChargeTime(int drone_j, double J);
	// Get the maximum time required to fully charge drone j
	double GetTMax(int drone_j);
	// Get the maximum speed of drone j
	double GetDroneVMax(int drone_j);

	// Determines a theoretical upper bound on a possible solution
	double LowerBound();

private:
	std::vector<Agent> mRa;
	std::vector<Agent> mRg;
	std::vector<Node> nodes;
	double depot_x;
	double depot_y;

	void parseAgents(const YAML::Node& agents);
	void parseScenario(const YAML::Node& scenario);
};
