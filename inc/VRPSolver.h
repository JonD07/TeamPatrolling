/*
 * VRPSolver.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 26, 2024
 *
 * Description: Wrapper class for VRP Solvers. As of writing, this class uses the vrp-cli to
 * solve VRP instances. Checkout https://reinterpretcat.github.io/vrp/index.html for more
 * information on how the solver works.
 */


#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>

#include <nlohmann/json.hpp>
#include "ClusteringAlgorithm.h"
#include "LKH_TSP_Solver.h"

#include "defines.h"
#include "Utilities.h"

#define DEBUG_VRP	DEBUG || 0
#define VRP_ALGORITHM  1 //0 for richVRP, 1 for VRPSolver

using json = nlohmann::json;


// VRP Point used as input to VRP solver
struct VRPPoint {
	// x,y coordinate in the original problem input
    double x, y;
    // Node ID (assumed to be the index of the PoI in the input vector)
    int ID;

    VRPPoint(int id, double x, double y) {
    	ID = id;
    	this->x = x;
    	this->y = y;
    }

    VRPPoint(const VRPPoint& other) {
    	ID = other.ID;
    	x = other.x;
    	y = other.y;
    }
};

struct VRPLocation {
	double lat;
	double lng;
};

struct VRPJob {
	std::string id;
	VRPLocation location;
	int duration;
	int demand;
};

struct VRPVehicle {
	std::string id;
	VRPLocation start_location;
	VRPLocation end_location;
	std::string start_time;
	std::string end_time;
	double fixed_cost;
	double distance_cost;
	double time_cost;
	int capacity;
};


class VRPSolver {
public:
	VRPSolver();
	virtual ~VRPSolver();

	// Run VRP solver. Takes in a list of VRPPoints. We assume that the last node in the list is the depot.
	bool SolveVRP(std::vector<VRPPoint>& nodes, int num_vehicles, std::vector<std::vector<int>>& tours);
	bool SolveNonRichVRP(std::vector<VRPPoint>& nodes, int num_vehicles, std::vector<std::vector<int>>& tours);
protected:
private:
	//	Base location (with an alternative)
	//	38.650323, -107.319970
	//	38.786140, -107.337353
	//  38.780365, -107.341204
	double base_lat = 38.780365;
	double base_long = -107.341204;

	void generateInputJson(const std::string& filename, const std::vector<VRPJob>& jobs, const std::vector<VRPVehicle>& vehicles);
	void runVrpCli(const std::string& inputFilename, const std::string& outputFilename);
	void readOutputJson(const std::string& filename, std::vector<std::vector<int>>& tour);
	double getLat(double dy);
	double getLng(double dx);
};
