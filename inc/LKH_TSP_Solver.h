/*
 * LKH_TSP_Solver.h
 *
 * Created by:	Peter Hall
 * On: 			10/11/2022
 *
 * Description: Uses the LKH library to solve a TSP instance.
 */

#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <boost/numeric/conversion/cast.hpp>

#include "defines.h"

#define DEBUG_LKH	DEBUG || 0

struct Stop {
	int ID;
	double X,Y,Z;

	Stop(int id, double x, double y, double z) {
		ID = id;
		X = x;
		Y = y;
		Z = z;
	}
	Stop(const Stop& other) {
		ID = other.ID;
		X = other.X;
		Y = other.Y;
		Z = other.Z;
	}
	Stop& operator=(const Stop& other) {
		ID = other.ID;
		X = other.X;
		Y = other.Y;
		Z = other.Z;

	    return *this;
	}
};

class LKH_TSP_Solver {
public:
	LKH_TSP_Solver();
	~LKH_TSP_Solver();
	/*
	 * Runs the LKH heuristic TSP solver on the locations in vLoc and stores the
	 * results in vPath. vPath will be an ordered list of indexes that point to
	 * the locations in vLoc.
	 */
	void Solve_TSP(std::vector<Stop> &vStops, std::vector<int> &vPath);

private:
	// Generates the config files to solve TSP on the locations in vLoc
	void write_LKH_Config(std::vector<Stop> &vLoc);
};

