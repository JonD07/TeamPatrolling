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
#include <cmath>
#include <cstddef>
#include <string>
#include <boost/numeric/conversion/cast.hpp>

#include "Utilities.h"
#include "Solution.h"
#include "gurobi_c++.h"
#include "SOCTypes.h"
#include "PatrollingInput.h"
#include "UAV.h"
#include "UGV.h"
#include "defines.h"
#include "gurobi_c++.h"

#define DEBUG_LAUNCHOPTOBS		DEBUG || 0
#define CONST_RELAXATION(X)		X,X+0.1



class LaunchOptimizerOBS {
public:
	LaunchOptimizerOBS();
	~LaunchOptimizerOBS();

	bool OptLaunching(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_final);
protected:
private:
	// Function to check if a point is inside a polygon using cross product method
	bool static isPointInSquare(double px, double py, double x1, double y1, double x2, double y2,
			double x3, double y3, double x4, double y4);
	// Helper Function to calculate distance from a point to a line segment
	double static distancePointToLineSegment(double px, double py, double x1, double y1, double x2, double y2);
	// Returns true if the square defined by the given 4 points does not intersect with an obstacle
	bool static obstaclesInSquare(const Obstacle& obs_to_ignore, const std::vector<Obstacle>& obstacles,
			double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
	// Used for weird edge case where there is another obstacle overlapping where we want to put a square...
	const Obstacle& getBadObstacle(const Obstacle& obs_to_ignore, const std::vector<Obstacle>& obstacles,
			double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4);
	// Adds a floating box constraint
	void addFloatingBoxConstraints(GRBModel* model, std::vector<std::vector<GRBVar>>* act_pos_var,
			const SOCAction& p2, const Obstacle& obstacle, std::vector<Obstacle>& obstacles);
	/*
	 * Adds Point-to-Point constraints. These are intended for waypoint actions and prevent Gurobi
	 * from pushing action-to-action trajectories into an obstacle.
	 */
	void addP2PConstraints(GRBModel* model, std::vector<std::vector<GRBVar>>* act_pos_var,
			const UGVAction& p1, const UGVAction& p2, const UGVAction& p3, const Obstacle& obstacle);
	// Returns a pointer to the obstacle associated with the mDetails of this SOV action
	const Obstacle* getObstaclePointer(PatrollingInput* input, SOCAction& action);
};
