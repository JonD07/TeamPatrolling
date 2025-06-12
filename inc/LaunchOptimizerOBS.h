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

#define CONVEX_SQUARE			1



class LaunchOptimizerOBS {
public:
	LaunchOptimizerOBS();
	~LaunchOptimizerOBS();

	bool OptLaunching(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_final);
    void addCorridorConstraints(GRBModel* model, std::vector<std::vector<GRBVar>>* act_pos_var,
                                const UGVAction& p1, const UGVAction& p2, const UGVAction& p3,
                                const Obstacle& obstacle, std::vector<Obstacle>& obstacles, int rec_count = 0);
protected:
private:
    // Function to check if a point is inside a polygon using cross product method
    bool static isPointInSquare(double px, double py, 
                    double x1, double y1, double x2, double y2, 
                    double x3, double y3, double x4, double y4);
    // Helper Function to calculate distance from a point to a line segment
    double static distancePointToLineSegment(double px, double py, 
                                 double x1, double y1, double x2, double y2);
    // Returns true if the square defined by the given 4 points does not intersect with an obstacle
    bool static checkObstaclesInSquare(const Obstacle& obs_to_ignore, const std::vector<Obstacle>& obstacles,
                           double x1, double y1, double x2, double y2, 
                           double x3, double y3, double x4, double y4);
    // Used for weird edge case where there is another obstacle overlapping where we want to put a square...
    const Obstacle& getBadObstacle(const Obstacle& obs_to_ignore, const std::vector<Obstacle>& obstacles,
                               double x1, double y1, double x2, double y2,
                               double x3, double y3, double x4, double y4);

    // Determine the maximum size of a bounding box
    void findBoxCoords(double& CS_P_1x, double& CS_P_2x, double& CS_P_3x, double& CS_P_4x,
    		double& CS_P_1y, double& CS_P_2y, double& CS_P_3y, double& CS_P_4y, double a_x, double a_y,
    		const Obstacle& obstacle, std::vector<Obstacle>& obstacles);

    // Similar to corridor, but just adds the box. Intended to box in actions that get pushed into obstacles.
    void addBoxConstraints(GRBModel* model, std::vector<std::vector<GRBVar>>* act_pos_var,
    		const SOCAction& p2, const Obstacle& obstacle, std::vector<Obstacle>& obstacles);

    // Builds a box around the action
	void buildBoxOnAction(PatrollingInput* input, GRBModel* model, std::vector<std::vector<GRBVar>>* act_pos_var, SOCAction& action);
};
