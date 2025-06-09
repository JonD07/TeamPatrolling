/*
 * Solver.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 25, 2024
 *
 * Description: Parent class for all solvers
 */

#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <cmath>
#include <complex>
#include <queue> 
#include <optional>

#include "Solution.h"
#include "PatrollingInput.h"
#include "Utilities.h"
#include "Roots.h"
#include "KMeansSolver.h"
#include "VRPSolver.h"
#include <boost/numeric/conversion/cast.hpp>
#include "LaunchOptimizerOBS.h"
#include "OMPL_RRTSTAR.h"


#define DEBUG_SOLVER	DEBUG || 0


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

struct GeneralVertex {
	int nID;
	double x;
	double y;
};


class Solver {
public:
	Solver();
	virtual ~Solver();

	virtual void Solve(PatrollingInput* input, Solution* sol_final) = 0;
	// Runs the baseline solver, setting an initial solution to build off of
	void RunBaseline(PatrollingInput* input, Solution* sol_final, std::vector<std::vector<int>>& drones_to_UGV, bool obstacle_avoidance = false);
	void RunDepletedSolver(PatrollingInput* input, Solution* sol_final, std::vector<std::vector<int>>& drones_to_UGV);
protected:
	// Adds (possibly multiple) actions to UGV action queue to move the vehicle to a new point
	double moveUGVtoPoint(PatrollingInput* input, Solution* sol_final, int j_actual, double p_x, double p_y, int subtour, E_UGVActionTypes move_type, bool obstacle_avoidance);
	/*
	 * Solves TSP on on vertices held in lst and stores found ordering in result. The multiplier
	 * variable can be set to force the solver to solver a fixed-HPP (forcing the first and last
	 * vertices in lst to be connected in the TSP solution).
	 */
	void solverTSP_LKH(std::vector<GeneralVertex>& lst, std::vector<GeneralVertex>& result, double multiplier);
	// * Right under optimizeWithObstacles; this is where the actual obstacle avoidance logic is held
	bool moveAroundObstacles(int ugv_num, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV);
	// * Calls moveAroundobstacles and the optimizer in a loop until there are no more conflicts; highest level
	void optimizeWithObstacles(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV);
	// * Helper function to see if after optmizations any of the move actions are no longer needed
	void checkForRedundantMoves(PatrollingInput* input, int ugv_num, Solution* sol_current, const std::vector<Obstacle>& obstacles);
	// * Helper function to determine if a action is inside a obstacle
	bool actionInObstacle(const UGVAction& action, const Obstacle& obstacle, double buffer = OMPL_OBST_BUFFER_DIST);
	// Cycles through the obstacles (circles) and determines if this point is in at least on obstacle
	bool pointInObstacle(double x, double y, const std::vector<Obstacle>& obstacles);
	// Iteratively moves action out of obstacles by pushing away from center of all obstacles that the action falls into
	bool findClosestOutsidePointIterative(double* x, double* y, const std::vector<Obstacle>& obstacles);
	/*
	 * Determine which obstacle's boarder location B is closest to. If no obstacle is within some reasonable distance,
	 * attempt to remove the point and verify that moving from A to C does not lead to a collision. Returns -1 if B
	 * isn't reasonably close to anything and removing it does not lead to a collision.
	 */
	int determineAssociatedObstacle(GeneralVertex A, GeneralVertex B, GeneralVertex C, const std::vector<Obstacle>& obstacles);
	// * Pushing a aciton outside of a obstacle and changes the action lists after the move based on the action type (above fixOverlappingActionOBS)
	bool pushActionsOutside(int ugv_num, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV);
	// * This is the function that does the moving of the action (low level) this performs that actual geometry; This is basically a helper function
	int fixOverlappingActionOBS(UGVAction& issueAction, const DroneAction& stepTowardsAction, const std::vector<Obstacle>& input_obstacles);
	// Attempts to update all sub-tours belong to drone drone_id. Returns true if one of the updates improved solution quality (that is, we changed the ordering of a sub-tour).
	bool updateSubtours(int drone_id, Solution* sol_final);
private:
	double calcUGVMovingEnergy(UGVAction& UGV_last, UGVAction& UGV_current, UGV& UGV_current_object); 
	double calcDroneMovingEnergy(std::vector<DroneAction>& droneActionsSoFar,std::queue<DroneAction>& drone_action_queue, UAV UAV_current_object, double UAV_VMax); 
	// * The drone action queue is purpelsely passed by value since the function needs to iterate through it (by popping) but also needs to keep the queue in tact to build the final action at the end
	void calcDroneWaypointActions(int droneId, int waypointId, std::map< int, std::map<int, std::vector<DroneAction>>>& DroneWaypointActions, std::map< int, std::map<int, std::vector<double>>>& DroneWaypointActionsTimes, std::queue<DroneAction> drone_action_queue, double prevActionTime);
	KMeansSolver mKMeansSolver;
	VRPSolver mVRPSolver;
};
