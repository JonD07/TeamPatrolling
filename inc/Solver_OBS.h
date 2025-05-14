/*
 * Solver_OBS.h
 *
 * Created by: Jack "Siggy" Sigler
 * On: March 26, 2025
 *
 * Description: This solver extends the baseline solver and implements a new scenario
 * where there are obstacles in the UGV must avoid 
 */

#pragma once

#include "Solver.h"
#include "PatrollingInput.h"
#include "Solution.h"
#include "Solver_Baseline.h"
#include "defines.h"
#include "Solver_ILO.h"
#include "LaunchOptimizerOBS.h"
#include "OMPL_RRTSTAR.h"

#include <cstddef>


#define DEBUG_OBS DEBUG || 1



class Solver_OBS: public Solver {
public:
    Solver_OBS();  
    ~Solver_OBS();  

    void Solve(PatrollingInput* input, Solution* sol_final) override;
    // * Helper function that sees if a obstacle is between two points 
    bool static checkForObstacle(double x1, double y1, double x2, double y2, Obstacle obstacle); 
private:
    LaunchOptimizerOBS optimizer;
	bool updateSubtours(int drone_id, Solution* sol_final);
    
    // * Calls moveAroundobstacles and the optimizer in a loop until there are no more conflicts; highest level 
    void optimizeWithObstacles(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV);
    // * Right under optimizeWithObstacles; this is where the actual obstacle avoidance logic is held 
    bool moveAroundObstacles(int ugv_num, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV);
    // * Helper function to see if after optmizations any of the move actions are no longer needed
    void checkForRedundantMoves(int ugv_num, Solution* sol_current, const std::vector<Obstacle>& obstacles); 
    // * Helper function to determine if a action is inside a obstacle 
    bool isActionInsideObstacle(const UGVAction& action, const Obstacle& obstacle); 
    // * Pushing a aciton outside of a obstacle and changes the action lists after the move based on the action type (above fixOverlappingActionOBS)
    void pushActionsOutside(int ugv_num, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV);
    // * This is the function that does the moving of the action (low level) this performs that actual geometry; This is basically a helper function 
    UGVAction fixOverlappingActionOBS(const UGVAction& issueAction, const DroneAction& stepTowardsAction, const std::vector<Obstacle>& input_obstacles);


};