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


#define DEBUG_OBS DEBUG || 1

class Solver_OBS: public Solver {
public:
    Solver_OBS();  
    ~Solver_OBS();  

    void Solve(PatrollingInput* input, Solution* sol_final) override;
    bool checkForObstacle(UGVAction& action1, UGVAction& action2, Obstacle); 
    bool isActionInsideObstacle(const UGVAction& action, const Obstacle& obstacle);
};