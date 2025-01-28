/*
 * Solver_Depleted.h
 *
 * Created by: Jack "Siggy" Sigler
 * On: October 24, 2024
 *
 * Description: This solver extends the baseline solver and implements a new scenario
 * where UGVs continue until their battery is depleted.
 */

#pragma once

#include "Solver.h"

#define DEBUG_DEPLETED DEBUG || 0

class DepletedSolver : public Solver {
public:
    DepletedSolver();  
    ~DepletedSolver();  

    void Solve(PatrollingInput* input, Solution* sol_final) override;
};
