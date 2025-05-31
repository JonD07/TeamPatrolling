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

#include <cstddef>


#define DEBUG_OBS DEBUG || 1



class Solver_OBS: public Solver {
public:
    Solver_OBS();  
    ~Solver_OBS();  

    void Solve(PatrollingInput* input, Solution* sol_final) override;
private:
};
