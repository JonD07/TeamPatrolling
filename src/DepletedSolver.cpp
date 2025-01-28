/*
 * Solver_Depleted.h
 *
 * Created by: Jack "Siggy" Sigler
 * On: October 24, 2024
 *
 * Description: This solver extends the baseline solver and implements a new scenario
 * where UGVs continue until their battery is depleted.
 */


#include "DepletedSolver.h"
#include "Solver_Baseline.h"

DepletedSolver::DepletedSolver() {
    // Constructor implementation (if any initialization is required)
    if(SANITY_PRINT)
		printf("Hello from Depleted Solver!\n");
}

DepletedSolver::~DepletedSolver() {
    // Destructor implementation (clean up any resources if necessary)
}

void DepletedSolver::Solve(PatrollingInput* input, Solution* sol_final) {
    // Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();

	// Assign drones to UGVs
	std::vector<std::vector<int>> drones_to_UGV;
	input->AssignDronesToUGV(drones_to_UGV);

	// Sanity print
	if(DEBUG_DEPLETED) {
		printf("UGVs-to-Drones:\n");
		for(int j_g = 0; j_g < input->GetMg(); j_g++) {
			printf(" UGV %d:\n  ", j_g);
			for(int n : drones_to_UGV.at(j_g)) {
				printf("%d ", n);
			}
			printf("\n");
		}
	}

	// Run the baseline solver to get an initial solution
	RunBaseline(input, sol_final, drones_to_UGV);

	printf("now printing the solution \n");
	sol_final->PrintSolution(); 
	RunDepletedSolver(input, sol_final, drones_to_UGV);

}

