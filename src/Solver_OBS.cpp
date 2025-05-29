#include "Solver_OBS.h"
#include "PatrollingInput.h"
#include "Solution.h"
#include "defines.h"
#include <cmath>
#include <cstdio>
#include <vector>
#include <optional>

Solver_OBS::Solver_OBS() {
    if(SANITY_PRINT)
		printf("Hello from Obstacle Solver!\n");
}

Solver_OBS::~Solver_OBS() {}


void Solver_OBS::Solve(PatrollingInput* input, Solution* sol_final) {
//    // Get the POI Nodes from the input
//	std::vector<Node> vctrPOINodes = input->GetNodes();
//
	// Assign drones to UGVs
	std::vector<std::vector<int>> drones_to_UGV;
	input->AssignDronesToUGV(drones_to_UGV);

	// Sanity print
	if(DEBUG_OBS) {
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

	if(DEBUG_OBS) {
		printf("solution after baseline \n");
		sol_final->PrintSolution();
		printf("\n");
	}
	
	if(DEBUG_OBS) {
		// Record this so we can watch how well the optimizer is improving things
		FILE * pOutputFile;
		pOutputFile = fopen("ilo_improvement.dat", "a");
		fprintf(pOutputFile, "%d %f ", input->GetN(), sol_final->GetTotalTourTime(0));
		fclose(pOutputFile);
	}




	bool opt_flag = true;
	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		// Need to break things up a little before continuing...
		optimizeWithObstacles(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final, drones_to_UGV);
	}

	/// Do..
	do {
		// Create a temporary solution
		Solution sol_new(*sol_final);
		opt_flag = false;

		if(DEBUG_OBS) {
			printf("**** ILO ****\nCurrent par = %f\n", sol_new.CalculatePar());
		}

		/// For each UGV...
		for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {

			/// For each drone...
			if(DEBUG_OBS) {
				printf(" Update Sub-tours\n");
			}
			for(int drone_j : drones_to_UGV.at(ugv_num)) {
				/// Update-Subtours()
				updateSubtours(drone_j, &sol_new);
			}


			/// optimize-launch-land()
			if(DEBUG_OBS) {
				printf(" Optimizing step\n");
			}
			optimizeWithObstacles(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final, drones_to_UGV); 
		}

		/// Did we improve the solution?
		if(sol_new.CalculatePar() < sol_final->CalculatePar()) {
			if(DEBUG_OBS) {
				printf(" Found better solution!\n");
			}
			/// Update the final solution
			*sol_final = Solution(sol_new);
			/// Run again
			opt_flag = true;
		} 
		
		if(DEBUG_OBS) {
			printf("New Solution, PAR = %f\n", sol_final->CalculatePar());
		}
	/// While we made an improvement
	} while(opt_flag);

	// * Sometimes the finished product has "redudant" move to position actions where the obstacle is already being avoided
	// * No need to include these in the final solution
	for (int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		checkForRedundantMoves(input, ugv_num, sol_final, input->GetObstacles());
	}

    
	printf("\nFinal Solution:\n");
	sol_final->PrintSolution();
	printf("\n");
	if(DEBUG_OBS) {
		// Record this so we can watch how well the optimizer is improving things
		FILE * pOutputFile;
		pOutputFile = fopen("ilo_improvement.dat", "a");
		fprintf(pOutputFile, "%d %f ", input->GetN(), sol_final->GetTotalTourTime(0));
		fclose(pOutputFile);
	}
	
}
