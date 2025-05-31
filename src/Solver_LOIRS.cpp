#include "Solver_LOIRS.h"


Solver_LOIRS::Solver_LOIRS(bool swapping, bool replanning) {
	bSwapping = swapping;
	bReplanning = replanning;

	if(SANITY_PRINT)
		printf("Hello from Launch-Optimizer with Swapping(%d) + Replanning(%d)!\n", bSwapping, bReplanning);
	if(SANITY_PRINT)
		printf("Hello from LOS+IterativeReplanning!\n");
}

Solver_LOIRS::~Solver_LOIRS() { }

void Solver_LOIRS::Solve(PatrollingInput* input, Solution* sol_final) {
	if(SANITY_PRINT)
		printf("\nStarting LLS-OBS solver\n\n");

	// Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();

	// Assign drones to UGVs
	std::vector<std::vector<int>> drones_to_UGV;
	input->AssignDronesToUGV(drones_to_UGV);

	/*
	 * Launch-Optimizer with Swapping (LOS) Algorithm
	 *
	 * Find Baseline Solution
	 * Optimize solution
	 * For each UGV...
	 *   Do
	 *     Perform action swapping
	 *     For each drone
	 *       Update-Subtours()
	 *     end-for
	 *     While UGV tour has collisions
	 *       Run obstacle avoidance
	 *       Optimize solution
	 *     end-while
	 *   While made update to Subtours
	 * end-for
	 */

	// Sanity print
	if(DEBUG_LOS_IR) {
		printf("UGVs-to-Drones:\n");
		for(int j_g = 0; j_g < input->GetMg(); j_g++) {
			printf(" UGV %d:\n  ", j_g);
			for(int n : drones_to_UGV.at(j_g)) {
				printf("%d ", n);
			}
			printf("\n");
		}
	}

	/// Find Baseline Solution
	RunBaseline(input, sol_final, drones_to_UGV);

	if(DEBUG_LOS_IR) {
		printf("**** Found initial solution ****\n Current par = %f\n", sol_final->CalculatePar());
	}

	/// Optimize launch/land actions once
	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		// Need to break things up a little before continuing...
		optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final);
	}

	if(DEBUG_LOS_IR) {
		double old_par = sol_final->CalculatePar();
		printf("**** Optimized initial solution ****\n New par = %f\n", old_par);
	}
	/// For each UGV...
	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		// Stopping condition
		bool made_update = false;

		/// Do
		do {
			made_update = false;

			if(bSwapping) {
				/// Perform action swapping
				swapper.LazySwap(input, sol_final, ugv_num, drones_to_UGV);
			}

			if(bReplanning) {
				// Create a temporary solution
				Solution sol_new(*sol_final);

				/// For each drone
				for(int drone_j : drones_to_UGV.at(ugv_num)) {
					/// Update-Subtours()
					made_update |= updateSubtours(drone_j, &sol_new);
				}

				if(made_update) {
					/// Update the final solution
					*sol_final = Solution(sol_new);
					optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final);
				}
			}

			/// While UGV tour has collisions... run obstacle avoidance
			while(moveAroundObstacles(ugv_num, input, sol_final, drones_to_UGV)) {
				sol_final->GenerateYAML("midsolve_plan.yaml");

				/// Optimize solution
				optimizerOBS.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final);
			}

		} while(made_update);

	}

	if(DEBUG_LOS_IR) {
		printf("\nFinal Solution:\n");
		sol_final->PrintSolution();
		printf("\n");
		// Record this so we can watch how well the optimizer is improving things
		FILE * pOutputFile;
		pOutputFile = fopen("ilo_improvement.dat", "a");
		fprintf(pOutputFile, "%d %f ", input->GetN(), sol_final->GetTotalTourTime(0));
		fclose(pOutputFile);
	}
}
