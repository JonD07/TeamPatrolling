#include "Solver_ILO.h"


Solver_ILO::Solver_ILO() {
	if(SANITY_PRINT)
		printf("Hello from Iterative Launch Optimizer!\n");
}

Solver_ILO::~Solver_ILO() { }


void Solver_ILO::Solve(PatrollingInput* input, Solution* sol_final) {
	if(SANITY_PRINT)
		printf("\nStarting Greedy solver\n\n");

	/*
	 * ILO Algorithm:
	 *
	 * Find Baseline Solution
	 * For each UGV
	 *   opt-flag := True
	 *   while opt-flag
	 *     optimize-launch-land()
	 *     opt-flag := False
	 *     For each drone
	 *       opt-flag |= Update-Subtours()
	 *     end-for
	 *   end-while
	 * end-for
	 */


	/// S := Find Baseline Solution
	/// Do..
	///   S' := S
	///   For each UGV j...
	///     For each drone k...
	///       S' := Update-Subtours(S', k)
	///     end-for
	///     S' := optimize-launch-land(S', j)
	///   end-for
	///   if par(S') < S
	///     S := S'
	///   end-if
	/// While S changed

	/*
	 * Update-Subtours Algorithm:
	 *
	 * Given set of drone sub-tours T_ot
	 * opt-flag := False
	 * For each sub-tour T in T_ot
	 *   T' := SolveTSP(T)
	 *   if dist(T') < dist(T)
	 *     Update T in T_ot
	 *     opt-flag := True
	 *   end-if
	 * end-for
	 * return opt-flag
	 */

	// Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();

	// Assign drones to UGVs
	std::vector<std::vector<int>> drones_to_UGV;
	input->AssignDronesToUGV(drones_to_UGV);
	
	// Sanity print
	if(DEBUG_ILO) {
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
	// printf("PRINT CLEAN Baseline Solution:\n");
	// sol_final->PrintSolution();
	// bool valid = sol_final->ValidSolution();
	// printf("VALID: %d\n", valid);

	if(DEBUG_ILO) {
		// Record this so we can watch how well the optimizer is improving things
		FILE * pOutputFile;
		pOutputFile = fopen("ilo_improvement.dat", "a");
		fprintf(pOutputFile, "%d %f ", input->GetN(), sol_final->GetTotalTourTime(0));
		fclose(pOutputFile);
	}

	bool opt_flag = true;
	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		// Need to break things up a little before continuing...
		optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final);
	}

	/// Do..
	do {
		// Create a temporary solution
		Solution sol_new(*sol_final);
		opt_flag = false;

		if(DEBUG_ILO) {
			printf("**** ILO ****\nCurrent par = %f\n", sol_new.CalculatePar());
		}

		/// For each UGV...
		for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {

			/// For each drone...
			if(DEBUG_ILO) {
				printf(" Update Sub-tours\n");
			}
			for(int drone_j : drones_to_UGV.at(ugv_num)) {
				/// Update-Subtours()
				updateSubtours(drone_j, &sol_new);
			}

			/// optimize-launch-land()
			if(DEBUG_ILO) {
				printf(" Optimizing step\n");
			}
			optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, &sol_new);
		}

		/// Did we improve the solution?
		if(sol_new.CalculatePar() < sol_final->CalculatePar()) {
			if(DEBUG_ILO) {
				printf("  Found better solution!\n");
			}
			/// Update the final solution
			*sol_final = Solution(sol_new);
			/// Run again
			opt_flag = true;
		}
		if(DEBUG_ILO) {
			printf("New Solution, PAR = %f\n", sol_final->CalculatePar());
		}
	/// While we made an improvement
	} while(opt_flag);

	if(DEBUG_ILO) {
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
