#include "../inc/Solver_LLS_OBS.h"



Solver_LLS_OBS::Solver_LLS_OBS() {
	if(SANITY_PRINT)
		printf("Hello from Launch, Land, Switch Optimizer!\n");
}

Solver_LLS_OBS::~Solver_LLS_OBS() { }

void Solver_LLS_OBS::Solve(PatrollingInput* input, Solution* sol_final) {
	if(SANITY_PRINT)
		printf("\nStarting LLS-OBS solver\n\n");


	// Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();

	// Assign drones to UGVs
	std::vector<std::vector<int>> drones_to_UGV;
	input->AssignDronesToUGV(drones_to_UGV);

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
	
	// Sanity print
	if(DEBUG_LLS_OBS) {
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

	if(DEBUG_LLS_OBS) {
		printf("**** Found initial solution ****\n Current par = %f\n", sol_final->CalculatePar());
	}

	/// Optimize launch/land actions once
	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		// Need to break things up a little before continuing...
		optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final);
	}

	double old_par = sol_final->CalculatePar();

	if(DEBUG_LLS_OBS) {
		printf("**** Optimized initial solution ****\n New par = %f\n", old_par);
	}
	/// Swap overlapping actions
	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
//		this->LLSRelaxAll(ugv_num, drones_to_UGV, input, sol_final);
		while(true) {
			// Create interim solution
			Solution sol_temp(*sol_final);

			// Swap overlapping actions
			LazyLLS(ugv_num, drones_to_UGV, input, &sol_temp);
			double new_par = sol_temp.CalculatePar();

			if(DEBUG_LLS_OBS) {
				printf("**** Swapped overlap ****\n New par = %f\n", new_par);
			}

			// Did we improve solution quality?
			if((old_par - new_par) > EPSILON) {
				*sol_final = sol_temp;
				old_par = new_par;

				if(DEBUG_LLS_OBS) {
					printf("  Keep solution, run again\n");
				}

				continue;
			}
			else {
				if(DEBUG_LLS_OBS) {
					printf("  Old solution was better\n");
				}

				// Stop running swap logic
				break;
			}
		}

		/// Move around obstacles
		moveAroundObstacles(ugv_num, input, sol_final, drones_to_UGV);
	}




//	/// Do.. (iterative part)
//	bool opt_flag = true;
//	do {
//		// Create a temporary solution
//		Solution sol_new(*sol_final);
//		opt_flag = false;
//
//		if(DEBUG_LLS_OBS) {
//			printf("**** LO ****\nCurrent par = %f\n", sol_new.CalculatePar());
//		}
//
//		/// For each UGV...
//		for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
//
//			/// For each drone...
//			if(DEBUG_LLS_OBS) {
//				printf(" Update Sub-tours\n");
//			}
//			for(int drone_j : drones_to_UGV.at(ugv_num)) {
//				/// Update-Subtours()
//				updateSubtours(drone_j, &sol_new);
//			}
//
//			/// optimize-launch-land()
//			if(DEBUG_LLS_OBS) {
//				printf(" Optimizing step\n");
//			}
//			LazyLLS(ugv_num, drones_to_UGV, input, &sol_new);
//		}
//
//		/// Did we improve the solution?
//		if(sol_new.CalculatePar() < sol_final->CalculatePar()) {
//			if(DEBUG_LLS_OBS) {
//				printf("  Found better solution!\n");
//			}
//			/// Update the final solution
//			*sol_final = Solution(sol_new);
//			/// Run again
//			opt_flag = true;
//		}
//
//		if(DEBUG_LLS_OBS) {
//			printf("New Solution, PAR = %f\n", sol_final->CalculatePar());
//		}
//	/// While we made an improvement
//	} while(opt_flag);

    
	if(DEBUG_LLS_OBS) {
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

/*
*  Perform the basic LLS, but don't worry about improvements. Swap all actions that are on top of
*  each other and then optimize. This returns an optimized (consistent) solution.
*/
void Solver_LLS_OBS::LazyLLS(int ugv_num, std::vector<std::vector<int>>& drones_to_UGV, PatrollingInput* input, Solution* sol_current) {
	if(SANITY_PRINT) {
		printf("Attempting to perform relaxing swaps!\n");
	}

	// * Retrieve the updated UGV action list after every iteration
	std::vector<UGVAction> curr_UGV_actions;
	sol_current->GetUGVActionList(ugv_num, curr_UGV_actions);

	// * Search for Launch/Lands next to each other
	for(size_t curr_ugv_index = 0; curr_ugv_index < curr_UGV_actions.size() - 2; curr_ugv_index++) {
		UGVAction& prev_action = curr_UGV_actions[curr_ugv_index];
		UGVAction& curr_action = curr_UGV_actions[curr_ugv_index + 1];
		UGVAction& next_action = curr_UGV_actions[curr_ugv_index + 2];

		switch(curr_action.mActionType)
		{
			case E_UGVActionTypes::e_MoveToWaypoint:
				// * Is the MoveToWaypoint a dummy?
				if(isMoveADummy(curr_action, prev_action)) {
					if(DEBUG_LLS) {
						printf("A dummy MoveToWaypoint has been found\n");
					}
					if(isOverlappingLaunchOrReceive(prev_action, next_action)) {
						if(DEBUG_LLS) {
							printf("A overlapping launch and/or receive has been found\n");
						}

						// * Swap these actions
						int prev_action_index = curr_ugv_index;
						int next_action_index = curr_ugv_index + 2;
						sol_current->swapUGVActions(ugv_num, prev_action_index, next_action_index);
					}
				}

				// Leave switch statement
				break;

			default:

				// Leave switch statement
				break;
		}
	}

	// Run optimizer once
	optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_current);
}
