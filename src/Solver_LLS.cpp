#include "Solver_LLS.h"



Solver_LLS::Solver_LLS() {
	if(SANITY_PRINT)
		printf("Hello from Launch, Land, Switch Optimizer!\n");
}

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

Solver_LLS::~Solver_LLS() { }


bool Solver_LLS::areActionsOverlapping(const UGVAction& action1, const UGVAction& action2) {
    // Compute Euclidean distance between the two actions
    double dx = action1.fX - action2.fX;
    double dy = action1.fY - action2.fY;
    double distance = std::sqrt(dx * dx + dy * dy);

	// Check if the action has effectively no movement
	bool no_movement = (distance < LLS_DISTANCE_TOLERANCE);

    // Check if actions are within distance tolerance AND action2 happens after action1 within time tolerance
    return no_movement;
}

bool Solver_LLS::isMoveADummy(const UGVAction& action, const UGVAction& prev_action) {
	// Compute Euclidean distance between the two actions
	double dx = action.fX - prev_action.fX;
	double dy = action.fY - prev_action.fY;
	double distance = std::sqrt(dx * dx + dy * dy);

	// Check if the action has effectively no movement
	bool no_movement = (distance < LLS_DISTANCE_TOLERANCE);

	// Check if the action has effectively no time duration
	bool no_time = (action.fCompletionTime - prev_action.fCompletionTime) < LLS_TIME_TOLERANCE;
	return no_movement && no_time;
}

bool Solver_LLS::isOverlappingLaunchOrReceive(const UGVAction& action1, const UGVAction& action2) {
	return areActionsOverlapping(action1, action2) &&
			//* ensures that action1 is either a Launch or Land and that action2 is either a Launch or Land 
			(action1.mActionType == E_UGVActionTypes::e_LaunchDrone || action1.mActionType == E_UGVActionTypes::e_ReceiveDrone) &&
			(action2.mActionType == E_UGVActionTypes::e_LaunchDrone || action2.mActionType == E_UGVActionTypes::e_ReceiveDrone);
}

/*
*  This function is attempts to switch corresponding UGV launch and land actions that have been placed on top of each other by the optimizer
*  First the function looks for a dummy waypoint that has a launch and land sandwhiching it 
*  If it finds one of these, it makes a dummy sol, switches the actions and runs the optimizer again and then sees if the PAR is better
*  If its better -> switch the current soln to this temp soln and we restart the whole process from the beg of the while loop since this change could introduce new optimization potential 
*  TODO it is possible that we would need to record the successful swaps we are making if it turns out we there is a lot of repeated useless work happening every time we restart the loop
*/
void Solver_LLS::LLSRelaxAll(int ugv_num, std::vector<std::vector<int>>& drones_to_UGV, PatrollingInput* input, Solution* sol_current) {
	if(SANITY_PRINT) {
		printf("Attempting to perform relaxing swaps!\n");
	}
	optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_current);
	double prev_best_par = sol_current->CalculatePar();

	while(true) {  // * Loop forever until all swaps are exhausted
		bool swapped = false;

		// * Retrieve the updated UGV action list after every iteration
		std::vector<UGVAction> curr_UGV_actions;
		sol_current->GetUGVActionList(ugv_num, curr_UGV_actions);


		// * Search for Launch/Lands next to each other
		for(size_t curr_ugv_index = 0; curr_ugv_index < curr_UGV_actions.size() - 2; curr_ugv_index++) {
			UGVAction& prev_action = curr_UGV_actions[curr_ugv_index];
			UGVAction& curr_action = curr_UGV_actions[curr_ugv_index + 1];  
			UGVAction& next_action = curr_UGV_actions[curr_ugv_index + 2];  

			switch(curr_action.mActionType) {
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
							// * Create a temporary solution
							Solution sol_temp(*sol_current);

							// * Swap the actions and rerun optimizer
							int prev_action_index = curr_ugv_index; 
							int next_action_index = curr_ugv_index + 2; 
							sol_temp.swapUGVActions(ugv_num, prev_action_index, next_action_index); 
							
							// * Did we improve the solution?**
							if(DEBUG_LLS) {
								printf("Prev best par and corresponding solution %f\n", prev_best_par);
								sol_current->PrintSolution();
							}


							optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, &sol_temp);
							double new_par = sol_temp.CalculatePar(); 
							
							if(DEBUG_LLS) {
								printf("New par and temp sol %f\n", new_par);
								sol_temp.PrintSolution();
							}

							if(new_par < prev_best_par) {
								if (DEBUG_LLS) {
									printf("The LLS has made a successful switch\n");
								}

								prev_best_par = new_par;
								swapped = true;  
								*sol_current = sol_temp;
								break;  // * Exit the for-loop to restart while-loop
							}
							else {
								// * No improvement 
								if(DEBUG_LLS) {
									printf("The LLS switch was NOT successful\n");
								}
							}
						}
					}
					break; //* Exits the switch statement
				default:
					break; //* Exits the switch statement
			}

			if(swapped) {
				break;  // * Exit the for-loop to restart while-loop since we have a found a optimization
			}
		}

		if(!swapped) {
			break;  // * If no swaps occurred, we're done optimizing (We have RelaxedAll) and we need to break out of while-loop
		}
	}
}



void Solver_LLS::Solve(PatrollingInput* input, Solution* sol_final) {
	if(SANITY_PRINT)
		printf("\nStarting Greedy solver\n\n");


	// Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();

	// Assign drones to UGVs
	std::vector<std::vector<int>> drones_to_UGV;
	input->AssignDronesToUGV(drones_to_UGV);
	
	// Sanity print
	if(DEBUG_LLS) {
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

	if(DEBUG_LLS) {
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

		if(DEBUG_LLS) {
			printf("**** ILO ****\nCurrent par = %f\n", sol_new.CalculatePar());
		}

		/// For each UGV...
		for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {

			/// For each drone...
			if(DEBUG_LLS) {
				printf(" Update Sub-tours\n");
			}
			for(int drone_j : drones_to_UGV.at(ugv_num)) {
				/// Update-Subtours()
				updateSubtours(drone_j, &sol_new);
			}

			/// optimize-launch-land()
			if(DEBUG_LLS) {
				printf(" Optimizing step\n");
			}
			LLSRelaxAll(ugv_num, drones_to_UGV, input, &sol_new);
		}

		/// Did we improve the solution?
		if(sol_new.CalculatePar() < sol_final->CalculatePar()) {
			if(DEBUG_LLS) {
				printf("  Found better solution!\n");
			}
			/// Update the final solution
			*sol_final = Solution(sol_new);
			/// Run again
			opt_flag = true;
		}

		if(DEBUG_LLS) {
			printf("New Solution, PAR = %f\n", sol_final->CalculatePar());
		}
	/// While we made an improvement
	} while(opt_flag);
    
	if(DEBUG_LLS) {
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
