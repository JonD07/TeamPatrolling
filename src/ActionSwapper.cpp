#include "ActionSwapper.h"



ActionSwapper::ActionSwapper() {
	if(SANITY_PRINT)
		printf("Hello from Action Swapper!\n");
}

ActionSwapper::~ActionSwapper() { }

void ActionSwapper::LazySwap(PatrollingInput* input, Solution* sol_final, int ugv_num, std::vector<std::vector<int>>& drones_to_UGV) {
	double old_par = sol_final->CalculatePar();

	if(DEBUG_SWAP) {
		printf("Initial par = %f\n", old_par);
	}
	while(true) {
		// Create interim solution
		Solution sol_temp(*sol_final);

		if(DEBUG_SWAP) {
			printf("Run swapping..\n");
		}

		// Swap overlapping actions
		bool good_swap = LazyLLS(ugv_num, drones_to_UGV, input, &sol_temp);
		// Did we successfully swap something?
		if(good_swap) {
			// We found a new value for PAR
			double new_par = sol_temp.CalculatePar();

			if(DEBUG_SWAP) {
				printf("**** Swapped overlap ****\n New par = %f\n", new_par);
			}

			// Did we improve solution quality?
			if((old_par - new_par) > EPSILON) {
				*sol_final = sol_temp;
				old_par = new_par;

				if(DEBUG_SWAP) {
					printf("  Keep solution, run again\n");
				}

				continue;
			}
			else {
				if(DEBUG_SWAP) {
					printf("  Old solution was better\n");
				}

				// Stop running swap logic
				break;
			}
		}
		else {
			if(DEBUG_SWAP)
				printf(" Swap was not successful\n");
			// Stop running swap logic
			break;
		}
	}
}

// Attempts optimizes the solution after every action swap, only keeping a swap if solution quality improves
// TODO it is possible that we would need to record the successful swaps we are making if it turns out we there is a lot of repeated useless work happening every time we restart the loop
void ActionSwapper::LLSRelaxAll(PatrollingInput* input, Solution* sol_current, int ugv_num, std::vector<std::vector<int>>& drones_to_UGV) {
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
						if(DEBUG_SWAP) {
							printf("A dummy MoveToWaypoint has been found\n");
						}
						if(isOverlappingLaunchOrReceive(prev_action, next_action)) {
							if(DEBUG_SWAP) {
								printf("A overlapping launch and/or receive has been found\n");
							}
							// * Create a temporary solution
							Solution sol_temp(*sol_current);

							// * Swap the actions and rerun optimizer
							int prev_action_index = curr_ugv_index;
							int next_action_index = curr_ugv_index + 2;
							sol_temp.swapUGVActions(ugv_num, prev_action_index, next_action_index);

							// * Did we improve the solution?**
							if(DEBUG_SWAP) {
								printf("Prev best par and corresponding solution %f\n", prev_best_par);
							}

							bool good_swap = optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, &sol_temp);
							if(good_swap) {
								double new_par = sol_temp.CalculatePar();

								if(DEBUG_SWAP) {
									printf("New par and temp sol %f\n", new_par);
								}

								if(new_par < prev_best_par) {
									if (DEBUG_SWAP) {
										printf("The LLS has made a successful switch\n");
									}

									prev_best_par = new_par;
									swapped = true;
									*sol_current = sol_temp;
									break;  // * Exit the for-loop to restart while-loop
								}
								else {
									// * No improvement
									if(DEBUG_SWAP) {
										printf("The LLS switch was NOT helpful\n");
									}
								}
							}
							else {
								// Not able to swap things
								if(DEBUG_SWAP) {
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


/*
*  Perform the basic LLS, but don't worry about improvements. Swap all actions that are on top of
*  each other and then optimize. Returns true if we were able to optimize the solution in sol_current.
*/
bool ActionSwapper::LazyLLS(int ugv_num, std::vector<std::vector<int>>& drones_to_UGV, PatrollingInput* input, Solution* sol_current) {
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
					if(DEBUG_SWAP) {
						printf(" Found dummy MoveToWaypoint action: %d (overlaps: %d)\n", curr_action.mActionID, prev_action.mActionID);
					}
					if(isOverlappingLaunchOrReceive(prev_action, next_action)) {
						if(DEBUG_SWAP) {
							printf("  Previous (%d) and next (%d) actions can be swapped\n", prev_action.mActionID, next_action.mActionID);
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
	return optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_current);
}


bool ActionSwapper::areActionsOverlapping(const UGVAction& action1, const UGVAction& action2) {
    // Compute Euclidean distance between the two actions
    double dx = action1.fX - action2.fX;
    double dy = action1.fY - action2.fY;
    double distance = std::sqrt(dx * dx + dy * dy);

	// Check if the action has effectively no movement
	bool no_movement = (distance < LLS_DISTANCE_TOLERANCE);

    // Check if actions are within distance tolerance AND action2 happens after action1 within time tolerance
    return no_movement;
}

bool ActionSwapper::isMoveADummy(const UGVAction& action, const UGVAction& prev_action) {
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

bool ActionSwapper::isOverlappingLaunchOrReceive(const UGVAction& action1, const UGVAction& action2) {
	return areActionsOverlapping(action1, action2) &&
			//* ensures that action1 is either a Launch or Land and that action2 is either a Launch or Land
			(action1.mActionType == E_UGVActionTypes::e_LaunchDrone || action1.mActionType == E_UGVActionTypes::e_ReceiveDrone) &&
			(action2.mActionType == E_UGVActionTypes::e_LaunchDrone || action2.mActionType == E_UGVActionTypes::e_ReceiveDrone);
}
