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
	printf("PRINT CLEAN Baseline Solution:\n");
	sol_final->PrintSolution();
	bool valid = sol_final->ValidSolution();
	printf("VALID: %d\n", valid);

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

//	/// For each UGV...
//	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
//		if(DEBUG_ILO) {
//			printf("**** ILO for UGV %d ****\nCurrent par = %f\n", ugv_num, sol_final->CalculatePar());
//		}
//
//		/// opt-flag := True
//		bool opt_flag = true;
//		/// while opt-flag
//		while(opt_flag) {
//			opt_flag = false;
//
//			/// optimize-launch-land()
//			if(DEBUG_ILO) {
//				printf(" Optimizing step\n");
//			}
//			optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final);
//
//			/// For each drone...
//			for(int drone_j : drones_to_UGV.at(ugv_num)) {
//				/// opt-flag |= Update-Subtours()
//				opt_flag |= updateSubtours(drone_j, sol_final);
//			}
//		}
//
//	}

	if(DEBUG_ILO) {
		sol_final->PrintSolution();
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


bool Solver_ILO::updateSubtours(int drone_id, Solution* sol_final) {
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

	if(DEBUG_ILO)
		printf(" Updating sub-tours for drone %d\n", drone_id);

	/// opt-flag := False
	bool opt_flag = false;

	// Start a new action list
	std::vector<DroneAction> new_action_list;
	std::vector<DroneAction> sub_tour_action_list;

	// Create a sub-tour vector
	std::vector<TSPVertex> lst;
	double old_subtour_dist = 0.0;
	double prev_x=0.0, prev_y=0.0;
	DroneAction sub_tour_start(E_DroneActionTypes::e_LaunchFromUGV, prev_x, prev_y, 0.0);

	// Run through this drones action list...
	std::vector<DroneAction> old_action_list;
	sol_final->GetDroneActionList(drone_id, old_action_list);

	if(DEBUG_ILO)
		printf(" Running through actions\n");

	for(DroneAction a : old_action_list) {
//		if(DEBUG_ILO)
//			printf("  action-type: %d", int(a.mActionType));
		// Are we starting a new sub-tour?
		if(a.mActionType == E_DroneActionTypes::e_LaunchFromUGV) {
//			if(DEBUG_ILO)
//				printf(" starting new sub-tour");

			// Clear old sub-tour
			lst.clear();
			sub_tour_action_list.clear();
			old_subtour_dist = 0.0;

			// Create a TSP vertex
			TSPVertex depot;
			depot.nID = -1;
			depot.x = a.fX;
			depot.y = a.fY;
			lst.push_back(depot);

			// Record where we started the sub-tour
			sub_tour_start.mActionID = a.mActionID;
			sub_tour_start.mActionType = a.mActionType;
			sub_tour_start.fX = a.fX;
			sub_tour_start.fY = a.fY;
			sub_tour_start.fCompletionTime = a.fCompletionTime;
			sub_tour_start.mDetails = a.mDetails;
			prev_x = a.fX;
			prev_y = a.fY;
		}
		// Did we move to a node?
		else if(a.mActionType == E_DroneActionTypes::e_MoveToNode) {
//			if(DEBUG_ILO)
//				printf(" stopping at node %d", a.mDetails);
			// Add node to current sub-tour
			TSPVertex node;
			node.nID = a.mDetails;
			node.x = a.fX;
			node.y = a.fY;
			lst.push_back(node);
			sub_tour_action_list.push_back(a);

			// Update sub-tour distance
			old_subtour_dist += distAtoB(prev_x, prev_y, a.fX, a.fY);
			prev_x = a.fX;
			prev_y = a.fY;
		}
		// Is this the end of the tour?
		else if(a.mActionType == E_DroneActionTypes::e_MoveToUGV) {
//			if(DEBUG_ILO)
//				printf(" end of sub-tour");
			// Add terminal to current sub-tour
			TSPVertex terminal;
			terminal.nID = -2;
			terminal.x = a.fX;
			terminal.y = a.fY;
			lst.push_back(terminal);

			// Update sub-tour distance
			old_subtour_dist += distAtoB(prev_x, prev_y, a.fX, a.fY);

			// Run TSP solver....
			std::vector<TSPVertex> result;
			solverTSP_LKH(lst, result, 1000);

			// Determine the distance of this new tour
			double new_subtour_dist = 0;
			std::vector<TSPVertex>::iterator nxt = result.begin()++;
			std::vector<TSPVertex>::iterator prev = result.begin();
			while(nxt != result.end()) {
				// Get distance to next stop
				new_subtour_dist += distAtoB(prev->x, prev->y, nxt->x, nxt->y);
				// Update iterators
				prev = nxt;
				nxt++;
			}

			/// Start re-building tour
//			if(DEBUG_ILO)
//				printf(". Old dist = %.1f, new dist = %.1f", old_subtour_dist, new_subtour_dist);
			// Add in original take-off
			new_action_list.push_back(sub_tour_start);

			// Did we find a shorter distance?
			if(new_subtour_dist < (old_subtour_dist-EPSILON)) {
//				if(DEBUG_ILO)
//					printf(". NEW TOUR!");
				// Found better tour
				opt_flag = true;
				// Add in new sub-tour
				for(TSPVertex v : result) {
					if(v.nID >= 0) {
						// Push action for this stop onto the drone
						DroneAction nodeStop(E_DroneActionTypes::e_MoveToNode, v.x, v.y, 0.0, v.nID);
						new_action_list.push_back(nodeStop);
					}
				}
			}
			else {
				// Just put the original tour back
				for(DroneAction node_act : sub_tour_action_list) {
					new_action_list.push_back(node_act);
				}
			}
			// Move to UGV
			new_action_list.push_back(a);
		}
		// Not on a sub-tour...
		else {
			// Just push this action back onto the list
			new_action_list.push_back(a);
		}
//		if(DEBUG_ILO)
//			printf("\n");
	}

	// Did we create a new tour?
	if(opt_flag) {
//		if(DEBUG_ILO)
//			printf("  * Update solution\n");
		// Clear old solution
		sol_final->ClearDroneSolution(drone_id);
		for(DroneAction a : new_action_list) {
			sol_final->PushDroneAction(drone_id, a);
		}
	}

	/// return opt-flag
	return opt_flag;
}
