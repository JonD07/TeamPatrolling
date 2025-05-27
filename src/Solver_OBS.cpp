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

//	// * Sometimes the finished product has "redudant" move to position actions where the obstacle is already being avoided
//	// * No need to include these in the final solution
//	for (int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
//		checkForRedundantMoves(input, ugv_num, sol_final, input->GetObstacles());
//	}

    
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


bool Solver_OBS::updateSubtours(int drone_id, Solution* sol_final) {
	if(DEBUG_OBS)
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

	if(DEBUG_OBS)
		printf(" Running through actions\n");

	for(DroneAction a : old_action_list) {
		// Are we starting a new sub-tour?
		if(a.mActionType == E_DroneActionTypes::e_LaunchFromUGV) {

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
			// Add in original take-off
			new_action_list.push_back(sub_tour_start);

			// Did we find a shorter distance?
			if(new_subtour_dist < (old_subtour_dist-EPSILON)) {
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
	}

	// Did we create a new tour?
	if(opt_flag) {
		// Clear old solution
		sol_final->ClearDroneSolution(drone_id);
		for(DroneAction a : new_action_list) {
			sol_final->PushDroneAction(drone_id, a);
		}
	}

	/// return opt-flag
	return opt_flag;
}
