#include "Solver_OBS.h"
#include "Solution.h"
#include <cstdio>


// TODO Remove when done testing
void runOptimization(); 

Solver_OBS::Solver_OBS() {
    if(SANITY_PRINT)
		printf("Hello from Obstacle Solver!\n");
}

Solver_OBS::~Solver_OBS() {}

bool Solver_OBS::checkForObstacle(double x1, double y1, double x2, double y2, Obstacle obstacle) {
    /*
    * Determines whether the line segment between two points intersects a circular obstacle.
    *
    * The function computes:
    *  - The vector from (x1, y1) to (x2, y2) (the segment being evaluated)
    *  - The projection of the obstacle's center onto that segment
    *  - The closest point on the line to the obstacle
    *
    * It returns true if:
    *  - Either endpoint is inside the obstacle, OR
    *  - The closest point lies within the segment and is within the obstacle's radius
    *
    * Special handling is included for degenerate cases where the segment length is effectively zero.
    */
    double lineVecX = x2 - x1;
    double lineVecY = y2 - y1;
    double dx = obstacle.location.x - x1;
    double dy = obstacle.location.y - y1;
    double lineLength = std::sqrt(lineVecX * lineVecX + lineVecY * lineVecY);

    if (lineLength < std::numeric_limits<double>::epsilon()) {
        // If line length is essentially zero, check direct distance to obstacle
        double directDistance = std::sqrt(dx * dx + dy * dy);
        return directDistance <= obstacle.radius;
    }

    double unitLineVecX = lineVecX / lineLength;
    double unitLineVecY = lineVecY / lineLength;

    double projectionLength = dx * unitLineVecX + dy * unitLineVecY;

    double closestX = x1 + projectionLength * unitLineVecX;
    double closestY = y1 + projectionLength * unitLineVecY;

    double distanceToCenter = std::sqrt(
        (closestX - obstacle.location.x) * (closestX - obstacle.location.x) +
        (closestY - obstacle.location.y) * (closestY - obstacle.location.y)
    );

    bool isClosestPointOnSegment = 
        projectionLength >= 0 && 
        projectionLength <= lineLength;

    bool isEndpoint1InCircle = std::sqrt(
        (x1 - obstacle.location.x) * (x1 - obstacle.location.x) +
        (y1 - obstacle.location.y) * (y1 - obstacle.location.y)
    ) <= obstacle.radius;

    bool isEndpoint2InCircle = std::sqrt(
        (x2 - obstacle.location.x) * (x2 - obstacle.location.x) +
        (y2 - obstacle.location.y) * (y2 - obstacle.location.y)
    ) <= obstacle.radius;

    return isEndpoint1InCircle || isEndpoint2InCircle || 
           ((distanceToCenter <= obstacle.radius) && isClosestPointOnSegment);
}

bool Solver_OBS::isActionInsideObstacle(const UGVAction& action, const Obstacle& obstacle) {
    double dx = action.fX - obstacle.location.x;
    double dy = action.fY - obstacle.location.y;
    double distanceSquared = dx * dx + dy * dy;
    double radiusSquared = obstacle.radius * obstacle.radius;
    return distanceSquared <= radiusSquared;
}


 
bool Solver_OBS::moveAroundObstacles(int ugv_num, PatrollingInput* input, Solution* sol_current) {
	bool moved_around_obstacle = false; 

	// * Loop through each UGV actions to check for obstacles, find a path around a obstacle if it exists
	std::vector<UGVAction> new_UGV_action_list; 
    std::vector<UGVAction> ugv_action_list;
	sol_current->GetUGVActionList(ugv_num, ugv_action_list);
    std::vector<Obstacle> input_obstacles = input->GetObstacles(); 
    OMPL_RRTSTAR pathSolver; 
	for(size_t UGV_action_index = 0; UGV_action_index < ugv_action_list.size(); UGV_action_index++) {
            UGVAction curr_action = ugv_action_list[UGV_action_index];

            // TODO something needs to be done if a obstacle is directly over a obstacle 
            for (Obstacle obstacle : input_obstacles) {
                if (isActionInsideObstacle(ugv_action_list[UGV_action_index], obstacle)) {
                        throw std::runtime_error("Action is directly on top of obstacle — can't be handled right now");

                }
            }


            switch(curr_action.mActionType) {
                case E_UGVActionTypes::e_MoveToDepot: 
				case E_UGVActionTypes::e_MoveToWaypoint:
                    if ( UGV_action_index  == 0) {
                        throw std::runtime_error("Action #1 is a move action, something is malformed with the action list");

                    }

                    for (Obstacle obstacle : input_obstacles) {
                        UGVAction action1 = ugv_action_list[UGV_action_index - 1];
                        UGVAction action2 = curr_action;
                        bool obstacle_found = checkForObstacle(action1.fX, action1.fY, action2.fX, action2.fY, obstacle);
                        if (obstacle_found) {
							// * Set the return variable to true if we have to move around even 1 obstacle 
							moved_around_obstacle = true; 
                            if(DEBUG_OBS) {
                                printf("This obstacle:\n");
                                obstacle.printInfo();
                                printf("Was found between:\n"); 
                                action1.print(); 
                                action2.print(); 
                            }
                            // * Find a path around the obstacle
                            std::vector<std::pair<double, double>> path = 
                                pathSolver.findPathBetweenActions(action1, action2, input_obstacles);
                            
                            if (!path.empty()) {
                                // * Path found successfully!

								if (path.size() < 3) {
									throw std::runtime_error("Path around a obstacle should contain at least 3 points");
								}

								// * Add all middle items in the path to be MoveToPosition Actions
								for (int i = 1; i < path.size() - 1; i++) {
									// * The details holds the ID of the obstacle that this action is moving around, this is important later in the optimizer
									int obstacle_id = obstacle.get_id();
									new_UGV_action_list.emplace_back(E_UGVActionTypes::e_MoveToPosition, path[i].first, path[i].second, action2.fCompletionTime, obstacle_id);
                                }
                            }
                        } 

						
                    }


                    break; 
                default:
					// * Do nothing
					break; 
            }
			// * Make sure to add the action to the action list 
			new_UGV_action_list.emplace_back(curr_action.mActionType, curr_action.fX, curr_action.fY, curr_action.fCompletionTime, curr_action.mDetails);

        } 
	sol_current->swapUGVActionList(ugv_num, new_UGV_action_list); 
	return moved_around_obstacle; 
}


void Solver_OBS::optimizeWithObstacles(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_current) {
	
	//* Run the optimizer once to shake things up 
	optimizer.OptLaunching(ugv_num, drones_on_UGV, input, sol_current);
	
	 // * while we are finding collisions with obstacles 
	while(moveAroundObstacles(ugv_num, input, sol_current)) {
		if (DEBUG_OBS) {
			std::cout << "---------------------------" << std::endl;
			printf("Solution after attemping to move around obstacles\n");
			sol_current->PrintSolution();
			std::cout << "---------------------------" << std::endl;
		}
		optimizer.OptLaunching(ugv_num, drones_on_UGV, input, sol_current);
	} 

}





void Solver_OBS::Solve(PatrollingInput* input, Solution* sol_final) {
    // Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();

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
	printf("solution after baseline \n");
	sol_final->PrintSolution(); 

	// TODO actually not sure about this
	/*
	// Make sure to route around obstacles before we optmize for the first time
	for (int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		moveAroundObstacles(ugv_num, input, sol_final);
	}
	*/
	


	
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
		optimizeWithObstacles(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final);
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
			optimizeWithObstacles(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final); 



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
