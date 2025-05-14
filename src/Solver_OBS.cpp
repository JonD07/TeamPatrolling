#include "Solver_OBS.h"
#include "PatrollingInput.h"
#include "Solution.h"
#include "defines.h"
#include <cmath>
#include <cstdio>
#include <vector>
#include <optional>

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

UGVAction Solver_OBS::fixOverlappingActionOBS(const UGVAction& issueAction, const DroneAction& stepTowardsAction, const std::vector<Obstacle>& input_obstacles) {
	// * We step from the issueAction action to the stepTowardsAction until we are not in any obstacles 
	
	double dx = stepTowardsAction.fX - issueAction.fX;
	double dy = stepTowardsAction.fY - issueAction.fY;
	double distance = std::sqrt(dx * dx + dy * dy);
	if (distance == 0.0) {
		throw std::runtime_error("Cannot fix overlapping action — direction vector has zero length");
	}

	double fixedX = issueAction.fX;
	double fixedY = issueAction.fY;

	// * Create out unit vector 
	double ux = dx / distance;
	double uy = dy / distance;

	// * Stop when rounded coordinates match — "close enough" to consider on top of target
	while (std::round(fixedX) != std::round(stepTowardsAction.fX) || std::round(fixedY) != std::round(stepTowardsAction.fY)) {
		fixedX += OVERLAPPING_STEP_SIZE * ux; 
		fixedY += OVERLAPPING_STEP_SIZE * uy; 
		
		// * Test to see if our new action is inside any obstacles 
		UGVAction tempAction(issueAction.mActionType, fixedX, fixedY, 1111, issueAction.mDetails);
		bool isClear = true;
		for (const Obstacle& obstacle : input_obstacles) {
			if (isActionInsideObstacle(tempAction, obstacle)) {
				isClear = false; // * We need to keep stepping 
				break;
			}
		}
		if (isClear) return tempAction; // * We have stepped out of all obstacles 
	}

	// * Test one last time
	UGVAction tempAction(issueAction.mActionType, fixedX, fixedY, 1111, issueAction.mDetails);
	bool isClear = true;
	for (const Obstacle& obstacle : input_obstacles) {
		if (isActionInsideObstacle(tempAction, obstacle)) {
			isClear = false; // * We need to keep stepping 
			break;
		}
	}
	
	if (isClear) {
		return tempAction; 
	} else {
		std::cerr << "Could not step action out of obstacle -- solution is currently unsolveable" << std::endl;
		std::cerr.flush();
		throw std::runtime_error("Obstacle overlap");
	}
}

void Solver_OBS::pushActionsOutside(int ugv_num, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV) {
	std::vector<UGVAction> ugv_action_list;
	sol_current->GetUGVActionList(ugv_num, ugv_action_list);
    std::vector<Obstacle> input_obstacles = input->GetObstacles(); 
	
	// * create a temp action list that is a copy 
	std::vector<UGVAction> temp_ugv_action_list;
	sol_current->GetUGVActionList(ugv_num, temp_ugv_action_list);
	std::vector<int> drone_IDs = drones_to_UGV[ugv_num];

	// * Need to create a list mapping 
	std::map<int, std::vector<DroneAction>> ugv_drone_action_lists;

	for (int i = 0; i < 2; ++i) {
		if (i < drone_IDs.size()) {
			int droneId = drone_IDs[i];
			std::vector<DroneAction> temp_action_list;
			sol_current->GetDroneActionList(droneId, temp_action_list);
			ugv_drone_action_lists[droneId] = temp_action_list;
		} else {
			ugv_drone_action_lists[-1] = {}; // empty vector = missing slot
		}
	}

	// * First we need to check if action exist on top of obstacles, if so they need to be pushed out
	for(size_t UGV_action_index = 0; UGV_action_index < ugv_action_list.size(); UGV_action_index++) {
            UGVAction curr_action = ugv_action_list[UGV_action_index];

            for (const Obstacle& obstacle : input_obstacles) {
                if (isActionInsideObstacle(curr_action, obstacle)) {
						if (DEBUG_OBS) {
							printf("We have found a obstacle overlaping with a action\n");
							printf("Action:\n");
							curr_action.print();
							printf("Obstacle:\n");
							obstacle.printInfo(); 
							printf("\n");
						}
						switch (curr_action.mActionType) {
							// * We do nothing, since we want to move the launch/land and will just recreate this action later 
							case E_UGVActionTypes::e_MoveToWaypoint:
								break; 
							// * For both of these we will need to move this action out of the obstacle and then create a new corresponding move to waypoint
							case E_UGVActionTypes::e_LaunchDrone: {
								// * We need to determine the drone action we want to move towards 
								std::vector<DroneAction>& drone_actions = ugv_drone_action_lists.at(curr_action.mDetails);

								DroneAction* moveTowardsAction = nullptr;
								int swap_index = -1;
								for (int i = 0; i < drone_actions.size(); i++) {
									DroneAction& d_a = drone_actions[i]; 
									if (d_a.mActionType == E_DroneActionTypes::e_LaunchFromUGV &&
										d_a.fCompletionTime == curr_action.fCompletionTime) 
									{
										// * The action after the corresponding launch is the first move to waypoint
										// * This is the action we want to push towards
										moveTowardsAction = &drone_actions[i+1];
										
										// * Sanity Check
										if (moveTowardsAction->mActionType != E_DroneActionTypes::e_MoveToNode) {
											throw std::runtime_error("Action list is malformed, we are assuming to be moving toward a #2 action");
										}
										swap_index = i; 

										break;
									}
								}

								if (moveTowardsAction == nullptr) {
									throw std::runtime_error("Matching DroneAction not found for UGV LaunchDrone action.");
								}

								if (DEBUG_OBS) {
									printf("Pushing the action toward this action:\n");
									moveTowardsAction->print(); 
									printf("\n");
								}
								
								UGVAction fixed_action = fixOverlappingActionOBS(curr_action, *moveTowardsAction, input_obstacles); 


								if (DEBUG_OBS) {
									printf("Here is our pushed action: \n");
									fixed_action.print();
									printf("\n");
								}
								// * Now we create our new actions 
								DroneAction new_DLaunch(E_DroneActionTypes::e_LaunchFromUGV, fixed_action.fX, fixed_action.fY, 1111.0, moveTowardsAction->mDetails);
								int uav_num = curr_action.mDetails;
								if (ugv_drone_action_lists.find(uav_num) != ugv_drone_action_lists.end()) { // * Double check to make sure we have a drone action list for the UAV #
									ugv_drone_action_lists[uav_num][swap_index] = new_DLaunch;

								}
								else {
									// * Our lists doesn't exist in the mapping but it should so throw a error 
									throw std::runtime_error("Drone list mapping error");
								}
								

								UGVAction newUGVMove(E_UGVActionTypes::e_MoveToWaypoint, fixed_action.fX, fixed_action.fY, 1111.0, -1);
								temp_ugv_action_list[UGV_action_index - 1] = newUGVMove;
								temp_ugv_action_list[UGV_action_index] = fixed_action; 
							
								}
								break; 
							case E_UGVActionTypes::e_ReceiveDrone: {
								// * We need to determine the drone action we want to move towards 
								std::vector<DroneAction>& drone_actions = ugv_drone_action_lists.at(curr_action.mDetails);

								DroneAction* moveTowardsAction = nullptr;
								int swap_index = -1;
								for (int i = 0; i < drone_actions.size(); i++) {
									DroneAction& d_a = drone_actions[i]; 
									if (d_a.mActionType == E_DroneActionTypes::e_LandOnUGV &&
										d_a.fCompletionTime == curr_action.fCompletionTime) 
									{
										// * The action 2 actions before the Land is the last waypoint
										// * This is the action we want to push towards
										moveTowardsAction = &drone_actions[i-2];

										// * Sanity Check
										if (moveTowardsAction->mActionType != E_DroneActionTypes::e_MoveToNode) {
											throw std::runtime_error("Action list is malformed, we are assuming to be moving toward a #2 action");
										}
										swap_index = i; 

										break;
									}
								}

								if (moveTowardsAction == nullptr) {
									throw std::runtime_error("Matching DroneAction not found for UGV LaunchDrone action.");
								}

								if (DEBUG_OBS) {
									printf("Pushing the action toward this action:\n");
									moveTowardsAction->print(); 
									printf("\n");
								}
								
								UGVAction fixed_action = fixOverlappingActionOBS(curr_action, *moveTowardsAction, input_obstacles); 


								if (DEBUG_OBS) {
									printf("Here is our pushed action: \n");
									fixed_action.print();
									printf("\n");
								}

								// * Now we create our new actions 
								DroneAction new_DLaunch(E_DroneActionTypes::e_LandOnUGV, fixed_action.fX, fixed_action.fY, 1111.0, moveTowardsAction->mDetails);
								// * Unlike in the launch case, we need to also alter the move to ugv action since its paired at the same location as the land 
								DroneAction new_move_ugv(E_DroneActionTypes::e_MoveToUGV, fixed_action.fX, fixed_action.fY, 1111.0, moveTowardsAction->mDetails); 

								int uav_num = curr_action.mDetails;
								if (ugv_drone_action_lists.find(uav_num) != ugv_drone_action_lists.end()) { // * Double check to make sure we have a drone action list for the UAV #
									ugv_drone_action_lists[uav_num][swap_index - 1] = new_move_ugv;
									ugv_drone_action_lists[uav_num][swap_index] = new_DLaunch;
								}
								else {
									// * Our lists doesn't exist in the mapping but it should so throw a error 
									throw std::runtime_error("Done List Mapping Error");
								}
								

								UGVAction newUGVMove(E_UGVActionTypes::e_MoveToWaypoint, fixed_action.fX, fixed_action.fY, 1111.0, -1);
								temp_ugv_action_list[UGV_action_index - 1] = newUGVMove;
								temp_ugv_action_list[UGV_action_index] = fixed_action; 
							
								}
								break; 
							default:
								break; 
						
						}
                }
            }
		}
	

	// * Swap our temp lists into the solution 
	sol_current->swapUGVActionList(ugv_num, temp_ugv_action_list); 
	for (const auto& pair : ugv_drone_action_lists) {
		int drone_ID = pair.first;
		std::cout << drone_ID << std::endl; 
		const std::vector<DroneAction>& actionList = pair.second;

		if (drone_ID == -1) {
			continue; 
		}
		sol_current->swapDroneActionLists(drone_ID, actionList);

	}

	if (DEBUG_OBS) {
		printf("\n");
		printf("Solution after the actions are pushed out of obstacles\n");
		sol_current->PrintSolution();
		printf("\n");
	}
}

void Solver_OBS::checkForRedundantMoves(int ugv_num, Solution* sol_current, const std::vector<Obstacle>& obstacles) {
	std::vector<UGVAction> list_to_check; 
	sol_current->GetUGVActionList(ugv_num, list_to_check);

	std::vector<UGVAction> filtered_list;

	for (int i = 0; i < list_to_check.size(); ++i) {
		const UGVAction& curr = list_to_check[i];

		// * Check if it's a MoveToPosition and if we can skip it
		if (curr.mActionType == E_UGVActionTypes::e_MoveToPosition &&
			i > 0 && i < static_cast<int>(list_to_check.size()) - 1) 
		{
			const UGVAction& prev = list_to_check[i - 1];
			const UGVAction& next = list_to_check[i + 1];

			bool collision = false;
			for (const Obstacle& obs : obstacles) {
				if (checkForObstacle(prev.fX, prev.fY, next.fX, next.fY, obs)) {
					collision = true;
					break;
				}
			}

			if (!collision) {
				if (DEBUG_SOL) {
					printf("Redundant MoveToPosition removed:\n");
					curr.print();
				}
				continue; //* skip adding to filtered list
			}
		}

		filtered_list.push_back(curr); // * retain non-redundant actions
	}

	sol_current->swapUGVActionList(ugv_num, filtered_list);
}



 
bool Solver_OBS::moveAroundObstacles(int ugv_num, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV) {
	// * The first thing we need to do is push any overlapping actions outside of the obstacles
	pushActionsOutside(ugv_num, input, sol_current, drones_to_UGV);
	
	// * Loop through each UGV actions to check for obstacles, find a path around a obstacle if it exists
	bool moved_around_obstacle = false; 
	std::vector<UGVAction> new_UGV_action_list; 
    std::vector<UGVAction> ugv_action_list;
	sol_current->GetUGVActionList(ugv_num, ugv_action_list);
    std::vector<Obstacle> input_obstacles = input->GetObstacles(); 
    OMPL_RRTSTAR pathSolver; 

	for(size_t UGV_action_index = 0; UGV_action_index < ugv_action_list.size(); UGV_action_index++) {
            UGVAction curr_action = ugv_action_list[UGV_action_index];

            
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
								printf("\n");
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
									new_UGV_action_list.emplace_back(E_UGVActionTypes::e_MoveToPosition, path[i].first, path[i].second, 1111.0, obstacle_id);
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


void Solver_OBS::optimizeWithObstacles(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV) {
	
	//* Run the optimizer once to shake things up 
	optimizer.OptLaunching(ugv_num, drones_on_UGV, input, sol_current);

	sol_current->PrintSolution(); 
	
	 // * while we are finding collisions with obstacles 
	while(moveAroundObstacles(ugv_num, input, sol_current, drones_to_UGV)) {
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
	printf("\n"); 

	
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
		checkForRedundantMoves(ugv_num, sol_final, input->GetObstacles());
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
