#include "Solver_OBS.h"
#include "PatrollingInput.h"
#include "Solution.h"
#include "Solver_Baseline.h"
#include "OMPL_RRTSTAR.h"
#include "defines.h"
#include <cstddef>

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
	printf("now printing the solution \n");
	sol_final->PrintSolution(); 


    // * Loop through each UGV actions to check for obstacles, find a path around a obstalce if it exists
    std::vector<UGVAction> ugv_action_list;
    std::vector<Obstacle> input_obstacles = input->GetObstacles(); 
    OMPL_RRTSTAR pathSolver; 

	for(int UGV_ID = 0; UGV_ID < input->GetMg(); UGV_ID++) {
		sol_final->GetUGVActionList(UGV_ID, ugv_action_list);
        std::vector<UGVAction> new_UGV_action_list; 
		for(size_t UGV_action_index = 0; UGV_action_index < ugv_action_list.size(); UGV_action_index++) {
            UGVAction curr_action = ugv_action_list[UGV_action_index];

            // TODO something needs to be done if a obstacle is directly over a obstacle 
            for (Obstacle obstacle : input_obstacles) {
                if (isActionInsideObstacle(ugv_action_list[UGV_action_index], obstacle)) {
                        std::cerr << "Action is directly on top of obstacle this case can't be handled right now\n"; 
                        std::exit(1);
                }
            }
            switch(curr_action.mActionType) {
                case E_UGVActionTypes::e_MoveToDepot: 
				case E_UGVActionTypes::e_MoveToWaypoint:
                    if ( UGV_action_index  == 0) {
                        std::cerr << "Action before a move is invalid; Action list is likely malformed\n"; 
                        std::exit(1);
                    }

                    for (Obstacle obstacle : input_obstacles) {
                        UGVAction action1 = ugv_action_list[UGV_action_index - 1];
                        UGVAction action2 = curr_action;
                        bool obstacle_found = checkForObstacle(action1.fX, action1.fY, action2.fX, action2.fY, obstacle);
                        if (obstacle_found) {
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
                                for (int i = 0; i < path.size(); i++) {
                                    if (i == 0) {
                                        // * This item has already been added under the default clause since it technically the previous action 
                                    }
                                    // * Make sure the second path item mathces the second action 
                                    else if (i == path.size() - 1) {
                                        new_UGV_action_list.emplace_back(action2.mActionType, action2.fX, action2.fY, action2.fCompletionTime, action2.mDetails);
                                    }
                                    else {
                                        // TODO fix the completetion times here 
                                        new_UGV_action_list.emplace_back(E_UGVActionTypes::e_MoveToWaypoint, path[i].first, path[i].second, action2.fCompletionTime, 0);
                                    }
                                }
                
                                break;
                            }
                        }
                    }


                    break; 
                default:
                    // * If its not a move we still need to add it to our new list 
                    new_UGV_action_list.emplace_back(curr_action.mActionType, curr_action.fX, curr_action.fY, curr_action.fCompletionTime, curr_action.mDetails);
                    break;
            }
        }   
        sol_final->swapUGVActionList(UGV_ID, new_UGV_action_list); 
        
	}

    printf("New final solution (should remain unchanged if there is either no obstacles or no collisions)\n");
    sol_final->PrintSolution();

}

