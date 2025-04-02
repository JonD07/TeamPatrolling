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


bool Solver_OBS::checkForObstacle(UGVAction& action1, UGVAction& action2, Obstacle obstacle) {
    /*
    * Determines whether the line segment between two UGV actions intersects a circular obstacle.
    *
    * The function computes:
    *  - The vector from action1 to action2 (the segment being evaluated)
    *  - The projection of the obstacle's center onto that segment
    *  - The closest point on the line to the obstacle
    *
    * It returns true if:
    *  - Either endpoint is inside the obstacle, OR
    *  - The closest point lies within the segment and is within the obstacle's radius
    *
    * Special handling is included for degenerate cases where the segment length is effectively zero.
    */
    double lineVecX = action2.fX - action1.fX;
    double lineVecY = action2.fY - action1.fY;
    double dx = obstacle.location.x - action1.fX;
    double dy = obstacle.location.y - action1.fY;
    double lineLength = std::sqrt(lineVecX * lineVecX + lineVecY * lineVecY);

    if (lineLength < std::numeric_limits<double>::epsilon()) {
        // If line length is essentially zero, check direct distance to obstacle
        double directDistance = std::sqrt(dx * dx + dy * dy);
        return directDistance <= obstacle.radius;
    }

    double unitLineVecX = lineVecX / lineLength;
    double unitLineVecY = lineVecY / lineLength;

    double projectionLength = dx * unitLineVecX + dy * unitLineVecY;

    double closestX = action1.fX + projectionLength * unitLineVecX;
    double closestY = action1.fY + projectionLength * unitLineVecY;

    double distanceToCenter = std::sqrt(
        (closestX - obstacle.location.x) * (closestX - obstacle.location.x) +
        (closestY - obstacle.location.y) * (closestY - obstacle.location.y)
    );

    bool isClosestPointOnSegment = 
        projectionLength >= 0 && 
        projectionLength <= lineLength;

    bool isEndpoint1InCircle = std::sqrt(
        (action1.fX - obstacle.location.x) * (action1.fX - obstacle.location.x) +
        (action1.fY - obstacle.location.y) * (action1.fY - obstacle.location.y)
    ) <= obstacle.radius;

    bool isEndpoint2InCircle = std::sqrt(
        (action2.fX - obstacle.location.x) * (action2.fX - obstacle.location.x) +
        (action2.fY - obstacle.location.y) * (action2.fY - obstacle.location.y)
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

	for(int j = 0; j < input->GetMg(); j++) {
		sol_final->GetUGVActionList(j, ugv_action_list);
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
                        UGVAction action2 = ugv_action_list[UGV_action_index];
                        bool obstacle_found = checkForObstacle(action1, action2, obstacle);
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

                                
                                // TODO Convert the path to UGV actions                                
                                // TODO rigure out how to replace actions 
                                // TODO Find out what to do next 
                                break;
                            }
                        }
                    }


                    break; 
                default:
                    break;
            }
        }   
        
	}

}

