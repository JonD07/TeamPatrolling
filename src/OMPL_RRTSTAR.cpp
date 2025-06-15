#include "OMPL_RRTSTAR.h"
#include "PatrollingInput.h"
#include "Solution.h"
#include "Solver.h"
#include "defines.h"
#include <vector>



namespace ob = ompl::base;
namespace og = ompl::geometric;



OMPL_RRTSTAR::OMPL_RRTSTAR() : bounds(2) {
    bounds.setLow(0, 0.0);
    bounds.setHigh(0, 12.0);
    bounds.setLow(1, 0.0);
    bounds.setHigh(1, 12.0);
    planning_time = OMPL_PLANNING_TIME;
}


// * Logic to retry OMPL if it orginally fails with the set plannign time
void OMPL_RRTSTAR::retryForExactSolution(og::SimpleSetup& ss, std::shared_ptr<og::InformedRRTstar> planner) {
	fprintf(stderr,"[%s][OMPL_RRTSTAR::findPathXY] OMPL did not find an exact solution. Giving it more time...\n", WARNING);
	
	// Change the retry parameters to attempt to close the gap 
	planner->setGoalBias(0.3); 
	planner->setRange(20.0); // Fine-grained movement around obstacles
	ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
	// Try giving it more time - this continues from where it left off
	ob::PlannerStatus retry_solved = ss.solve(OMPL_PLANNING_TIME);
	
	if (retry_solved && ss.getProblemDefinition()->hasExactSolution()) {
		fprintf(stderr, "\033[32mFound exact solution on retry!\033[0m\n");
		return;
	}
	
	// Try 5 times with fresh restarts
	bool found_exact = false;
	
	for (int attempt = 1; attempt <= 5 && !found_exact; attempt++) {
		fprintf(stderr,"[%s][OMPL_RRTSTAR::findPathXY] Fresh restart attempt %d/5...\n", WARNING, attempt);
		
		// Clear everything and start over
		ss.getPlanner()->clear();
		ss.getProblemDefinition()->clearSolutionPaths();
		
		// Create new planner with normal settings
		auto fresh_planner = std::make_shared<og::InformedRRTstar>(ss.getSpaceInformation());
		fresh_planner->setGoalBias(0.05);  // Normal settings
		fresh_planner->setRange(50.0);     // Normal range
		ss.setPlanner(fresh_planner);
		ss.getSpaceInformation()->setStateValidityCheckingResolution(0.01); // Normal resolution
		
		// Try with normal settings
		ob::PlannerStatus fresh_solved = ss.solve(OMPL_PLANNING_TIME);
		
		if (fresh_solved && ss.getProblemDefinition()->hasExactSolution()) {
			fprintf(stderr, "\033[32mFound exact solution on fresh restart attempt %d!\033[0m\n", attempt);
			found_exact = true;
		} else {
			// Try again with aggressive params for half time
			fresh_planner->setGoalBias(0.35);  // Aggressive
			fresh_planner->setRange(15.0);     // Small steps
			ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005); // Fine resolution
			
			ob::PlannerStatus aggressive_solved = ss.solve(OMPL_PLANNING_TIME / 2);
			
			if (aggressive_solved && ss.getProblemDefinition()->hasExactSolution()) {
				fprintf(stderr, "\033[32mFound exact solution with aggressive params on attempt %d!\033[0m\n", attempt);
				found_exact = true;
			}
		}
	}
	
	if (!found_exact) {
		fprintf(stderr,"[%s][OMPL_RRTSTAR::findPathXY] All 5 attempts failed\n", WARNING);
	}
}


// * Useful debugPrint
void OMPL_RRTSTAR::debugPrint(og::SimpleSetup& ss, const UGVAction& action_goal, const std::vector<Obstacle>& subproblem_obstacles) {
	printf("\n=== FINAL APPROACH DEBUG ===\n");
			
	if (ss.getProblemDefinition()->hasApproximateSolution()) {
		auto approx_solution = ss.getProblemDefinition()->getSolutionPath();
		auto path = std::dynamic_pointer_cast<og::PathGeometric>(approx_solution);
		if (path && path->getStateCount() > 0) {
			
			// Print goal state
			printf("Goal state: (%.10f, %.10f)\n", action_goal.fX, action_goal.fY);
			
			// Print last 3 path points
			printf("\nLast 3 path points:\n");
			size_t start_idx = path->getStateCount() >= 3 ? path->getStateCount() - 3 : 0;
			for (size_t i = start_idx; i < path->getStateCount(); ++i) {
				const auto* state = path->getState(i)->as<ob::SE2StateSpace::StateType>();
				double dist_to_goal = sqrt(pow(state->getX() - action_goal.fX, 2) + pow(state->getY() - action_goal.fY, 2));
				printf("  Point %zu: (%.10f, %.10f) dist to goal: %.6f\n", 
						i, state->getX(), state->getY(), dist_to_goal);
			}
			
			// Print all obstacles within 500 units of goal
			printf("\nObstacles within 500 units of goal:\n");
			int count = 0;
			for (size_t i = 0; i < subproblem_obstacles.size(); ++i) {
				const auto& obs = subproblem_obstacles[i];
				double dist_to_goal = sqrt(pow(obs.location.x - action_goal.fX, 2) + 
											pow(obs.location.y - action_goal.fY, 2));
				if (dist_to_goal <= 500.0) {
					double clearance = dist_to_goal - obs.radius;
					printf("  Obstacle %zu: center (%.6f, %.6f) radius %.6f\n", 
							i, obs.location.x, obs.location.y, obs.radius);
					printf("    Distance to goal: %.6f, Clearance from goal: %.6f\n", 
							dist_to_goal, clearance);
					
					// Check if goal is inside this obstacle
					if (obs.containsPoint(action_goal.fX, action_goal.fY)) {
						printf("    *** GOAL IS INSIDE THIS OBSTACLE ***\n");
					}
					
					// Check clearance from last path point
					if (path->getStateCount() > 0) {
						const auto* last_state = path->getState(path->getStateCount()-1)->as<ob::SE2StateSpace::StateType>();
						double dist_last_to_obs = sqrt(pow(obs.location.x - last_state->getX(), 2) + 
														pow(obs.location.y - last_state->getY(), 2));
						double clearance_last = dist_last_to_obs - obs.radius;
						printf("    Clearance from last path point: %.6f\n", clearance_last);
					}
					count++;
				}
			}
			printf("Total obstacles within 500 units: %d\n", count);
			
			// Test line from last path point to goal
			if (path->getStateCount() > 0) {
				const auto* last_state = path->getState(path->getStateCount()-1)->as<ob::SE2StateSpace::StateType>();
				printf("\nTesting line from last path point to goal:\n");
				printf("Last point: (%.10f, %.10f)\n", last_state->getX(), last_state->getY());
				printf("Goal point: (%.10f, %.10f)\n", action_goal.fX, action_goal.fY);
				
				double line_dist = sqrt(pow(action_goal.fX - last_state->getX(), 2) + 
										pow(action_goal.fY - last_state->getY(), 2));
				printf("Distance: %.6f\n", line_dist);
				
				// Check if any obstacles block this line
				printf("Obstacles blocking final line segment:\n");
				bool line_blocked = false;
				for (size_t i = 0; i < subproblem_obstacles.size(); ++i) {
					const auto& obs = subproblem_obstacles[i];
					if (Obstacle::checkForObstacle(last_state->getX(), last_state->getY(), 
													action_goal.fX, action_goal.fY, obs, 0)) {
						printf("  Blocked by obstacle %zu at (%.6f, %.6f) radius %.6f\n",
								i, obs.location.x, obs.location.y, obs.radius);
						line_blocked = true;
					}
				}
				if (!line_blocked) {
					printf("  *** NO OBSTACLES BLOCK FINAL LINE - OMPL ISSUE ***\n");
				}
			}
		}
	}
	printf("=============================\n");
}


/*
* Dynamically creates a rectangler bounded box around two UGV actions 
* uses padding to ensure that even if the actions are lined up either horizontally or vertically 
* there will be still be enough space for a proper subproblem 
*/
void OMPL_RRTSTAR::setBounds(const UGVAction &action1,const UGVAction &action2) {
    // Get the x and y coordinates from both actions
    double x1 = action1.fX;
    double y1 = action1.fY;
    double x2 = action2.fX;
    double y2 = action2.fY;
    
    // Find the min and max values
    double x_min = std::min(x1, x2);
    double x_max = std::max(x1, x2);
    double y_min = std::min(y1, y2);
    double y_max = std::max(y1, y2);
    
    // Add padding to create space around the points
    double padding = OMPL_SUBPROBLEM_PADDING;
    x_min -= padding;
    x_max += padding;
    y_min -= padding;
    y_max += padding;
    
    // Set the bounds
    bounds.setLow(0, x_min);
    bounds.setHigh(0, x_max);
    bounds.setLow(1, y_min);
    bounds.setHigh(1, y_max);
}

void OMPL_RRTSTAR::get_subproblem_obstacles(const std::vector<Obstacle>* all_obstacles, std::vector<Obstacle>* obstaclesInBounds)  {
    double x_min = bounds.low[0];
    double x_max = bounds.high[0];
    double y_min = bounds.low[1];
    double y_max = bounds.high[1];
    
    for (const auto& obstacle : *all_obstacles) {
        // Find the closest point on the rectangle to the circle center
        double closestX = std::max(x_min, std::min(obstacle.location.x, x_max));
        double closestY = std::max(y_min, std::min(obstacle.location.y, y_max));
        
        // Calculate the distance between the closest point and the circle center
        double distanceX = obstacle.location.x - closestX;
        double distanceY = obstacle.location.y - closestY;
        double distanceSquared = distanceX * distanceX + distanceY * distanceY;
        
        // Check if any part of the circle intersects the rectangle
        if (distanceSquared <= obstacle.radius * obstacle.radius) {
            obstaclesInBounds->push_back(obstacle);
//            if (DEBUG_OMPL) {
//                printf("Found the following obstacles to be relevant to the subproblem: \n");
//                obstacle.printInfo();
//            }
        }
    }
}


bool OMPL_RRTSTAR::isStateValid(const ob::State *state, const std::vector<Obstacle> &obstacles) {
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
    double x = se2state->getX();
    double y = se2state->getY();

    for (const auto &obstacle : obstacles) {
        if (obstacle.containsPoint(x, y)) {
            return false;
        }
    }
    return true;
}

bool OMPL_RRTSTAR::findPathBetweenActions(
		PatrollingInput* input,
		const UGVAction& action1,
		const UGVAction& action2,
		const std::vector<Obstacle>& obstacles,
		std::vector<std::pair<double, double>>* path) {
	if(DEBUG_OMPL) {
		ompl::msg::setLogLevel(ompl::msg::LOG_INFO);  // or LOG_DEBUG if you want even more
	}
	else {
		ompl::msg::setLogLevel(ompl::msg::LOG_ERROR); // only show critical issues
	}
	setBounds(action1, action2);
	// Reducing the number of obstacles doesn't seem to make a noticeable difference
    std::vector<Obstacle> obstaclesInBounds;
    get_subproblem_obstacles(&obstacles, &obstaclesInBounds);

    // Do path planning!
    return findPathXY(input, action1, action2, obstaclesInBounds, path);
}

bool OMPL_RRTSTAR::findPathXY(
		PatrollingInput* input,
	    const UGVAction& action_start, const UGVAction& action_goal,
	    const std::vector<Obstacle>& subproblem_obstacles,
		std::vector<std::pair<double, double>>* resultPath) {
	auto space = std::make_shared<ob::SE2StateSpace>();
	space->setBounds(bounds);

	auto si = std::make_shared<ob::SpaceInformation>(space);

	// State validity checker
	si->setStateValidityChecker([&subproblem_obstacles](const ob::State* state) {
		return isStateValid(state, subproblem_obstacles);
	});

	// Inline MotionValidator subclass
	class LocalMotionValidator : public ob::MotionValidator {
	public:
		LocalMotionValidator(const ob::SpaceInformationPtr& si,
						   const std::vector<Obstacle>& obstacles, const UGVAction& goal)
			: ob::MotionValidator(si), obstacles_(obstacles) {}

		bool checkMotion(const ob::State* s1, const ob::State* s2) const override {
			const auto* se2s1 = s1->as<ob::SE2StateSpace::StateType>();
			const auto* se2s2 = s2->as<ob::SE2StateSpace::StateType>();
			double x1 = se2s1->getX(), y1 = se2s1->getY();
			double x2 = se2s2->getX(), y2 = se2s2->getY();

			for (const auto& obstacle : obstacles_) {
				if (Obstacle::checkForObstacle(x1, y1, x2, y2, obstacle)) {
					return false;
				}
			}
			return true;
		}

		// Required second overload (even if unused by RRT*)
		bool checkMotion(const ob::State* s1, const ob::State* s2,
						std::pair<ob::State*, double>& lastValid) const override {
			return checkMotion(s1, s2); // Simple fallback, no lastValid support
		}

	private:
		const std::vector<Obstacle>& obstacles_;
	};

	si->setMotionValidator(std::make_shared<LocalMotionValidator>(si, subproblem_obstacles, action_goal));

	og::SimpleSetup ss(si);

	ob::ScopedState<> start(space);
	start->as<ob::SE2StateSpace::StateType>()->setXY(action_start.fX, action_start.fY);
	ob::ScopedState<> goal(space);
	goal->as<ob::SE2StateSpace::StateType>()->setXY(action_goal.fX, action_goal.fY);
	ss.setStartAndGoalStates(start, goal);

	auto planner = std::make_shared<og::InformedRRTstar>(ss.getSpaceInformation());
	ss.setPlanner(planner);

	if(DEBUG_OMPL) {
		printf("Attempting to find path from (%.3f, %.3f) to (%.3f, %.3f) with %zu obstacles.\n",
			action_start.fX, action_start.fY,
			action_goal.fX, action_goal.fY,
			subproblem_obstacles.size());
	}

	ob::PlannerStatus solved = ss.solve(OMPL_PLANNING_TIME);

	if (solved) {
		if (DEBUG_OMPL) printf("Found a path\n");

		// Check if solution is exact
		if (!ss.getProblemDefinition()->hasExactSolution()) {
			retryForExactSolution(ss, planner);
		}

		if (!ss.getProblemDefinition()->hasExactSolution()) {
			fprintf(stderr,"[%s][OMPL_RRTSTAR::findPathXY] OMPL did not find an exact solution in the alloted time\n", WARNING);
			
			if (DEBUG_OMPL) {
				debugPrint(ss,action_goal, subproblem_obstacles);
			}
		}
		ss.simplifySolution();
		const og::PathGeometric& path = ss.getSolutionPath();

		for (size_t i = 0; i < path.getStateCount(); ++i) {
			const auto* state = path.getState(i)->as<ob::SE2StateSpace::StateType>();
			resultPath->push_back(std::make_pair(state->getX(), state->getY()));
		}

		if(resultPath->size() <= 2) {
			fprintf(stderr,"[%s][OMPL_RRTSTAR::findPathXY] OMPL did not find a valid solution in the alloted time\n", ERROR);
			throw PathPlanningException("OMPL: Failed to find valid solution\n");
		}

		if (DEBUG_OMPL) {
			printf("The path has %zu points\n", resultPath->size());
			printf("The points are as follows: \n");
			for (size_t i = 0; i < resultPath->size(); ++i) {
				printf("  Point %zu: (x = %.3f, y = %.3f)\n", i, resultPath->at(i).first, resultPath->at(i).second);
			}
		}
	} else if (DEBUG_OMPL) {
		printf("No path found :/ \n");
		std::cerr << "Obstacle is not able to be passed around\n";
		throw PathPlanningException("OMPL: Not able to pass obstacle\n");
	}

	return !resultPath->empty();
}


