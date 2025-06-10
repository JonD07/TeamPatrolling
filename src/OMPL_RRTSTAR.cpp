#include "OMPL_RRTSTAR.h"
#include "defines.h"



namespace ob = ompl::base;
namespace og = ompl::geometric;



OMPL_RRTSTAR::OMPL_RRTSTAR() : bounds(2) {
    bounds.setLow(0, 0.0);
    bounds.setHigh(0, 12.0);
    bounds.setLow(1, 0.0);
    bounds.setHigh(1, 12.0);
    planning_time = OMPL_PLANNING_TIME;
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
			: ob::MotionValidator(si), obstacles_(obstacles), goal_(goal) {}

		bool checkMotion(const ob::State* s1, const ob::State* s2) const override {
			const auto* se2s1 = s1->as<ob::SE2StateSpace::StateType>();
			const auto* se2s2 = s2->as<ob::SE2StateSpace::StateType>();
			double x1 = se2s1->getX(), y1 = se2s1->getY();
			double x2 = se2s2->getX(), y2 = se2s2->getY();

			// Check if this is a motion TO the goal state
			bool isMotionToGoal = (abs(x2 - goal_.fX) < 1e-6 && abs(y2 - goal_.fY) < 1e-6);
			
			double buffer; 
			if (isMotionToGoal) {
				buffer = 0;
			} else {
				buffer = OMPL_OBST_BUFFER_DIST;
			}

			

			for (const auto& obstacle : obstacles_) {
				if (Obstacle::checkForObstacle(x1, y1, x2, y2, obstacle, buffer)) {
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
		const UGVAction& goal_; 
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
    		fprintf(stderr,"[%s][OMPL_RRTSTAR::findPathXY] OMPL did not find an exact solution in the alloted time\n", WARNING);
        }

		ss.simplifySolution();
		const og::PathGeometric& path = ss.getSolutionPath();

		for (size_t i = 0; i < path.getStateCount(); ++i) {
			const auto* state = path.getState(i)->as<ob::SE2StateSpace::StateType>();
			resultPath->push_back(std::make_pair(state->getX(), state->getY()));
		}

		if(resultPath->size() <= 2) {
			fprintf(stderr,"[%s][OMPL_RRTSTAR::findPathXY] OMPL did not find a valid solution in the alloted time\n", ERROR);
			throw std::runtime_error("OMPL: Failed to find valid solution\n");
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
		throw std::runtime_error("OMPL: Not able to pass obstacle\n");
	}

	return !resultPath->empty();
}

