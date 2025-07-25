/*
 * OMPL_RRTSTAR.h
 *
 * Created by:	Jack Sigler
 * On: 			Mar 31, 2025
 *
 * Description: This class is built to take a obstacle subproblem instance, solve it, and return the new path 
 * It uses the OMPL library and the RRT* algo from it 
 */
#pragma once

#include "PatrollingInput.h"
#include "Solution.h"
//#include "Solver_OBS.h"
#include "Solver.h"
#include "PatrollingInput.h"
#include "defines.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
// Planner options..
#include <ompl/geometric/planners/rrt/RRT.h> // Fast
#include <ompl/geometric/planners/rrt/RRTConnect.h> // Wicked fast
#include <ompl/geometric/planners/rrt/RRTstar.h> // Asymptotically optimal
#include <ompl/geometric/planners/rrt/InformedRRTstar.h> // Same guarantees as RRT* but faster

#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include <utility>

#define DEBUG_OMPL	DEBUG || 0


class Circle {
private:
    double radius;
    double x_cord;
    double y_cord;

public:
    Circle(double x, double y, double r = 1.0);
    bool containsPoint(double px, double py) const;
    double getX() const;
    double getY() const;
    double getRadius() const;
};

class OMPL_RRTSTAR {
private:
	ompl::base::RealVectorBounds bounds;
	std::vector<Obstacle> subproblem_obstacles;
	double planning_time;
	static bool isStateValid(const ompl::base::State *state, const std::vector<Obstacle> &obstacles);
	void retryForExactSolution(ompl::geometric::SimpleSetup& ss, std::shared_ptr<ompl::geometric::InformedRRTstar> planner);
	void debugPrint(ompl::geometric::SimpleSetup& ss, const UGVAction& action_goal, const std::vector<Obstacle>& subproblem_obstacles);


public:
	OMPL_RRTSTAR();
	void setBounds(const UGVAction &action1,const UGVAction &action2);
	bool findPathBetweenActions(
			PatrollingInput* input,
			const UGVAction& action1,
			const UGVAction& action2,
			const std::vector<Obstacle>& obstacles,
			std::vector<std::pair<double, double>>* path
	);
	bool findPathXY(
			PatrollingInput* input,
			const UGVAction& action_start, const UGVAction& action_goal,
			const std::vector<Obstacle>& subproblem_obstacles,
			std::vector<std::pair<double, double>>* path
	);
	void get_subproblem_obstacles(const std::vector<Obstacle>* all_obstacles, std::vector<Obstacle>* obstaclesInBounds);


};

