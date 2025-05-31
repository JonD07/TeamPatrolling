/*
 * defines.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 25, 2024
 *
 * Description: Global project defines
 */



#pragma once

#define DEBUG			0
#define SANITY_PRINT	0

#define EPSILON			0.000001
#define INF				1000000000000
#define PI				3.14159265
#define ALPHA			0.003 

#define CREATE_SPLINES	0

// Number of simulation trials to run
#define N_S	3

#define DRONE_I				0
#define UAV_V_MAX			12.0
#define UAV_V_MAX_AFIELD	5.0


#define DRONE_PER_UGV	2 //scenario should define all of this

// This is tolerance used to consider if launch and land actions are considered on top of each other
#define LLS_DISTANCE_TOLERANCE	1.0
#define LLS_TIME_TOLERANCE		1.0

// OMPL Constants 
#define OMPL_PLANNING_TIME		0.25	// Adjust time (RRT* and its variants will run until the time-out)
#define OMPL_SUBPROBLEM_PADDING	10000.0
// TODO figure out if I need to implement a buffer dist 
#define OMPL_OBST_BUFFER_DIST	5

#define OVERLAPPING_STEP_SIZE	1

#define OBSTALCE_GURI_CORRIDOR_SIZE 20

// Pushing action points out of obstacles
#define OBS_MOVE_STEP_SIZE		10.0 // (meters)
#define OBS_MOVE_ITERATIONS		100

#define ERROR		"\x1B[31mERROR\x1B[0m"
#define WARNING		"\x1B[33mWARNING\x1B[0m"

enum {
	e_Algo_COMP = 0,
	e_Algo_GREEDY = 1,
	e_Algo_OPTLAUNCH = 2,
	e_Algo_ILO = 3,
	e_Algo_DEPLETED = 4, 
	e_Algo_LLS = 5, 
	e_Algo_OBS = 6, 
	// Evaluated algorithms (all incorporate obstacle avoidance)
	e_Algo_BASELINE_OBS = 10,	// Baseline algorithm
	e_Algo_LO = 11,				// Launch-Optimizer
	e_Algo_LOS = 12,			// Launch-Optimizer with Swapping
	e_Algo_LOR = 13,			// Launch-Optimizer with Replanning
	e_Algo_LORS = 14,			// Launch-Optimizer with Replanning + Swapping
	e_Algo_LOIR = 15,			// Launch-Optimizer with Iterative Replanning
	e_Algo_LOIS = 16,			// Launch-Optimizer with Iterative Swapping (just LOS...)
	e_Algo_LOISR = 17,			// Launch-Optimizer with Iterative Swapping + Replanning
};

