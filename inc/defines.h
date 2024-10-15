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

#define DRONE_I			0
#define UAV_V_MAX		12.0
#define UAV_V_MAX_AFIELD	5.0


#define DRONE_PER_UGV	2 //scenario should define all of this


enum {
	e_Algo_COMP = 0,
	e_Algo_GREEDY = 1,
	e_Algo_OPTLAUNCH = 2,
	e_Algo_ILO = 3,
};

