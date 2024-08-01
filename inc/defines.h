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
#define SANITY_PRINT	1

#define EPSILON			0.000001
#define INF				1000000000000
#define PI				3.14159265

#define UAV_MAX_D		11477.0
#define UAV_V_MAX		12.0
#define UAV_LAUNCH_TIME	10.0
#define UAV_LAND_TIME	60.0
#define UGV_V_MAX		4.5
#define UGV_V_CRG		3.0
#define LAUNCH_ENERGY	4000.0
#define LAND_ENERGY		7200.0
#define DRONE_JOULES_PER_SECONDS	376.4
#define DRONE_TOTAL_BAT	360381.0
#define SLOW_CHARGE_POINT	244537.0
#define FAST_CHARGE_A	0.06384091
#define FAST_CHARGE_B	403.886
#define T_MAX	991.4
#define T_STAR	555.6
#define ALPHA	0.003
#define P_STAR	476.407
#define E_STAR	244537
#define UGV_JOULES_PER_SECONDS_DRIVE	1750.7
#define UGV_JOULES_PER_SECONDS_WAIT	200.0
#define UGV_TOTAL_BAT	25010000.0
#define UGV_BAT_SWAP_TIME	300

// Number of simulation trials to run
#define N_S	3


#define DRONE_PER_UGV	2

enum {
	e_Algo_COMP = 0,
	e_Algo_GREEDY = 1,
	e_Algo_OPTLAUNCH = 2,
};
