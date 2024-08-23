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

//#define UAV_MAX_D		11477.0
#define UAV_LAUNCH_TIME	10.0
#define UAV_LAND_TIME	60.0
#define UGV_V_MAX		1.5
#define UGV_V_CRG		1.5
#define LAUNCH_ENERGY	4000.0
#define LAND_ENERGY		7200.0
#define DRONE_JOULES_PER_SECONDS	376.4
//#define DRONE_TOTAL_BAT	360381.0
#define SLOW_CHARGE_POINT	244537.0
#define FAST_CHARGE_A	0.06384091
#define FAST_CHARGE_B	403.886
#define CHARGE_EFFICIENCY	0.9
#define CHARGE_STARTUP_T	10.0
#define T_MAX	991.4
#define T_STAR	555.6
#define ALPHA	0.003
#define P_STAR	476.407
#define E_STAR	244537
#define UGV_JOULES_PER_SECONDS_DRIVE	1750.7
#define UGV_JOULES_PER_SECONDS_WAIT	200.0
//#define UGV_TOTAL_BAT	25010000.0
#define UGV_BAT_SWAP_TIME	300
#define CREATE_SPLINES	0
#define UGV_SPLINE_SEG_DIST	10.0

// Number of simulation trials to run
#define N_S	3

// TODO: Remove these when you have better data!!
#define UAV_V_OPT		UAV_V_MAX
#define UGV_V_OPT		UGV_V_MAX // NOT TRUE! UGV is most efficient while stationary (or moving slowly)
#define DRONE_I			0
#define UAV_V_MAX		12.0
#define UAV_V_MAX_AFIELD	5.0


#define DRONE_PER_UGV	2


enum {
	e_Algo_COMP = 0,
	e_Algo_GREEDY = 1,
	e_Algo_OPTLAUNCH = 2,
	e_Algo_ILO = 3,
};
