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

//#define UAV_MAX_D		11477.
#define UAV_LAUNCH_TIME	10.0 //DEFINED IN UAS TABLE as min
#define UAV_LAND_TIME	60.0 //DEFINED IN UAS TABLE as min
#define UGV_V_MAX		1.5 //Not sure where this is from? Velocity max? why not 4.5mps?
#define UGV_V_CRG		1.5 //Not sure where this is from? 
#define LAUNCH_ENERGY	4000.0 //given/const in the UAS specifications table
#define LAND_ENERGY		7200.0 // given/const
#define DRONE_JOULES_PER_SECONDS	376.4 // per drone at max speed of 12mps
//#define DRONE_TOTAL_BAT	360381.0 // = Emax which is given
#define SLOW_CHARGE_POINT	244537.0 //const = E*
#define FAST_CHARGE_A	0.06384091 //calculated from equations.
#define FAST_CHARGE_B	403.886 //const CALCULATED FROM EQUATIONS ONCE E* GIVEN
#define CHARGE_EFFICIENCY	0.9 // EVERY JOULE TRANSFERED TO UAS INCURS A 10% ADDITIONAL TRANSFER LOSS.
#define CHARGE_STARTUP_T	10.0 //Not sure where this is from? Const?
#define T_MAX	991.4 //calculated 
#define T_STAR	555.6 //calculated 
#define ALPHA	0.003 //const
#define P_STAR	476.407 // CALCULATED
#define E_STAR	244537 //given
#define UGV_JOULES_PER_SECONDS_DRIVE	1750.7 //USING ONLY 3 MP NOT 4.5 MP SO IT CAN ALWAYS CHARGE. Need to be calculated based on UGV vmax
#define UGV_JOULES_PER_SECONDS_WAIT	200.0 //given per UGV
//#define UGV_TOTAL_BAT	25010000.0
#define UGV_BAT_SWAP_TIME	300 //depot specific time to be input
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


#define DRONE_PER_UGV	2 //scenario should define all of this


enum {
	e_Algo_COMP = 0,
	e_Algo_GREEDY = 1,
	e_Algo_OPTLAUNCH = 2,
	e_Algo_ILO = 3,
};

// Defines should keep constants:
// 		EPI, Infin, PI, Alpha
// Vehicle Yaml files need to be read into the input and used to calculate scenario values
// 		LAUNCH_ENERGY, LAND_ENERGY, DRONE_TOTAL_BAT, SLOW_CHARGE_POINT, 
//			CHARGE_EFFICIENCY, UAV_LAUNCH_TIME, UAV_LAND_TIME, UGV_BAT_SWAP_TIME, UAV_V_MAX. E_STAR, UGV_JOULES_PER_SECONDS_WAIT
//			UGV_BAT_SWAP_TIME, UGV_MAX_SPEED
// Calculate and assign remainder of values:
//		FAST_CHARGE_A, FAST_CHARGE_B, DRONE_JOULES_PER_SECONDS, T_MAX, T_STAR, P_STAR
// Unsure Values:
// 		 UGV_V_Max, UGV_V_CRG, CHARGE_STARTUP_T


// vector of struct for vars/calculations for later use 
// functions to give back energy usage etc. on demand
//		changing speed constantly.
// input -> vehicle class/struct

// Make UAV/UGV objects that can provide calculations based on values from Vehicle YAML
// Change main to incorporate vehicle file folder


// 9/6 and 9/9
// 1. Battery state is in agent objects, I am not putting in the UGV and UAV objects. Also, charging pads is defined in agent and not in UGV/UAV
//		I could see a refactor to just eliminate Agents and put everything in UAV/UGVs
// 2. Messy to get to the UAV/UGV objects based on agent ID number. I have to try and get the agent ID, convert that to a subtype string, search the correct map and get the UAV/UGV object,
//		then get the value we want. Seems challenging to refactor everything into UGV/UAV objects alone.
// 3. Using Yaml 1_2_5_1 gives 6641.85 and 6616.85 values for answer. Depends on run. Any idea why?
// 4. LaunchOptimizer, Solution files have spots I am not sure how to test. I refactored, but I am unsure how to make sure that the new drone UGV/UAV code is working properly


// Notes: Put Agent stuff into UAV/UGV
// Continue refactoring all constants
// try to just use the index of the drones in the vectors and not the "ID". Dont use "ID" when placing into map
// UAV type and UGV type classes
// 

//9/18
// Launch Optimizer ~270, I only have ordered_action_list. To get the UAV_LAUNCH_TIME, I need to get a UAV. Based on our discussions
// 		I thought I needed to use indicies to get the drone stats and not through the ID. The ordered_action_list is based on the ID.
//		How do I get the UAV object not based on ID from ordered action list? Did I do it right?
// LaunchOptimiser ~279, how to get UGV ID to get UGV_V_CRG? Same with launch optimizer ~436
// For DRONE_JOULES_PER_SECONDS, is the speed ever being changed or is it always assuming max velocity? I dont see any velocity calculations in the code.
// Is Charge_startup a drone or ugv field?
// Do we need to make the battery modular? T_STAR, T_MAX and such, do we need to make it so we can define battery properties and then calculate
//  	all of these values based on that or can we just use these precalculated values from the document?


// 9/18 All Fields through... besides UGV_V_CRG, FAST_CHARGE_A, FAST_CHARGE_B, CHARGE_STARTUP_T, T_MAX, T_STAR, UGV_JOULES_PER_SECONDS_DRIVE (breaks solution)

// Jonathan Meeting 9/19: speed and total battery capacity need to be changed to be modular. Assume battery attributes are the same as given (placed into UAV/UGV as values)
// 		Launch Optimizer actions overall drones and such, not specific drone. Use the same functional form cubed - constant and define these values in
// 		YAML

//9/19 Solution 458, how to get UGV index?