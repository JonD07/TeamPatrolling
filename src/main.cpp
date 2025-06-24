#include <cstdio>
#include <stdio.h>
#include <vector>
#include <limits>
#include <chrono>
#include <iostream> 

#include "../inc/Solver_LOIRS.h"
#include "defines.h"
#include "PatrollingInput.h"
#include "Solution.h"
#include "Solver.h"
#include "Solver_Baseline.h"
#include "Solver_ILO.h"
#include "Solver_OptLaunch.h"
#include "Solver_Depleted.h"
#include "Solver_LLS.h"
#include "Solver_OBS.h"
#include "Solver_BaselineOBS.h"
#include "Solver_LORS.h"


#define DEBUG_MAIN	DEBUG || 0

#define DEFAULT_PRINT_ACTIONS	false
#define DEFAULT_PRINT_RESULTS	0
#define DATA_LOG_FORMAT	"alg_%d.dat"
#define DEFAULT_DATA_LOG_PATH	""
#define DEFAULT_RUN_NUM	0
#define DEFAULT_VEHICLE_DEFINE_LOCATION "../../VehicleInputs/StandardDefinitionVehicle.yaml"



int main(int argc, char *argv[]) {
	const char* inputPath;
	int algorithm;
	bool print_actions;
	bool printResults;
	const char* outputPath;
	int runnum;
	const char* vehiclePath;


	// Verify user input
	if(argc == 3) {
		inputPath = argv[1];
		algorithm = atoi(argv[2]);
		print_actions = DEFAULT_PRINT_ACTIONS;
		printResults = DEFAULT_PRINT_RESULTS;
		outputPath = DEFAULT_DATA_LOG_PATH;
		runnum = DEFAULT_RUN_NUM;
		vehiclePath = DEFAULT_VEHICLE_DEFINE_LOCATION;
	}
	else if(argc == 4) {
		inputPath = argv[1];
		algorithm = atoi(argv[2]);
		print_actions = atoi(argv[3]);
		printResults = DEFAULT_PRINT_RESULTS;
		outputPath = DEFAULT_DATA_LOG_PATH;
		runnum = DEFAULT_RUN_NUM;
		vehiclePath = DEFAULT_VEHICLE_DEFINE_LOCATION;
	}
	else if(argc == 5) {
		inputPath = argv[1];
		algorithm = atoi(argv[2]);
		print_actions = atoi(argv[3]);
		printResults = atoi(argv[4]);
		outputPath = DEFAULT_DATA_LOG_PATH;
		runnum = DEFAULT_RUN_NUM;
		vehiclePath = DEFAULT_VEHICLE_DEFINE_LOCATION;
	}
	else if(argc == 6) {
		inputPath = argv[1];
		algorithm = atoi(argv[2]);
		print_actions = atoi(argv[3]);
		printResults = atoi(argv[4]);
		outputPath = argv[5];
		runnum = DEFAULT_RUN_NUM;
		vehiclePath = DEFAULT_VEHICLE_DEFINE_LOCATION;
	}
	else if(argc == 7) {
		inputPath = argv[1];
		algorithm = atoi(argv[2]);
		print_actions = atoi(argv[3]);
		printResults = atoi(argv[4]);
		outputPath = argv[5];
		runnum = atoi(argv[6]);
		vehiclePath = DEFAULT_VEHICLE_DEFINE_LOCATION;
	}
	else if(argc == 8) {
		inputPath = argv[1];
		algorithm = atoi(argv[2]);
		print_actions = atoi(argv[3]);
		printResults = atoi(argv[4]);
		outputPath = argv[5];
		runnum = atoi(argv[6]);
		vehiclePath = argv[7];
	}
	else {
		printf("Received %d args, expected 1 or more.\nExpected use:\t./find-assignment <file path> <algorithm> [print actions] [print results] [result output path] [run number]\n\n", argc - 1);
		return 1;
	}

	// Seed random using runnum
	srand(runnum);

	Solver* solver = NULL;
	PatrollingInput input(inputPath,vehiclePath);
	Solution solution(&input);

	// Capture start time
	auto start = std::chrono::high_resolution_clock::now();
	switch(algorithm) {
	case e_Algo_GREEDY: {			// algo: 1
		solver = new BaselineSolver();
	}
	break;

	case e_Algo_OPTLAUNCH: {		// algo: 2
		solver = new OptLaunchSolver();
	}
	break;

	case e_Algo_ILO: {				// algo: 3
		solver = new Solver_ILO();
	}
	break;

	case e_Algo_DEPLETED: {			// algo: 4
		solver = new Solver_Depleted();
	}
	break;

	case e_Algo_LLS: {				// algo: 5
		solver = new Solver_LLS(); 
	}
	break;

	case e_Algo_OBS: {				// algo: 6
		solver = new Solver_OBS(); 
	}
	break; 

	// Evaluated algorithms (all incorporate obstacle avoidance)

	case e_Algo_BASELINE_OBS: {	// alg. 10 Baseline algorithm
		solver = new Solver_Baseline_OBS();
	}
	break;

	case e_Algo_LO: {			// alg. 11 Launch-Optimizer
		solver = new Solver_LORS(false, false);
	}
	break;

	case e_Algo_LOS: {			// alg. 12 Launch-Optimizer with Swapping
		solver = new Solver_LORS(true, false);
	}
	break;

	case e_Algo_LOR: {			// alg. 13 Launch-Optimizer with Replanning
		solver = new Solver_LORS(false, true);
	}
	break;

	case e_Algo_LORS: {			// alg. 14 Launch-Optimizer with Replanning + Swapping
		solver = new Solver_LORS(true, true);
	}
	break;

	case e_Algo_LOIR: {		// alg. 15 Launch-Optimizer with Iterative Replanning
		solver = new Solver_LOIRS(false, true);
	}
	break;

	case e_Algo_LOIS: {		// alg. 16 Launch-Optimizer with Iterative Swapping (just LOS...)
		solver = new Solver_LOIRS(true, false);
	}
	break;

	case e_Algo_LOISR: {		// alg. 17 Launch-Optimizer with Iterative Swapping + Replanning
		solver = new Solver_LOIRS(true, true);
	}
	break;

	case e_Algo_COMP:
	default:
		// No valid algorithm given
		fprintf(stderr, "[%s][main] : \n\tInvalid algorithm identifier!\n", ERROR);
		exit(1);
	}

	double par = INF;
	double worst_latency = INF;
	int valid = 0;

	try {
		// Run the solver
		solver->Solve(&input, &solution);
	}
	catch(const PathPlanningException& e) {
		fprintf(stderr, "[%s][main] Path planning failed: %s\n", ERROR, e.what());
		valid = 1;
	}
	catch(const PathOptimizerException& e) {
		fprintf(stderr, "[%s][main] Exception during optimization: %s", ERROR, e.what());
		valid = 2;
	}
	catch(const std::exception& e) {
		fprintf(stderr, "[%s][main] Unexpected Error: %s", ERROR, e.what());
		valid = 3;
	}

	// Capture end time
	auto stop = std::chrono::high_resolution_clock::now();
	// Determine the time it took to solve this
	long long int duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	double duration_s = (double)duration/1000.0; 

	// If we haven't already flagged this solution...
	if(valid == 0) {
		// Actually calculate PAR
		par = solution.CalculatePar();
		worst_latency = solution.CalculateWorstLatency();
		// Check that the solution is valid again
		if(!solution.is_valid(&input, algorithm)) {
			valid = 4;
		}
	}

	if(valid != 0) {
		fprintf(stderr, "[%s][main] Solution was found to be invalid\n", ERROR);
	}

	if(SANITY_PRINT) {
		printf("PAR: %f, computation time = %f, valid = %d\n", par, duration_s, valid);
	}

	// Print results to file
	if(printResults) {
		FILE * pOutputFile;
		char buff[100];
		sprintf(buff, "%s", outputPath);
		sprintf(buff + strlen(buff), DATA_LOG_FORMAT, algorithm);
		if(SANITY_PRINT)
			printf(" Printing results to: %s\n", buff);
		pOutputFile = fopen(buff, "a");
		// File format: n m_g m_a obst runmun par wl comp-time valid?
		fprintf(pOutputFile, "%d %d %d %ld %d ", input.GetN(), input.GetMa(), input.GetMg(), input.GetObstacles().size(), runnum);
		fprintf(pOutputFile, "%f %f %f", par, worst_latency, duration_s);
		fprintf(pOutputFile, " %d\n", valid);
		fclose(pOutputFile);
	}

	if(print_actions) {
		if(SANITY_PRINT)
			solution.PrintSolution();

		solution.GenerateYAML("output_plan.yaml");
	}

	delete solver;

	if(SANITY_PRINT)
		printf("Done!\n");
}
