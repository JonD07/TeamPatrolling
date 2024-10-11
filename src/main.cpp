#include <stdio.h>
#include <vector>
#include <limits>
#include <chrono>

#include "defines.h"
#include "PatrollingInput.h"
#include "Solution.h"
#include "Solver.h"
#include "Solver_Baseline.h"
#include "Solver_ILO.h"
#include "Solver_OptLaunch.h"


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
	case e_Algo_GREEDY: {		// algo: 1
		solver = new BaselineSolver();
	}
	break;

	case e_Algo_OPTLAUNCH: {	// algo: 2
		solver = new OptLaunchSolver();
	}
	break;

	case e_Algo_ILO: { 			// algo: 3
		solver = new Solver_ILO();
	}
	break;

	case e_Algo_COMP:
	default:
		// No valid algorithm given
		fprintf(stderr, "[ERROR][main] : \n\tInvalid algorithm identifier!\n");
		exit(1);
	}

	// Run the solver
	solver->Solve(&input, &solution);

	// Capture end time
	auto stop = std::chrono::high_resolution_clock::now();
	// Determine the time it took to solve this
	long long int duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	double duration_s = (double)duration/1000.0; //duration in ms
	if(duration_s > 10000){
		double second = duration_s / 1000;
		double minute = second / 60;
		double hour = minute / 60;
		double day = hour / 24;
		duration_s = day;
	}

	double par = solution.CalculatePar();

	if(SANITY_PRINT) {
		printf("PAR: %f, time = %f\n", par, duration_s);
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
		// File format: n m runmun computed_Z estimated_Z comp-time
		fprintf(pOutputFile, "%d %d %d %d ", input.GetN(), input.GetMa(), input.GetMg(), runnum);
		fprintf(pOutputFile, "%f %f", par, duration_s);
		fprintf(pOutputFile, " %d\n", solution.ValidSolution());
		fclose(pOutputFile);
	}

	if(print_actions) {
//		Solution runtime_solution(&input);
//		runtime_solution.CreateRuntimeSolution(solution);
//		runtime_solution.GenerateYAML("output_plan.yaml");

		solution.GenerateYAML("output_plan.yaml");
	}

	delete solver;

	if(SANITY_PRINT)
		printf("Done!\n");
}
