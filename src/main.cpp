#include <stdio.h>
#include <vector>
#include <limits>
#include <chrono>

#include "defines.h"
#include "PatrollingInput.h"
#include "Solution.h"
#include "Solver.h"
#include "Solver_Baseline.h"
#include "Solver_OptLaunch.h"


#define DEBUG_MAIN	DEBUG || 0

#define PRINT_RESULTS	0
#define DATA_LOG_FORMAT	"alg_%d.dat"
#define DATA_LOG_DEFLT_PATH	""


int main(int argc, char *argv[]) {
	srand(time(NULL));

	int algorithm = 3;
	bool printResults;
	const char* outputPath;
	int runnum = 0;

	// Verify user input
	if(argc == 3) {
		algorithm = atoi(argv[2]);
		printResults = PRINT_RESULTS;
		outputPath = DATA_LOG_DEFLT_PATH;
	}
	else if(argc == 4) {
		algorithm = atoi(argv[2]);
		printResults = atoi(argv[3]);
		outputPath = DATA_LOG_DEFLT_PATH;
	}
	else if(argc == 5) {
		algorithm = atoi(argv[2]);
		printResults = atoi(argv[3]);
		outputPath = argv[4];
	}
	else if(argc == 6) {
		algorithm = atoi(argv[2]);
		printResults = atoi(argv[3]);
		outputPath = argv[4];
		runnum = atoi(argv[5]);
	}
	else {
		printf("Received %d args, expected 1 or more.\nExpected use:\t./find-assignment <file path> <algorithm> [print results] [output path] [run number]\n\n", argc - 1);
		return 1;
	}

	Solver* solver = NULL;
	PatrollingInput input(argv[1]);
	Solution solution(&input);

	// Capture start time
	auto start = std::chrono::high_resolution_clock::now();

	switch(algorithm) {
	case e_Algo_GREEDY: {
		solver = new BaselineSolver();
	}
	break;

	case e_Algo_OPTLAUNCH: {
		solver = new OptLaunchSolver();
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
	double duration_s = (double)duration/1000.0;

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

	delete solver;

	if(SANITY_PRINT)
		printf("Done!\n");
}
