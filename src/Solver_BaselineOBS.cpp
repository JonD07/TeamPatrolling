#include "Solver_BaselineOBS.h"


Solver_Baseline_OBS::Solver_Baseline_OBS() {
	if(SANITY_PRINT)
		printf("Hello from Baseline Solver!\n");
}


void Solver_Baseline_OBS::Solve(PatrollingInput* input, Solution* sol_final) {
	if(SANITY_PRINT)
		printf("\nStarting Greedy solver\n\n");
	/*
	 * Current Baseline Algorithm:
	 * 1. k = 1
	 * 2. Form k clusters
	 * 3. Solve VRP on centroids of each cluster with depot and m_g vehicles
	 * 4. For each UGV j:
	 * 5.   For each cluster assigned to j
	 * 6.     Solve VRP on cluster with |D_j| vehicles
	 * 7.     If there exist drone-tour A such that dist(A) > d^a_max, increase k and repeat steps 2-7
	 * 8.   Form action lists by determining launch/receive times, charge times for drones/UGV
	 * 9.   If there exists UGV-tour A such that energy(A) > E^g_max, increase k and repeat steps 2-9
	 * 10 Return
	 */

	// Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();

	// Assign drones to UGVs
	std::vector<std::vector<int>> drones_to_UGV;
	input->AssignDronesToUGV(drones_to_UGV);

	// Sanity print
	if(DEBUG_GREEDY) {
		printf("UGVs-to-Drones:\n");
		for(int j_g = 0; j_g < input->GetMg(); j_g++) {
			printf(" UGV %d:\n  ", j_g);
			for(int n : drones_to_UGV.at(j_g)) {
				printf("%d ", n);
			}
			printf("\n");
		}
	}

	// Run the baseline solver
	RunBaseline(input, sol_final, drones_to_UGV, true);

	// Verify that each drone can start with enough energy
	if(DEBUG_GREEDY) {
		printf("\nFinal Solution:\n");
		sol_final->PrintSolution();
		printf("\n");
	}
}
