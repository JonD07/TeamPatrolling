#include "Solver_Baseline.h"


BaselineSolver::BaselineSolver() {
	if(SANITY_PRINT)
		printf("Hello from Baseline Solver!\n");
//	srand(time(NULL));
}


void BaselineSolver::Solve(PatrollingInput* input, Solution* sol_final) {
	if(SANITY_PRINT)
		printf("\nStarting Greedy solver\n\n");

	// Cluster nodes
	// Treat each cluster as a VRP for the drones, where the center of the cluster is the depot (UGV)
	// If any drone path is too long, increase the number of clusters and repeat
	// Else, treat the UGV stops as another VRP
	// Recharge the UGVs between tours, the UAVs between clusters

	/*
	 * Old Algorithm...
	 * 1. k = 1
	 * 2. form k clusters
	 * 3. find VRP solution on each cluster
	 * 4. if there exist sub-tour A such that dist(A) > d^a_max, increase k and repeat steps 2-4
	 * 5. find VRP on centroids of each cluster with depot
	 */

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
	{
		int min = floor(input->GetMa()/input->GetMg());
		int extra = input->GetMa() - (min*input->GetMg());
		int nxt = 0;
		for(int j_g = 0; j_g < input->GetMg(); j_g++) {
			std::vector<int> drones;
			for(int j_a = 0; j_a < min; j_a++) {
				drones.push_back(nxt + j_a);
			}
			if(extra > 0) {
				drones.push_back(nxt + min);
				extra--;
				nxt = nxt + min + 1;
			}
			else {
				nxt = nxt + min;
			}
			drones_to_UGV.push_back(drones);
		}
	}

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
	RunBaseline(input, sol_final, drones_to_UGV);

	// Verify that each drone can start with enough energy
	if(DEBUG_GREEDY) {
		printf("\nFinal Solution:\n");
		sol_final->PrintSolution();
		printf("\n");
	}
}
