#include "Solver_ILO.h"


Solver_ILO::Solver_ILO() {
	if(SANITY_PRINT)
		printf("Hello from Iterative Launch Optimizer!\n");
}

Solver_ILO::~Solver_ILO() { }


void Solver_ILO::Solve(PatrollingInput* input, Solution* sol_final) {
	if(SANITY_PRINT)
		printf("\nStarting Greedy solver\n\n");

	/*
	 * ILO Algorithm:
	 *
	 * Find Baseline Solution
	 * For each UGV
	 *   opt-flag := True
	 *   while opt-flag
	 *     optimize-launch-land()
	 *     For each drone
	 *       opt-flag |= Update-Subtours()
	 *     end-for
	 *   end-while
	 * end-for
	 */

	/*
	 * Update-Subtours Algorithm:
	 *
	 * Given set of drone sub-tours T_ot
	 * opt-flag := False
	 * For each sub-tour T in T_ot
	 *   T' := SolveTSP(T)
	 *   if dist(T') < dist(T)
	 *     Update T in T_ot
	 *     opt-flag := True
	 *   end-if
	 * end-for
	 * return opt-flag
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
	if(DEBUG_ILO) {
		printf("UGVs-to-Drones:\n");
		for(int j_g = 0; j_g < input->GetMg(); j_g++) {
			printf(" UGV %d:\n  ", j_g);
			for(int n : drones_to_UGV.at(j_g)) {
				printf("%d ", n);
			}
			printf("\n");
		}
	}

	/// Find Baseline Solution
	RunBaseline(input, sol_final, drones_to_UGV);


	if(DEBUG_ILO) {
		printf("\nFinal Solution:\n");
		sol_final->PrintSolution();
		printf("\n");
		// Record this so we can watch how well the optimizer is improving things
		FILE * pOutputFile;
		pOutputFile = fopen("qp_improvement.dat", "a");
		fprintf(pOutputFile, "%d %f ", input->GetN(), sol_final->GetTotalTourTime(0));
		fclose(pOutputFile);
	}


	/// For each UGV...
	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		/// opt-flag := True
		bool opt_flag = true;

		/// while opt-flag
		while(opt_flag) {

		}

		/// optimize-launch-land()
		optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final);

		/// For each drone...
		for(int drone_j : drones_to_UGV.at(ugv_num)) {
			/// opt-flag |= Update-Subtours()
			opt_flag |= updateSubtours(drone_j, sol_final);
		}
	}

	if(DEBUG_ILO) {
		sol_final->PrintSolution();
	}
}


bool Solver_ILO::updateSubtours(int drone_id, Solution* sol_final) {
	/*
	 * Update-Subtours Algorithm:
	 *
	 * Given set of drone sub-tours T_ot
	 * opt-flag := False
	 * For each sub-tour T in T_ot
	 *   T' := SolveTSP(T)
	 *   if dist(T') < dist(T)
	 *     Update T in T_ot
	 *     opt-flag := True
	 *   end-if
	 * end-for
	 * return opt-flag
	 */

	if(DEBUG_LAUNCHOPT)
		printf("Updating sub-tours for drone %d\n", drone_id);

	/// opt-flag := False
	bool opt_flag = true;

	/// For each sub-tour T in T_ot
	// Run through this drones action list...
	std::vector<DroneAction> action_list;
	sol_final->GetDroneActionList(drone_id, action_list);

	/// return opt-flag
	return opt_flag;
}


