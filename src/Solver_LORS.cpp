#include "Solver_LORS.h"


Solver_LORS::Solver_LORS(bool swapping, bool replanning) {
	bSwapping = swapping;
	bReplanning = replanning;

	if(SANITY_PRINT)
		printf("Hello from Launch-Optimizer with Swapping(%d) + Replanning(%d)!\n", bSwapping, bReplanning);
}

Solver_LORS::~Solver_LORS() { }

void Solver_LORS::Solve(PatrollingInput* input, Solution* sol_final) {
	if(SANITY_PRINT)
		printf("\nStarting LLS-OBS solver\n\n");

	// Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();

	// Assign drones to UGVs
	std::vector<std::vector<int>> drones_to_UGV;
	input->AssignDronesToUGV(drones_to_UGV);

	/*
	 * Launch-Optimizer with Swapping (LOS) Algorithm
	 *
	 * Find Baseline Solution
	 * Optimize solution
	 * For each UGV...
	 *   Perform action swapping
	 *   For each drone
	 *     Update-Subtours()
	 *   end-for
	 *   While UGV tour has collisions
	 *     Run obstacle avoidance
	 *     Optimize solution
	 *   end-while
	 * end-for
	 */

	// Sanity print
	if(DEBUG_LORS) {
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

	if(DEBUG_LORS) {
		printf("* Found initial solution *\n Current par = %f\n", sol_final->CalculatePar());
		sol_final->GenerateYAML("intial_solution.yaml");
	}

	/// Optimize launch/land actions once
	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		// Need to break things up a little before continuing...
		bool good_run = optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final);
		if(!good_run) {
			// Gurobi failed to find a solution..
			throw PathOptimizerException("[LORS] Gurobi failed after baseline\n");
		}
	}

	if(DEBUG_LORS) {
		double old_par = sol_final->CalculatePar();
		printf("* Optimized initial solution *\n New par = %f\n", old_par);
		sol_final->GenerateYAML("intial_opt.yaml");
	}

	/// For each UGV...
	int iter = 0;
	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		// Are we running the swapping logic?
		if(bSwapping) {
			if(DEBUG_LORS) {
				printf("** Attempting to swap actions **\n");
			}

			/// Perform action swapping
			swapper.LazySwap(input, sol_final, ugv_num, drones_to_UGV);
		}

		// Are we replanning drone sub-tours?
		if(bReplanning) {
			if(DEBUG_LORS) {
				printf("** Attempting to replan sub-tours **\n");
			}
			// Stopping condition
			bool made_update = false;

			// Create a temporary solution
			Solution sol_new(*sol_final);

			/// For each drone
			for(int drone_j : drones_to_UGV.at(ugv_num)) {
				/// Update-Subtours()
				made_update |= updateSubtours(drone_j, &sol_new);
			}

			if(made_update) {
				// Try to optimize this new solution
				bool good_run = optimizer.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, &sol_new);
				// Did the optimizer succeed?
				if(good_run) {
					if(DEBUG_LORS) {
						printf("*** Good re-plan, updating solution ***\n");
					}
					/// Update the final solution
					*sol_final = Solution(sol_new);
				}
			}
		}

		if(DEBUG_LORS) {
			printf("* Running obstacle avoidance *\n");
		}
		/// While UGV tour has collisions... run obstacle avoidance
		while(moveAroundObstacles(ugv_num, input, sol_final, drones_to_UGV)) {
			if(DEBUG_LORS) {
				printf("** Ran avoidance algorithm, updating solution **\n");
			}

			if(DEBUG_LORS) {
				char buff[100];
				snprintf(buff, sizeof(buff), "midsolve_%d.yaml", iter);
				std::string buffAsStdStr = buff;
				sol_final->GenerateYAML(buffAsStdStr);
				iter++;
			}

			/// Optimize solution
			bool good_run = optimizerOBS.OptLaunching(ugv_num, drones_to_UGV.at(ugv_num), input, sol_final);
			if(!good_run) {
				// Gurobi failed to find a solution..
				throw PathOptimizerException("[LORS] Gurobi failed while avoiding obstacles\n");;
			}

			if(DEBUG_LORS) {
				char buff[100];
				snprintf(buff, sizeof(buff), "midsolve_%d.yaml", iter);
				std::string buffAsStdStr = buff;
				sol_final->GenerateYAML(buffAsStdStr);
				iter++;
			}

//			if(iter > 4) {
//				sol_final->PrintSolution();
//				exit(1);
//			}

			if(DEBUG_LORS) {
				printf("** Ran optimizer, running obstacle avoidance **\n");
			}
		}

		if(DEBUG_LORS) {
			printf("* Team complete, check next team *\n");
		}
	}

	if(DEBUG_LORS) {
		printf("* LORS algorithm complete *\n");
	}
}
