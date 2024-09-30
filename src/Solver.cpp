#include "Solver.h"

Solver::Solver() {
}

Solver::~Solver() {}


// Runs the baseline solver, setting an initial solution to build off of
void Solver::RunBaseline(PatrollingInput* input, Solution* sol_final, std::vector<std::vector<int>>& drones_to_UGV) {
	// Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();
	// Used for clustering
	std::vector<ClusteringPoint> vctrPoIPoint;
	// Each depot has a set of PoI, an ordering for a UGV to visit, and a set of sub-tours
	std::vector<VRPPoint> depots;
	std::vector<std::vector<int>> depot_order;
	// depot < subtour < PoI > >
	std::vector<std::vector<std::vector<int>>> depots_tours;

	if(DEBUG_SOLVER)
		printf("Input PoI Nodes:\n");
	for(long unsigned int i = 0; i < vctrPOINodes.size(); i++) {
		if(DEBUG_SOLVER)
			printf(" %ld: (%f, %f)\n", i, vctrPOINodes.at(i).location.x, vctrPOINodes.at(i).location.y);
		// Create a PoI point
		ClusteringPoint point(i, vctrPOINodes.at(i).location.x, vctrPOINodes.at(i).location.y, 0);

		// Add to list of PoI Points
		vctrPoIPoint.push_back(point);
	}


	/// 1. k = 1
	int K = 1;

	// While we don't have a valid solution
	bool valid_solution = false;
	while(!valid_solution && K <= boost::numeric_cast<int>(vctrPOINodes.size())) {
		if(SANITY_PRINT)
			printf("\n--------------------------------------------------------\nNew Round, k = %d\n", K);

		// Clear previous solution
		depots.clear();
		depot_order.clear();
		depots_tours.clear();
		sol_final->ClearSolution();
		// Assume we stop increasing k
		valid_solution = true;


		/// 2. form k clusters

		// Clusters array
		std::vector<std::vector<ClusteringPoint>> clusters;
		if(K > 1) {
			// Find k clusters
			mKMeansSolver.SolveKMeans(vctrPoIPoint, K);

			// Add point vectors clusters
			for(int i = 0; i < K; i++) {
				std::vector<ClusteringPoint> points;
				clusters.push_back(points);
			}

			// Push points into new vectors
			for(ClusteringPoint n : vctrPoIPoint) {
				clusters[n.cluster].push_back(n);
			}
		}
		else {
			// Single cluster...
			clusters.push_back(vctrPoIPoint);
		}

		// Find centroids for each cluster
		for(int i = 0; i < K; i++) {
			// Put a depot in the middle of the cluster
			double x = 0, y = 0;
			int count = 0;
			for(ClusteringPoint n : clusters.at(i)) {
				x += n.x;
				y += n.y;
				count++;
			}
			VRPPoint depot(i, x/count, y/count);
			depots.push_back(depot);
		}

		// Add the depot onto the solution
		{
			double depot_x, depot_y;
			input->GetDepot(0, &depot_x, &depot_y);
			VRPPoint UGV_depot(-1, depot_x, depot_y);
			depots.push_back(UGV_depot);
		}

		// Debugging
		if(DEBUG_SOLVER) {
			printf("Clusters:\n");
			for(int i = 0; i < K; i++) {
				printf(" %d@(%f, %f):  ", i, depots.at(i).x, depots.at(i).y);
				for(ClusteringPoint n : clusters.at(i)) {
					printf("%d@(%f, %f) ", n.ID, n.x, n.y);
				}
				printf("\n");
			}
		}


		/// 3. Solve VRP on centroids of each cluster with depot and m_g vehicles
		mVRPSolver.SolveVRP(depots, input->GetMg(), depot_order);


		/// 4. For each UGV j:
		for(int j_g = 0; j_g < input->GetMg(); j_g++) {


			/// 5. For each cluster assigned to j
			for(int k : depot_order.at(j_g)) {
				// Create a list of nodes for VRP
				std::vector<VRPPoint> nodes;
				// Add the PoI from this cluster
				for(ClusteringPoint n : clusters.at(k)) {
					VRPPoint point(n.ID, n.x, n.y);
					nodes.push_back(point);
				}
				// Add in the depot
				VRPPoint depot(k, depots.at(k).x, depots.at(k).y);
				nodes.push_back(depot);
				if(DEBUG_SOLVER) {
					printf("Solving VRP on cluster %d, depot at (%f, %f)\n", k, depot.x, depot.y);
				}


				/// 6. Solve VRP on cluster with |D_j| vehicles
				std::vector<std::vector<int>> subtours;
				mVRPSolver.SolveVRP(nodes, boost::numeric_cast<int>(drones_to_UGV.at(j_g).size()), subtours);
				depots_tours.push_back(subtours);

				if(DEBUG_SOLVER)
					printf("VRP solver finished, checking tours\n");

				// Calculate the longest sub-tour distance
				for(const std::vector<int> &subtour : subtours) {
					// Find distance from depot to first stop
					double tour_length = distAtoB(depot.x, depot.y, vctrPOINodes.at(subtour.front()).location.x, vctrPOINodes.at(subtour.front()).location.y);
					std::vector<int>::const_iterator prev = subtour.begin();
					std::vector<int>::const_iterator nxt = prev+1;
					while(nxt != subtour.end()) {
						// Calculate distance from prev to nxt
						tour_length += distAtoB(vctrPOINodes.at(*prev).location.x, vctrPOINodes.at(*prev).location.y, vctrPOINodes.at(*nxt).location.x, vctrPOINodes.at(*nxt).location.y);
						// Update iterators
						prev = nxt;
						nxt++;
					}
					// Add in trip back to depot
					tour_length += distAtoB(vctrPOINodes.at(subtour.back()).location.x, vctrPOINodes.at(subtour.back()).location.y, depot.x, depot.y);

					if(DEBUG_SOLVER)
						printf(" Distance of tour: %f\n", tour_length);


					/// 7. If there exists drone-tour A such that dist(A) > d^a_max, increase k and repeat steps 2-8
					if(tour_length > input->GetDroneMaxDist(DRONE_I)) {
						if(DEBUG_SOLVER)
							printf("Tour too long -> increase k\n");

						K++;
						valid_solution = false;
						break;
					}
				}
				if(!valid_solution) {
					break;
				}
			}
			if(!valid_solution) {
				break;
			}
		}

		// Do we still have a valid solution?
		if(valid_solution) {

			/// 8. Form action lists by determining launch/receive times, charge times for drones/UGV
			// Print final routes:
			if(DEBUG_SOLVER) {
				printf("Found valid solution\n");
				printf("\nFound tours:\n");
				// For each UGV
				for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
					printf("UGV %d\n", ugv_num);
					// For each depot that the UGV visits
					for(int n : depot_order.at(ugv_num)) {
						printf(" Depot %d : (%f, %f)\n", n, depots.at(n).x, depots.at(n).y);
						// For each drone that launches from the UGV
						for(long unsigned int drone_i = 0; drone_i < depots_tours.at(n).size(); drone_i++) {
							int drone = drones_to_UGV.at(ugv_num).at(drone_i);
							printf("  Drone %d:\n", drone);
							// Print which points this drone visits
							for(int stop : depots_tours.at(n).at(drone)) {
								printf("   %d:%s (%f, %f)\n", stop, vctrPOINodes.at(stop).ID.c_str(), vctrPOINodes.at(stop).location.x, vctrPOINodes.at(stop).location.y);
							}
						}
					}
				}
			}

			if(DEBUG_SOLVER)
				printf("\nBuilding Action Lists:\n");

//			int actionID = 0;
			sol_final->ClearSolution();

			// Note how long each drone needs to charge before launching
			std::vector<double> chargeTimes;
			// Track how much energy has been transfered to drones
			std::vector<double> ugvEnergySpent;

			// Set initial charge times (updated later)
			for(int drone = 0; drone < input->GetMa(); drone++) {
				chargeTimes.push_back(0.0);
			}

			// Push initial actions for all UGV and drones
			for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
				// UGV is at the base depot
				UGVAction initialAction(E_UGVActionTypes::e_AtDepot, depots.back().x, depots.back().y, 0);
				sol_final->PushUGVAction(ugv_num, initialAction);
				// Initially, we do not give any energy to the drone
				ugvEnergySpent.push_back(0.0);
				// For each drone assigned to this UGV (we expect these to go in numerical order)
				for(int drone : drones_to_UGV.at(ugv_num)) {
					// Drone is at the UGV
					DroneAction initialAction(E_DroneActionTypes::e_AtUGV, depots.back().x, depots.back().y, 0, ugv_num);
					sol_final->PushDroneAction(drone, initialAction);
				}

				// Determine how long each drone should have been charged from previous tour
				for(long unsigned int drone_i = 0; drone_i < depots_tours.at(depot_order.at(ugv_num).back()).size(); drone_i++) {
					int drone = drones_to_UGV.at(ugv_num).at(drone_i);
					UAV uav = input->getUAV(drone);
					double totalFlyTime = 0.0;
					double prev_x = depots.at(depot_order.at(ugv_num).back()).x;
					double prev_y = depots.at(depot_order.at(ugv_num).back()).y;
					// For each PoI in drone tour
					for(int stop : depots_tours.at(depot_order.at(ugv_num).back()).at(drone_i)) {
						// Determine time required to move to node
						double dist_prev_next = distAtoB(prev_x, prev_y, vctrPOINodes.at(stop).location.x, vctrPOINodes.at(stop).location.y);
						totalFlyTime += dist_prev_next/input->GetDroneVMax(DRONE_I);
						// Update for next stop
						prev_x = vctrPOINodes.at(stop).location.x;
						prev_y = vctrPOINodes.at(stop).location.y;
					}
					// Move back to UGV
					double dist_to_ugv = distAtoB(prev_x, prev_y, depots.at(depot_order.at(ugv_num).back()).x, depots.at(depot_order.at(ugv_num).back()).y);
					totalFlyTime += dist_to_ugv/input->GetDroneVMax(DRONE_I);
					// Energy from flying
					// double joules = totalFlyTime*DRONE_JOULES_PER_SECONDS;
					double energyPerSecond = uav.getJoulesPerSecondFlying(uav.maxSpeed);
					double joules = totalFlyTime*energyPerSecond;
					// Add in energy required to launch and to land
					joules += uav.energyToTakeOff + uav.energyToLand;
					// joules += LAUNCH_ENERGY + LAND_ENERGY;
					// Determine how long the drone must charge
					double finalChargeTime = input->calcChargeTime(drone, joules);

					// Determine time required to move to base
					double dist_to_base = distAtoB(depots.at(depot_order.at(ugv_num).back()).x, depots.at(depot_order.at(ugv_num).back()).y, depots.back().x, depots.back().y);
					// double time_to_base = dist_to_base/UGV_V_CRG;
					double time_to_base = dist_to_base/input->getUGV(ugv_num).ugv_v_crg;
					// How long has the drone been charging by the end of the kernel?
					UGV ugv = input->getUGV(ugv_num);
					double kernel_end_time = time_to_base + ugv.batterySwapTime;
					// double kernel_end_time = time_to_base + UGV_BAT_SWAP_TIME;
					double charge_time_at_start = std::max(finalChargeTime - kernel_end_time, 0.0);
					chargeTimes.at(drone) = charge_time_at_start;

					if(DEBUG_SOLVER)
						printf("Drone %d, requires %f charging at start\n", drone, charge_time_at_start);
				}
			}

			// Used to track when the kernel ends
			double kernel_complete_time;

			// For each UGV
			for(int ugv_num = 0; ugv_num < input->GetMg() && valid_solution; ugv_num++) {
				if(DEBUG_SOLVER)
					printf(" UGV: %d\n", ugv_num);

				// For each depot that the UGV visits
				for(int n : depot_order.at(ugv_num)) {
					// Order drones by the time each drone finishes charging
					std::priority_queue<Arrival, std::vector<Arrival>, CompareArrival> charging_queue;
					for(int drone : drones_to_UGV.at(ugv_num)) {
						DroneAction drone_last = sol_final->GetLastDroneAction(drone);
						// We assume that the last drone action was landing (or at the depot)
						double charge_end_time = chargeTimes.at(drone) + drone_last.fCompletionTime;
						charging_queue.emplace(charge_end_time, drone);

					}

					double ugv_arrival_time = 0.0;
					// Move to the depot
					{
						// Determine how long it took to move to the next depot
						UGVAction ugv_last = sol_final->GetLastUGVAction(ugv_num);
						double dist_prev_next = distAtoB(ugv_last.fX, ugv_last.fY, depots.at(n).x, depots.at(n).y);
						// double t_duration = dist_prev_next/UGV_V_CRG;
						double t_duration = dist_prev_next/input->getUGV(ugv_num).ugv_v_crg;
						ugv_arrival_time = ugv_last.fCompletionTime + t_duration;
						// Create action for moving here
						UGVAction moveToDepot(E_UGVActionTypes::e_MoveToWaypoint, depots.at(n).x, depots.at(n).y, ugv_arrival_time, n);
						sol_final->PushUGVAction(ugv_num, moveToDepot);
						// Calculate how much energy we used
						UGV ugv = input->getUGV(ugv_num);
						// double moveEnergy = UGV_JOULES_PER_SECONDS_DRIVE*t_duration;
						double drivingEnergy = ugv.getJoulesPerSecondDriving(ugv.maxDriveAndChargeSpeed); 
						double moveEnergy = drivingEnergy*t_duration;
						ugvEnergySpent.at(ugv_num) += moveEnergy;

						if(DEBUG_SOLVER)
							printf("  Arrive at depot %d at %f, energy to move: %f\n", n, ugv_arrival_time, moveEnergy);
					}

					std::priority_queue<Arrival, std::vector<Arrival>, CompareArrival> arrival_queue;
					// For each drone that launches from the UGV
					for(long unsigned int drone_i = 0; drone_i < depots_tours.at(n).size(); drone_i++) {
						// Which drone gets launched next?
						Arrival next_drone = charging_queue.top();
						charging_queue.pop();
						int drone = next_drone.ID;
						UAV uav = input->getUAV(drone);
						double totalFlyTime = 0.0;
						// Launch the drone
						{
							// Determine what time to launch the drone
							UGVAction ugv_last = sol_final->GetLastUGVAction(ugv_num);
							DroneAction drone_last = sol_final->GetLastDroneAction(drone);
							double time_to_charge = chargeTimes.at(drone);
							double done_charging = drone_last.fCompletionTime+time_to_charge;
							double launch_start_time = std::max(ugv_last.fCompletionTime, done_charging);
							// double completion_time = launch_start_time + UAV_LAUNCH_TIME;
							double completion_time = launch_start_time + uav.timeNeededToLaunch;


							// Launch the drone (needs actions for both the drone and the UGV)
							DroneAction launchFromUGV(E_DroneActionTypes::e_LaunchFromUGV, ugv_last.fX, ugv_last.fY, completion_time, ugv_num);
							sol_final->PushDroneAction(drone, launchFromUGV);
							UGVAction launchDrone(E_UGVActionTypes::e_LaunchDrone, ugv_last.fX, ugv_last.fY, completion_time, drone);
							sol_final->PushUGVAction(ugv_num, launchDrone);

							if(DEBUG_SOLVER)
								printf("   Drone %d, done charge at %f, launch at %f\n", drone, done_charging, launch_start_time);
						}
						// For each PoI in drone tour
						for(int stop : depots_tours.at(n).at(drone_i)) {
							// Have the drone visit each node
							DroneAction drone_last = sol_final->GetLastDroneAction(drone);
							// Determine time required to move to node
							double dist_prev_next = distAtoB(drone_last.fX, drone_last.fY, vctrPOINodes.at(stop).location.x, vctrPOINodes.at(stop).location.y);
							double t_duration = dist_prev_next/input->GetDroneVMax(DRONE_I);
							double visit_time = drone_last.fCompletionTime + t_duration;
							totalFlyTime += t_duration;
							DroneAction moveToNode(E_DroneActionTypes::e_MoveToNode, vctrPOINodes.at(stop).location.x,
									vctrPOINodes.at(stop).location.y, visit_time, stop);
							sol_final->PushDroneAction(drone, moveToNode);

							if(DEBUG_SOLVER)
								printf("    Visit PoI %d at %f\n", stop, visit_time);
						}
						// Move back to UGV
						{
							DroneAction drone_last = sol_final->GetLastDroneAction(drone);
							UGVAction ugv_last = sol_final->GetLastUGVAction(ugv_num);
							// Determine time required to move to node
							double dist_prev_next = distAtoB(drone_last.fX, drone_last.fY, ugv_last.fX, ugv_last.fY);
							double t_duration = dist_prev_next/input->GetDroneVMax(DRONE_I);
							totalFlyTime += t_duration;
							double arrival_time = drone_last.fCompletionTime + t_duration;
							DroneAction moveToUGV(E_DroneActionTypes::e_MoveToUGV, ugv_last.fX, ugv_last.fY, arrival_time, ugv_num);
							// Push action
							sol_final->PushDroneAction(drone, moveToUGV);
							// Add arrival time to priority queue
							arrival_queue.emplace(arrival_time, drone);

							if(DEBUG_SOLVER)
								printf("    Arrive at UGV %d at %f\n", ugv_num, arrival_time);
						}

						// Determine how many joules of energy each drone has consumed
						{
							// Energy from launching and landing
							// double joules = totalFlyTime*DRONE_JOULES_PER_SECONDS;
							double energyPerSecond = uav.getJoulesPerSecondFlying(uav.maxSpeed);
							double joules = totalFlyTime*energyPerSecond;
							// Add in energy required to launch and to land
							UAV uav = input->getUAV(drone);
							joules += uav.energyToTakeOff + uav.energyToLand;
							// joules += LAUNCH_ENERGY + LAND_ENERGY;
							// Determine how long the drone must charge
							double chargeTime = input->calcChargeTime(drone, joules);
							chargeTimes.at(drone) = chargeTime;
							// Record how much the UGV has given away
							UGV ugv = input->getUGV(ugv_num);
							// ugvEnergySpent.at(ugv_num) += joules/CHARGE_EFFICIENCY;
							ugvEnergySpent.at(ugv_num) += joules/ugv.chargeEfficiency;

							if(DEBUG_SOLVER)
								printf("    Energy used: %f, charge time: %f\n", joules, chargeTime);
						}
					}

					// Used to UGV wait time
					double ugv_exit_time = 0.0;

					// Land drones on UGV
					while(!arrival_queue.empty()) {
						// Get next drone that shows up
						Arrival next_arrival = arrival_queue.top();
						// Have drone land on UGV
						{
							// Get the last action by the UGV
							UGVAction ugv_last = sol_final->GetLastUGVAction(ugv_num);
							// What time did we land?
							int drone = next_arrival.ID;
							UAV uav = input->getUAV(drone);
							// double landOnArrival = next_arrival.time + UAV_LAND_TIME;
							// double landWhenReady = ugv_last.fCompletionTime + UAV_LAND_TIME;
							double landOnArrival = next_arrival.time + uav.timeNeededToLand;
							double landWhenReady = ugv_last.fCompletionTime + uav.timeNeededToLand;
							double landTime = std::max(landOnArrival, landWhenReady);
							// Land the drone
							DroneAction landOnUGV(E_DroneActionTypes::e_LandOnUGV, ugv_last.fX, ugv_last.fY, landTime, ugv_num);
							sol_final->PushDroneAction(next_arrival.ID, landOnUGV);
							UGVAction landDrone(E_UGVActionTypes::e_ReceiveDrone, ugv_last.fX, ugv_last.fY, landTime, next_arrival.ID);
							sol_final->PushUGVAction(ugv_num, landDrone);
							// If needed, update when the UGV moves on
							ugv_exit_time = std::max(ugv_exit_time, landTime);

							if(DEBUG_SOLVER)
								printf("   Drone %d arrived at %f, lands at %f\n", next_arrival.ID, next_arrival.time, landTime);
						}
						// Pop arrival data off of priority queue
						arrival_queue.pop();
					}

					// Calculate how much energy we used while waiting
					double ugv_wait_time = ugv_exit_time - ugv_arrival_time;
					UGV ugv = input->getUGV(ugv_num);
					double ugv_wait_energy = ugv.joulesPerSecondWhileWaiting*ugv_wait_time;
					// double ugv_wait_energy = UGV_JOULES_PER_SECONDS_WAIT*ugv_wait_time;
					ugvEnergySpent.at(ugv_num) += ugv_wait_energy;
					if(DEBUG_SOLVER)
						printf("   UGV energy waiting: %f\n", ugv_wait_energy);
				}

				// Return UGV to base
				{
					// Get last action
					UGVAction ugv_last = sol_final->GetLastUGVAction(ugv_num);
					// Determine time required to move to base
					double dist_prev_next = distAtoB(ugv_last.fX, ugv_last.fY, depots.back().x, depots.back().y);
					// double t_duration = dist_prev_next/UGV_V_CRG;
					double t_duration = dist_prev_next/input->getUGV(ugv_num).ugv_v_crg;
					double arrive_time = ugv_last.fCompletionTime + t_duration;
					// Create action for moving here
					UGVAction moveToDepot(E_UGVActionTypes::e_MoveToDepot, depots.back().x, depots.back().y, arrive_time);
					sol_final->PushUGVAction(ugv_num, moveToDepot);

					// Calculate how much energy we used
					UGV ugv = input->getUGV(ugv_num);
					double drivingEnergy = ugv.getJoulesPerSecondDriving(ugv.maxDriveAndChargeSpeed); 
					double moveEnergy = drivingEnergy*t_duration; 
					double energy_to_return = moveEnergy*t_duration;
					// double energy_to_return = UGV_JOULES_PER_SECONDS_DRIVE*t_duration;
					ugvEnergySpent.at(ugv_num) += energy_to_return;

					// Swap UGV battery
					kernel_complete_time = arrive_time+ugv.batterySwapTime;
					// kernel_complete_time = arrive_time+UGV_BAT_SWAP_TIME;
					UGVAction ugvChargeAction(E_UGVActionTypes::e_AtDepot, depots.back().x, depots.back().y, kernel_complete_time);
					sol_final->PushUGVAction(ugv_num, ugvChargeAction);

					if(DEBUG_SOLVER) {
						printf("   Arrive at base at %f, energy to move: %f\n", arrive_time, energy_to_return);
						printf("  Total UGV energy used: %f, kernel end time: %f\n", ugvEnergySpent.at(ugv_num), kernel_complete_time);
					}


					// 9. If there exists UGV-tour A such that energy(A) > E^g_max, increase k and repeat steps 2-9
					if(ugvEnergySpent.at(ugv_num) > input->GetUGVBatCap(ugv_num)) {
						if(DEBUG_SOLVER) {
							printf("\nUGV %d will run out of energy!\nIncrease k, start over...\n", ugv_num);
						}

						K++;
						valid_solution = false;
					}
					else {
						// Add in end-of-kernel action for the UGV and each of its drones
						UGVAction ugvEndAction(E_UGVActionTypes::e_KernelEnd, depots.back().x, depots.back().y, kernel_complete_time);
						sol_final->PushUGVAction(ugv_num, ugvEndAction);
						for(int drone : drones_to_UGV.at(ugv_num)) {
							DroneAction endAction(E_DroneActionTypes::e_KernelEnd, depots.back().x, depots.back().y, kernel_complete_time);
							sol_final->PushDroneAction(drone, endAction);
						}
					}
				}
			}
		}
	}
}

/*
 * Solves TSP on on vertices held in lst and stores found ordering in result. The multiplier
 * variable can be set to force the solver to solver a fixed-HPP (forcing the first and last
 * vertices in lst to be connected in the TSP solution).
 */
void Solver::solverTSP_LKH(std::vector<TSPVertex>& lst, std::vector<TSPVertex>& result, double multiplier) {
	if(DEBUG_SOLVER)
		printf("Creating LKH Data Files\n");

	// Mark which vertices are the depot and terminal (assumed to be last and second to last)
	int depot_id = lst.front().nID;
	int terminal_id= lst.back().nID;
	int depot_index = 0;
	int terminal_index= boost::numeric_cast<int>(lst.size()-1);
	if(DEBUG_SOLVER)
		printf(" Depot: %d, Terminal: %d, Multiplier: %f\n", depot_id, terminal_id, multiplier);

	// Print solver parameter file
	if(DEBUG_SOLVER)
		printf(" Creating parameter file\n");
	FILE * pParFile;
	char buff1[100];
	sprintf(buff1, "FixedHPP.par");
	if(DEBUG_SOLVER)
		printf("  %s\n", buff1);
	pParFile = fopen(buff1, "w");

	fprintf(pParFile, "PROBLEM_FILE = FixedHPP.tsp\n");
	fprintf(pParFile, "COMMENT Fixed Hamiltonian Path Problem\n");
	fprintf(pParFile, "TOUR_FILE = LKH_output.dat\n");
	fprintf(pParFile, "TRACE_LEVEL = 0\n");

	if(DEBUG_SOLVER)
		printf("  Done!\n");
	fclose(pParFile);

	// Print node data to file
	if(DEBUG_SOLVER)
		printf(" Creating node data file\n");
	FILE * pDataFile;
	char buff2[100];
	sprintf(buff2, "FixedHPP.tsp");
	if(DEBUG_SOLVER)
		printf("  %s\n", buff2);
	pDataFile = fopen(buff2, "w");

	if(DEBUG_SOLVER)
		printf("  Adding graph specification data\n");
	fprintf(pDataFile, "NAME : FixedHPP \n");
	fprintf(pDataFile, "COMMENT : Fixed Hamiltonian Path Problem \n");
	fprintf(pDataFile, "TYPE : TSP \n");
	fprintf(pDataFile, "DIMENSION : %ld \n", lst.size());
	fprintf(pDataFile, "EDGE_WEIGHT_TYPE : EXPLICIT \n");
	fprintf(pDataFile, "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n");

	if(DEBUG_SOLVER)
		printf("  Adding NxN weights matrix\n");
	fprintf(pDataFile, "EDGE_WEIGHT_SECTION\n");
	for(TSPVertex v : lst) {
		for(TSPVertex u : lst) {
			if((v.nID == depot_id && u.nID == terminal_id) || (v.nID == terminal_id && u.nID == depot_id)) {
				fprintf(pDataFile, "%f\t", 0.0);
			}
			else if((v.nID == depot_id) || (u.nID == depot_id) || (u.nID == terminal_id) || (v.nID == terminal_id)) {
				fprintf(pDataFile, "%f\t", distAtoB(v.x, v.y, u.x, u.y)*multiplier );
			}
			else {
				fprintf(pDataFile, "%.5f\t", distAtoB(v.x, v.y, u.x, u.y));
			}
		}
		fprintf(pDataFile, "\n");
	}

	if(DEBUG_SOLVER)
		printf("  Done!\n");
	fprintf(pDataFile, "EOF\n");
	fclose(pDataFile);

	if(DEBUG_SOLVER)
		printf("Running LKH\n");

	// Run TSP solver on this sub-tour
	int sys_output = std::system("LKH FixedHPP.par > trash.out");

	if(DEBUG_SOLVER)
		printf("System Call returned: %d\nFound the following solution:\n", sys_output);

	// Open file with results
	std::ifstream file("LKH_output.dat");
	// Remove the first few lines...
	std::string line;
	for(int i = 0; i < 6; i++) {
		std::getline(file, line);
	}

	/// Make list of total-tour minus depot and terminal
	std::list<int> totalPath;

	// Start parsing the data
	for(int i = 0; i < boost::numeric_cast<int>(lst.size()); i++) {
		std::getline(file, line);
		std::stringstream lineStreamN(line);
		// Parse the way-point from the line
		int n;
		lineStreamN >> n;
		totalPath.push_back(n-1);
		if(DEBUG_SOLVER)
			printf(" %d", n-1);
	}
	if(DEBUG_SOLVER)
		printf("\n");

	file.close();

	/// Correct the returned list from the LKH solver
	// Check for weird (easy) edge-cases
	if((totalPath.front() == depot_index) && (totalPath.back() == terminal_index)) {
		// Nothing to fix...
	}
	else if((totalPath.front() == terminal_index) && (totalPath.back() == depot_index)) {
		// Easy fix, just reverse the list
		totalPath.reverse();
	}
	else {
		// Correcting the path will take a little more work...
		// Scan totalPath to determine the given order
		bool reverseList = true;
		{
			std::list<int>::iterator it = totalPath.begin();

			while((*it != depot_index) && (it != totalPath.end())) {
				if(*it == terminal_index) {
					reverseList = false;
				}
				it++;
			}
		}

		// Rotate list so that it starts at the closest-to-depot way-point,
		//  and ends with the closest-to-ideal-stop
		bool rotate_again = true;
		while(rotate_again) {
			if(totalPath.front() == depot_index) {
				// Total path has been corrected
				rotate_again = false;
			}
			else {
				// Keep rotating list
				int temp = totalPath.front();
				totalPath.pop_front();
				totalPath.push_back(temp);
			}
		}

		if(reverseList) {
			// We were given the list "backwards", we need to reverse it
			int temp = totalPath.front();
			totalPath.pop_front();
			totalPath.push_back(temp);

			totalPath.reverse();
		}
	}

	// Verify that the list is correct
	if((totalPath.front() != depot_index) || (totalPath.back() != terminal_index)) {
		// Something went wrong...
		fprintf(stderr, "[ERROR] : Solver::solverTSP_LKH() : totalPath order is not as expected\n");
		for(int n : totalPath) {
			fprintf(stderr, " %d", n);
		}
		fprintf(stderr, "\n");
		exit(1);
	}

	// Sanity print
	if(DEBUG_SOLVER) {
		printf("Fixed total path:\n");
		for(int n : totalPath) {
			printf(" %d", n);
		}
		printf("\n");
	}

	// Store the solution in Results
	for(int i : totalPath) {
		result.push_back(lst.at(i));
	}

	// Sanity print
	if(DEBUG_SOLVER) {
		printf("Returning result:\n");
		for(TSPVertex v : result) {
			printf(" %d", v.nID);
		}
		printf("\n\n");
	}
}

