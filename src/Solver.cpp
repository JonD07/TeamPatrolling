#include "Solver.h"

Solver::Solver() {
}

Solver::~Solver() {}


// Runs the baseline solver, setting an initial solution to build off of
void Solver::RunBaseline(PatrollingInput* input, Solution* sol_final, std::vector<std::vector<int>>& drones_to_UGV, bool obstacle_avoidance) {
	// Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();
	// Used for clustering
	std::vector<ClusteringPoint> vctrPoIPoint;
	// Each depot has a set of PoI, an ordering for a UGV to visit, and a set of sub-tours
	std::vector<VRPPoint> depots;
	// The order that each team visits cluster depots
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


	/// 1. m' = m_g, k = 1
	/// 2. k = max(k, m')
	/// 3. Form k clusters
	/// 3.5 (optional) push centroids out of obstacles
	/// 4. Solve VRP on centroids of each cluster with depot and m' vehicles
	/// 5. For each UGV j:
	/// 6.   For each cluster assigned to j
	/// 7.     Solve VRP on cluster with |D_j| vehicles
	/// 8.     If there exists drone-tour A such that dist(A) > d^a_max, increase k and repeat steps 2-8
	/// 9. Form action lists by determining launch/receive times, charge times for drones/UGV
	/// 9.5 (optional) add obstacle avoidance to UGV tour
	///10. If there exists UGV-tour A such that energy(A) > E^g_max, increase m' by m_g and repeat steps 2-10


	/// 1. m' = m_g, k = 1
	int K = 1;
	int Mp = input->GetMg();

	/*
	 * Increasing clusters (k) -> reduces the work that each drone must do
	 * Increasing VRP vehicles to visit centroids -> reduces the work each UGV must do per tour
	 */

	// While we don't have a valid solution
	bool valid_solution = false;
	while(!valid_solution) {

		// Clear previous solution
		depots.clear();
		depot_order.clear();
		depots_tours.clear();
		sol_final->ClearSolution();
		// Assume we stop increasing k
		valid_solution = true;


		/// 2. k = max(k, m')
		K = std::max(Mp, K);
		if(SANITY_PRINT)
			printf("\n--------------------------------------------------------\n** New Round, k = %d, m' = %d\n", K, Mp);

		// Did we go over...?
		if(K > boost::numeric_cast<int>(vctrPOINodes.size())) {
			// We did.. just exit!
			fprintf(stderr, "[%s][Solver:RunBaseline] K = %d > N = %d\n", ERROR, K, input->GetN());
			throw std::runtime_error("RunBaseline Error\n");
		}

		// Set empty set of sub-tours for each cluster
		for(int k = 0; k < K; k++) {
			std::vector<std::vector<int>> subtours_on_cluster;
			depots_tours.push_back(subtours_on_cluster);
		}


		/// 3. Form k clusters

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
			double c_x = x/count;
			double c_y = y/count;

			// Are we avoiding obstacles?
			if(obstacle_avoidance) {
				/// 3.5 (optional) push centroids out of obstacles
				findClosestOutsidePointIterative(&c_x, &c_y, input->GetObstacles());
			}

			VRPPoint depot(i, c_x, c_y);
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
			printf("Solver VRP on cluster centroids\n");
		}


		/// 4. Solve VRP on centroids of each cluster with depot and m' vehicles
		mVRPSolver.SolveVRP(depots, Mp, depot_order);

		// Debugging
		if(DEBUG_SOLVER) {
			printf("Ordering to cluster depots:\n");
			for(int i = 0; i < K; i++) {
				printf(" %d@(%f, %f):  ", i, depots.at(i).x, depots.at(i).y);
				for(ClusteringPoint n : clusters.at(i)) {
					printf("%d@(%f, %f) ", n.ID, n.x, n.y);
				}
				printf("\n");
			}
		}


		/// 5. For each UGV j:
		for(int j_p = 0; j_p < Mp && j_p < boost::numeric_cast<int>(depot_order.size()); j_p++) {
			int j_actual = j_p % input->GetMg();

			/// 6. For each cluster assigned to j
			for(int k : depot_order.at(j_p)) {
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


				/// 7. Solve VRP on cluster with |D_j| vehicles
				std::vector<std::vector<int>> subtours;
				mVRPSolver.SolveVRP(nodes, boost::numeric_cast<int>(drones_to_UGV.at(j_actual).size()), subtours);
				depots_tours.at(k) = subtours;
//				depots_tours.push_back(subtours);

				if(DEBUG_SOLVER)
					printf("VRP solver finished, checking tours\n");

				// Calculate the longest sub-tour distance
				/// 8. If there exists drone-tour A such that dist(A) > d^a_max, increase k and repeat steps 2-8
				for(const std::vector<int> &subtour : subtours) {
					if(subtour.size() > 0) {
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


						// dist(A) > d^a_max ?
						if(tour_length > input->GetDroneMaxDist(DRONE_I)) {
							// Yes, increase k and repeat steps 2-8
							if(DEBUG_SOLVER)
								printf("Drone tour too long -> increase k\n");

							K++;
							valid_solution = false;
							break;
						}
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

		// Print final routes:
		if(DEBUG_SOLVER) {
			printf("\nFound working drone sub-tours:\n");
			// For each UGV
			for(int ugv_num = 0; ugv_num < Mp && ugv_num < boost::numeric_cast<int>(depot_order.size()); ugv_num++) {
				int j_actual = ugv_num % input->GetMg();
				printf("UGV %d\n", j_actual);
				// For each depot that the UGV visits
				for(int n : depot_order.at(ugv_num)) {
					printf(" Depot %d : (%f, %f)\n", n, depots.at(n).x, depots.at(n).y);
					// For each drone that launches from the UGV
					for(long unsigned int drone_i = 0; drone_i < depots_tours.at(n).size(); drone_i++) {
						int drone = drones_to_UGV.at(j_actual).at(drone_i);
						printf("  Drone %d:\n", drone);
						// Print which points this drone visits
						for(int stop : depots_tours.at(n).at(drone_i)) {
							printf("   %d:%s (%f, %f)\n", stop, vctrPOINodes.at(stop).ID.c_str(), vctrPOINodes.at(stop).location.x, vctrPOINodes.at(stop).location.y);
						}
					}
				}
			}
		}

		// Do we still have a valid solution?
		if(valid_solution) {
			/// 9. Form action lists by determining launch/receive times, charge times for drones/UGV
			if(DEBUG_SOLVER)
				printf("\nBuilding Action Lists:\n");

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
			for(int ugv_num_p = 0; ugv_num_p < Mp && ugv_num_p < boost::numeric_cast<int>(depot_order.size()); ugv_num_p++) {
				int j_actual = ugv_num_p % input->GetMg();
				// Initially, we do not give any energy to the drone
				ugvEnergySpent.push_back(0.0);

				// Is this the first time we are addressing this UGV?
				if(ugv_num_p < input->GetMg()) {
					// Yes, give this UGV and its drones a starting action
					UGVAction initialAction(E_UGVActionTypes::e_AtDepot, depots.back().x, depots.back().y, 0);
					sol_final->PushUGVAction(j_actual, initialAction);
					// For each drone assigned to this UGV (we expect these to go in numerical order)
					for(int drone : drones_to_UGV.at(j_actual)) {
						// Drone is at the UGV
						DroneAction initialAction(E_DroneActionTypes::e_AtUGV, depots.back().x, depots.back().y, 0, j_actual);
						sol_final->PushDroneAction(drone, initialAction);
					}
				}

				/// Assume that all drones are recharged before starting next tour
#if 0
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
					double energyPerSecond = uav.getJoulesPerSecondFlying(uav.maxSpeed);
					double joules = totalFlyTime*energyPerSecond;
					// Add in energy required to launch and to land
					joules += uav.energyToTakeOff + uav.energyToLand;
					// Determine how long the drone must charge
					double finalChargeTime = input->calcChargeTime(drone, joules);

					// Determine time required to move to base
					double dist_to_base = distAtoB(depots.at(depot_order.at(ugv_num).back()).x, depots.at(depot_order.at(ugv_num).back()).y, depots.back().x, depots.back().y);
					double time_to_base = dist_to_base/input->getUGV(ugv_num).maxDriveAndChargeSpeed;
					// How long has the drone been charging by the end of the kernel?
					UGV ugv = input->getUGV(ugv_num);
					double kernel_end_time = time_to_base + ugv.batterySwapTime;
					double charge_time_at_start = std::max(finalChargeTime - kernel_end_time, 0.0);
					chargeTimes.at(drone) = charge_time_at_start;

					if(DEBUG_SOLVER)
						printf("Drone %d, requires %f charging at start\n", drone, charge_time_at_start);
				}


#endif
			}

			// For each UGV
			for(int ugv_num_p = 0; ugv_num_p < Mp && valid_solution && ugv_num_p < boost::numeric_cast<int>(depot_order.size()); ugv_num_p++) {
				int j_actual = ugv_num_p % input->GetMg();
				if(DEBUG_SOLVER)
					printf(" UGV: %d\n", ugv_num_p);

				// For each depot that the UGV visits
				for(int n : depot_order.at(ugv_num_p)) {
					// Order drones by the time each drone finishes charging
					std::priority_queue<Arrival, std::vector<Arrival>, CompareArrival> charging_queue;
					for(int drone : drones_to_UGV.at(j_actual)) {
						DroneAction drone_last = sol_final->GetLastDroneAction(drone);
						// We assume that the last drone action was landing (or at the depot)
						double charge_end_time = chargeTimes.at(drone) + drone_last.fCompletionTime;
						charging_queue.emplace(charge_end_time, drone);

					}

					double ugv_arrival_time = 0.0;
					// Move to the centroid
					{
						double t_duration = moveUGVtoPoint(input, sol_final, j_actual, depots.at(n).x, depots.at(n).y, n, E_UGVActionTypes::e_MoveToWaypoint, obstacle_avoidance);

						// Determine how long it took to move to the next depot
						ugv_arrival_time = sol_final->GetLastUGVAction(j_actual).fCompletionTime;

						// Calculate how much energy we used
						UGV ugv = input->getUGV(j_actual);
						double drivingEnergy = ugv.getJoulesPerSecondDriving(ugv.maxDriveAndChargeSpeed); 
						double moveEnergy = drivingEnergy*t_duration;
						ugvEnergySpent.at(ugv_num_p) += moveEnergy;

						if(DEBUG_SOLVER)
							printf("  Arrive at depot %d at %f, energy to move: %f\n", n, ugv_arrival_time, moveEnergy);
					}

					std::priority_queue<Arrival, std::vector<Arrival>, CompareArrival> arrival_queue;
					// For each drone that launches from the UGV
					for(long unsigned int drone_i = 0; drone_i < depots_tours.at(n).size(); drone_i++) {
						// Are there any stops on this drone sub-tour..?
						if(depots_tours.at(n).at(drone_i).size() > 0) {
							// Which drone gets launched next?
							Arrival next_drone = charging_queue.top();
							charging_queue.pop();
							int drone = next_drone.ID;
							UAV uav = input->getUAV(drone);
							double totalFlyTime = 0.0;
							// Launch the drone
							{
								// Determine what time to launch the drone
								UGVAction ugv_last = sol_final->GetLastUGVAction(j_actual);
								DroneAction drone_last = sol_final->GetLastDroneAction(drone);
								double time_to_charge = chargeTimes.at(drone);
								double done_charging = drone_last.fCompletionTime+time_to_charge;
								double launch_start_time = std::max(ugv_last.fCompletionTime, done_charging);
								double completion_time = launch_start_time + uav.timeNeededToLaunch;


								// Launch the drone (needs actions for both the drone and the UGV)
								DroneAction launchFromUGV(E_DroneActionTypes::e_LaunchFromUGV, ugv_last.fX, ugv_last.fY, completion_time, j_actual);
								sol_final->PushDroneAction(drone, launchFromUGV);
								UGVAction launchDrone(E_UGVActionTypes::e_LaunchDrone, ugv_last.fX, ugv_last.fY, completion_time, drone);
								sol_final->PushUGVAction(j_actual, launchDrone);

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
								UGVAction ugv_last = sol_final->GetLastUGVAction(j_actual);
								// Determine time required to move to node
								double dist_prev_next = distAtoB(drone_last.fX, drone_last.fY, ugv_last.fX, ugv_last.fY);
								double t_duration = dist_prev_next/input->GetDroneVMax(DRONE_I);
								totalFlyTime += t_duration;
								double arrival_time = drone_last.fCompletionTime + t_duration;
								DroneAction moveToUGV(E_DroneActionTypes::e_MoveToUGV, ugv_last.fX, ugv_last.fY, arrival_time, j_actual);
								// Push action
								sol_final->PushDroneAction(drone, moveToUGV);
								// Add arrival time to priority queue
								arrival_queue.emplace(arrival_time, drone);

								if(DEBUG_SOLVER)
									printf("    Arrive at UGV %d at %f\n", j_actual, arrival_time);
							}

							// Determine how many joules of energy each drone has consumed
							{
								// Energy from launching and landing
								double energyPerSecond = uav.getJoulesPerSecondFlying(uav.maxSpeed);
								double joules = totalFlyTime*energyPerSecond;
								// Add in energy required to launch and to land
								UAV uav = input->getUAV(drone);
								joules += uav.energyToTakeOff + uav.energyToLand;
								// Determine how long the drone must charge
								double chargeTime = input->calcChargeTime(drone, joules);
								chargeTimes.at(drone) = chargeTime;
								// Record how much the UGV has given away
								UGV ugv = input->getUGV(j_actual);
								ugvEnergySpent.at(ugv_num_p) += joules/ugv.chargeEfficiency;

								if(DEBUG_SOLVER)
									printf("    Energy used: %f, charge time: %f\n", joules, chargeTime);
							}
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
							UGVAction ugv_last = sol_final->GetLastUGVAction(j_actual);
							// What time did we land?
							int drone = next_arrival.ID;
							UAV uav = input->getUAV(drone);
							double landOnArrival = next_arrival.time + uav.timeNeededToLand;
							double landWhenReady = ugv_last.fCompletionTime + uav.timeNeededToLand;
							double landTime = std::max(landOnArrival, landWhenReady);
							// Land the drone
							DroneAction landOnUGV(E_DroneActionTypes::e_LandOnUGV, ugv_last.fX, ugv_last.fY, landTime, j_actual);
							sol_final->PushDroneAction(next_arrival.ID, landOnUGV);
							UGVAction landDrone(E_UGVActionTypes::e_ReceiveDrone, ugv_last.fX, ugv_last.fY, landTime, next_arrival.ID);
							sol_final->PushUGVAction(j_actual, landDrone);
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
					UGV ugv = input->getUGV(j_actual);
					double ugv_wait_energy = ugv.joulesPerSecondWhileWaiting*ugv_wait_time;
					ugvEnergySpent.at(ugv_num_p) += ugv_wait_energy;
					if(DEBUG_SOLVER)
						printf("   UGV energy waiting: %f\n", ugv_wait_energy);
				}

				// Return UGV to base
				{
					double t_duration = moveUGVtoPoint(input, sol_final, j_actual, depots.back().x, depots.back().y, -1, E_UGVActionTypes::e_MoveToDepot, obstacle_avoidance);
					// The last action should have been "return to base"
					double arrive_time = sol_final->GetLastUGVAction(j_actual).fCompletionTime;

					// Calculate how much energy we used
					UGV ugv = input->getUGV(j_actual);
					double drivingEnergy = ugv.getJoulesPerSecondDriving(ugv.maxDriveAndChargeSpeed); 
					double energy_to_return = drivingEnergy*t_duration; 
					ugvEnergySpent.at(ugv_num_p) += energy_to_return;

					// Swap UGV battery
					double team_tour_complete_time = arrive_time+ugv.batterySwapTime;
					UGVAction ugvChargeAction(E_UGVActionTypes::e_AtDepot, depots.back().x, depots.back().y, team_tour_complete_time);
					sol_final->PushUGVAction(j_actual, ugvChargeAction);

					// Add an "at UGV" command to each drone
					for(int drone : drones_to_UGV.at(j_actual)) {
						// Drone is at the UGV
						DroneAction lastAction(E_DroneActionTypes::e_AtUGV, depots.back().x, depots.back().y, team_tour_complete_time, j_actual);
						sol_final->PushDroneAction(drone, lastAction);
					}


					if(DEBUG_SOLVER) {
						printf("   Arrive at base at %f, energy to move: %f\n", arrive_time, energy_to_return);
						printf("  Total UGV energy used: %f, kernel end time: %f\n", ugvEnergySpent.at(ugv_num_p), team_tour_complete_time);
					}


					/// 10. If there exists UGV-tour A such that energy(A) > E^g_max, increase m' by m_g and repeat steps 2-10
					if(ugvEnergySpent.at(ugv_num_p) > input->GetUGVBatCap(j_actual)) {
						if(DEBUG_SOLVER) {
							printf("\n\n\n\n***** UGV %d will run out of energy!\nIncrease m', start over...\n", j_actual);
						}

						Mp += input->GetMg();
						valid_solution = false;
					}
				}
			}

			// If the solution is still valid at this point... we are done!
			if(valid_solution) {
				// Add in end-of-kernel action for each UGV and each of its drones
				for(int ugv_num = 0; ugv_num < input->GetMg() ; ugv_num++) {
					// The last action should have been to wait for the UGV to swap batteries
					UGVAction ugv_last = sol_final->GetLastUGVAction(ugv_num);
					double kernel_complete_time = ugv_last.fCompletionTime;

					UGVAction ugvEndAction(E_UGVActionTypes::e_KernelEnd, depots.back().x, depots.back().y, kernel_complete_time);
					sol_final->PushUGVAction(ugv_num, ugvEndAction);
					for(int drone : drones_to_UGV.at(ugv_num)) {
						DroneAction endAction(E_DroneActionTypes::e_KernelEnd, depots.back().x, depots.back().y, kernel_complete_time);
						sol_final->PushDroneAction(drone, endAction);
					}
				}

				if(DEBUG_SOLVER) {
					sol_final->PrintSolution();
				}
			}
		}
	}
}


double Solver::calcUGVMovingEnergy(UGVAction& UGV_last, UGVAction& UGV_current, UGV& UGV_current_object) {
	double dist_prev_next = distAtoB(UGV_last.fX, UGV_last.fY, UGV_current.fX, UGV_current.fY);
	double t_duration = dist_prev_next/UGV_current_object.maxDriveAndChargeSpeed;
	double drivingEnergy = UGV_current_object.getJoulesPerSecondDriving(UGV_current_object.maxDriveAndChargeSpeed);
	double move_energy = drivingEnergy*t_duration; 
	if (DEBUG_SOL) {
		printf("The Energy calcuated from the move: [%f]\n", move_energy);
	}
	return move_energy; 
}

double Solver::calcDroneMovingEnergy(std::vector<DroneAction>& droneActionsSoFar,std::queue<DroneAction>& drone_action_queue, UAV UAV_current_object, double UAV_VMax) {

	// TODO figure out the totalFlyTime from the drone_action_queue 
	double totalFlyTime = 0;
	while (!drone_action_queue.empty()) {
		DroneAction currentDroneAction = drone_action_queue.front();
		drone_action_queue.pop(); // Remove the action after printing


		// * If the drone action tpye is a MoveToNode or MoveToUGV we should update our flight time 
		if (currentDroneAction.mActionType == E_DroneActionTypes::e_MoveToNode || currentDroneAction.mActionType == E_DroneActionTypes::e_MoveToUGV) {
			if (droneActionsSoFar.empty()) {
				throw std::runtime_error("Drone Actions Vector is empty and this should be impossible");

			}
			DroneAction dronePrev = droneActionsSoFar.back(); 
			double dist_prev_next = distAtoB(dronePrev.fX, dronePrev.fY, currentDroneAction.fX, currentDroneAction.fY);
			double t_duration = dist_prev_next/UAV_VMax;
			totalFlyTime += t_duration;

		}

		droneActionsSoFar.push_back(currentDroneAction);
		// * If the action type is a land on UGV we are this energy step and we have our flight time
		if (currentDroneAction.mActionType == E_DroneActionTypes::e_LandOnUGV) {
			break; 
		}
	}

	
	// Energy from launching and landing
	double energyPerSecond = UAV_current_object.getJoulesPerSecondFlying(UAV_current_object.maxSpeed);
	double joules = totalFlyTime*energyPerSecond;
	// Add in energy required to launch and to land
	joules += UAV_current_object.energyToTakeOff + UAV_current_object.energyToLand;

	if (DEBUG_SOL) {
		printf("The drone used this much energy from launching + visiting + landing: [%f]\n", joules);
	}

	return joules;
}

void Solver::calcDroneWaypointActions(int droneId, int waypointId, std::map< int, std::map<int, std::vector<DroneAction>>>& DroneWaypointActions, std::map< int, std::map<int, std::vector<double>>>& DroneWaypointActionsTimeDifferences, std::queue<DroneAction> drone_action_queue, double prevActionTime) {
	printf("inside the waypoint action function \n"); 

	while (!drone_action_queue.empty()) {
		DroneAction currentDroneAction = drone_action_queue.front();
		drone_action_queue.pop(); // Remove the action after printing
		printf("  [%d] %d(%d) : (%f, %f) - %f\n", currentDroneAction.mActionID, static_cast<std::underlying_type<E_UGVActionTypes>::type>(currentDroneAction.mActionType), currentDroneAction.mDetails, currentDroneAction.fX, currentDroneAction.fY, currentDroneAction.fCompletionTime);

		if (currentDroneAction.mActionType == E_DroneActionTypes::e_AtUGV) {
			prevActionTime = currentDroneAction.fCompletionTime; 
			printf(" special prev time [%f] \n", prevActionTime); 
			continue; 
		}
		DroneWaypointActions[droneId][waypointId].push_back(currentDroneAction);
		double timeDifference = currentDroneAction.fCompletionTime - prevActionTime; 
		printf(" current time [%f] and previous time [%f] \n", currentDroneAction.fCompletionTime, prevActionTime);
		printf("time difference [%f]\n", timeDifference); 
		DroneWaypointActionsTimeDifferences[droneId][waypointId].push_back(timeDifference);
		prevActionTime = currentDroneAction.fCompletionTime;

		// * If the action type is a land on UGV we are done with this waypoint 
		if (currentDroneAction.mActionType == E_DroneActionTypes::e_LandOnUGV) {
			break; 
		}
	}
}



void Solver::RunDepletedSolver(PatrollingInput* input, Solution* sol_final, std::vector<std::vector<int>>& drones_to_UGV){
	if(DEBUG_SOL) {
		printf("Hello from Run Depleted Solver!\n");
		printf("Building Action Lists to Calculate Energies\n");
	}

	// Intialize data strucutures 
	std::vector<DroneAction> drone_action_list;
	std::vector<UGVAction> ugv_action_list;
	std::vector<std::queue<DroneAction>> drone_action_queues(input->GetMa());
	std::vector<std::queue<UGVAction>> ugv_action_queues(input->GetMg());


	// Add the items into the queues 
	// Print the actions of each robot
	for(int j = 0; j < input->GetMa(); j++) {
		sol_final->GetDroneActionList(j, drone_action_list);
		for(DroneAction action : drone_action_list) {
			drone_action_queues[j].push(action);
		}
		drone_action_list.clear();
	}
	for(int j = 0; j < input->GetMg(); j++) {
		sol_final->GetUGVActionList(j, ugv_action_list);
		for(UGVAction action : ugv_action_list) {
			ugv_action_queues[j].push(action);
		}
		ugv_action_list.clear();
	}


	// Loop through all the UGV 
	
	for (size_t j = 0; j < ugv_action_queues.size(); ++j) { 
		printf("---------------------------------------------------------------------------\n");
		printf("Using the Depleted solver to see if UGV: [%ld] can be optimized (do a loop)\n", j);
		printf("---------------------------------------------------------------------------\n");
		// * This variables will store which actions from the UGVActionsSoFar have been added to the NewUGVActionList
		size_t newUGVSolIndex = 0; 
		double energyUsed = 0; 
		double firstMoveEnergyUsed = 0; 
		std::vector<UGVAction> UGVActionsSoFar;
		std::vector<double> UGVActionsSoFarTimes;
		// * First int is each drone, second int is a particular waypoint 
		std::map< int, std::map<int, std::vector<DroneAction>>> DroneWaypointActions; 
		std::map< int, std::map<int, std::vector<double>>> DroneWaypointActionsTimeDifferences; 
		// * This stores how much time each action adds so I can add this to the previous time during a loop 
		std::vector<double> UGVLoopingActionsTimes;
		std::vector<UGVAction> UGVLoopingActions;
		// * This maps a the drone actions so far for a particular drone  
		std::map<int , std::vector<DroneAction>> droneActionsSoFar;
		std::vector<UGVAction> newUGVActionList; 
		UGVAction previousMoveAction(E_UGVActionTypes::e_MoveToDepot, 0, 0, 0);
		UGVAction firstWaypoint(E_UGVActionTypes::e_MoveToDepot, 0, 0, 0);
		bool hasMoved = false;
		UGV currentUGV = input->getUGV(j);
		int currentUGVId = static_cast<int>(j); 


		// Now go through the action queue for each respective UGV
		while (!ugv_action_queues[currentUGVId].empty()) {
			UGVAction action = ugv_action_queues[currentUGVId].front();
			ugv_action_queues[currentUGVId].pop(); // Remove the action after printing
			if (DEBUG_SOL) {
				printf("  [%d] %d(%d) : (%f, %f) - %f\n",
					action.mActionID,
					static_cast<std::underlying_type<E_UGVActionTypes>::type>(action.mActionType),
					action.mDetails,
					action.fX,
					action.fY,
					action.fCompletionTime);	
			}

			switch (action.mActionType) {
				case E_UGVActionTypes::e_AtDepot:
					if (DEBUG_SOL) {
						printf("At depot (%d) \n", action.mDetails);
					}
					break; 
				case E_UGVActionTypes::e_MoveToWaypoint:
					if (UGVActionsSoFar.empty()) {
						throw std::runtime_error("UGV Actions Vector is empty and this should be impossible");
					}

					if (DEBUG_SOL) {
						printf("Currently at waypoint (%f, %f)\n" , action.fX, action.fY); 
						printf("Came from the following (%f, %f)\n", UGVActionsSoFar.back().fX, UGVActionsSoFar.back().fY);  
					} 


					// * Assuming this is not the first move to waypoint in this trip, there has been some waiting time since the last move
					if (!hasMoved) {
						firstMoveEnergyUsed = calcUGVMovingEnergy(UGVActionsSoFar.back(), action, currentUGV); 
						hasMoved = true; 
						firstWaypoint = action; 
						double actionTimeDifference = action.fCompletionTime - UGVActionsSoFar.back().fCompletionTime;
						UGVActionsSoFar.push_back(action);
						UGVActionsSoFarTimes.push_back(actionTimeDifference);
						continue; 
					}
					else {
						// * We need to calculate the time in between the prevMoveAction and the previous action from the current action 
						double ugv_wait_time = UGVActionsSoFar.back().fCompletionTime - previousMoveAction.fCompletionTime;
						double ugv_wait_energy = currentUGV.joulesPerSecondWhileWaiting*ugv_wait_time;
						energyUsed += calcUGVMovingEnergy(UGVActionsSoFar.back(), action, currentUGV);
						energyUsed += ugv_wait_energy; 

						if(DEBUG_SOL) {
							printf("   UGV energy waiting: %f\n", ugv_wait_energy);
						}

					}
					break; 
				case E_UGVActionTypes::e_LaunchDrone:
					if (DEBUG_SOL) {
						printf("Launching Drone\n");
					}
					{
						int launchedDroneId = action.mDetails;
						int waypointId = UGVActionsSoFar.back().mDetails; 
						std::queue<DroneAction>& launchedDroneQueue = drone_action_queues[launchedDroneId]; 
						std::vector<DroneAction>& launchedDroneActionsSoFar = droneActionsSoFar[launchedDroneId];
						
						double prevActionTime;  
						if (!launchedDroneActionsSoFar.empty()) {
							prevActionTime = launchedDroneActionsSoFar.back().fCompletionTime;
						}
						else {
							prevActionTime = 0.0; 
						}


						
						printf("\n");
						printf("drone id [%d] and waypoint id [%d] \n", launchedDroneId, waypointId);
						printf("\n");

						calcDroneWaypointActions(launchedDroneId, waypointId, DroneWaypointActions, DroneWaypointActionsTimeDifferences, launchedDroneQueue, prevActionTime); 
						double joules = calcDroneMovingEnergy(launchedDroneActionsSoFar, launchedDroneQueue, input->getUAV(launchedDroneId), input->GetDroneVMax(launchedDroneId));

						for (std::size_t i = 0; i < launchedDroneActionsSoFar.size(); i++) {
							DroneAction action = launchedDroneActionsSoFar[i];
							if (DEBUG_SOL) {
								printf("  [%d] %d(%d) : (%f, %f) - %f\n",
									action.mActionID,
									static_cast<std::underlying_type<E_UGVActionTypes>::type>(action.mActionType),
									action.mDetails,
									action.fX,
									action.fY,
									action.fCompletionTime);	
								}
						}
						
						energyUsed += joules/currentUGV.chargeEfficiency;
					}
					break;
				case E_UGVActionTypes::e_MoveToDepot:
					if (DEBUG_SOL) {
						printf("UGV is considering wether to return to depot of loop again \n");
					}
					{
						// We need to compute how much energy it will take to return to the depot 
						double returnDepotEnergy = calcUGVMovingEnergy(UGVActionsSoFar.back(), action, currentUGV);
						// We need to compute how much energy it will take to return to the first waypoint 
						double returnFirstWaypointEnergy = calcUGVMovingEnergy(firstWaypoint, action, currentUGV);

						// Figure out how much energy a second loop would cost 
						double loopEnergy = firstMoveEnergyUsed + energyUsed + returnFirstWaypointEnergy + energyUsed + returnDepotEnergy; 


						if (SANITY_PRINT) {
							printf("\n");
							printf("The energy for the fist move is [%f]\n", firstMoveEnergyUsed); 
							printf("The energy to return to the depot is [%f]\n", returnDepotEnergy);
							printf("The energy to return to the first waypoint is [%f]\n", returnFirstWaypointEnergy);
							printf("The energy to up to this point is: [%f]\n", energyUsed);
							printf("The energy for another loop is [%f]\n", loopEnergy); 
							printf("The UGV has this much total energy [%f]\n", input->GetUGVBatCap(currentUGVId));
							printf("\n");
						}

						// * Figure out how many loops we can do 
						int numLoops = 0;  
						while (loopEnergy < input->GetUGVBatCap(currentUGVId)) {
							numLoops++; 
							loopEnergy += returnFirstWaypointEnergy + energyUsed; 
						}
						
						if (numLoops == 0) {
							printf("\n");
							printf("---------------------------------------------------------\n");
							printf("This Trip could not be optimized by the depleted solver \n");
							printf("---------------------------------------------------------\n");
							printf("\n");
					


							break;
						} else {
							printf("\n");
							printf("---------------------------------------------------------\n");
							printf("This input IS ABLE to be optimized by the depleted solver\n");
							printf("---------------------------------------------------------\n");
							printf("\n");
						}

						
						if (DEBUG_SOL) {
							printf("UGV looping Actions\n"); 
							for(UGVAction action : UGVLoopingActions) {
								printf("  [%d] %d(%d) : (%f, %f) - %f\n", action.mActionID, static_cast<std::underlying_type<E_UGVActionTypes>::type>(action.mActionType), action.mDetails, action.fX, action.fY, action.fCompletionTime);
							}
						}


						// * These two base case must be handled 
						if (newUGVSolIndex == 0) {
							UGVAction copyAction = UGVActionsSoFar[newUGVSolIndex];
							double completionTime = 0;
							newUGVActionList.emplace_back(copyAction.mActionType, copyAction.fX, copyAction.fY, completionTime, copyAction.mDetails);
							newUGVSolIndex++; 
						} 

						// * Add all the actions from the newUGVSolIndex that havent been added yet
						while (newUGVSolIndex < UGVActionsSoFar.size()) {
							UGVAction copyAction = UGVActionsSoFar[newUGVSolIndex];
							double completionTime = newUGVActionList.back().fCompletionTime + UGVActionsSoFarTimes[newUGVSolIndex];
							newUGVActionList.emplace_back(copyAction.mActionType, copyAction.fX, copyAction.fY, completionTime, copyAction.mDetails);
							newUGVSolIndex++; 
						}

	
						// * Add the looped actions to the UGVActions
						for (int i = 0; i < numLoops; i++) {
							// * Create and add a action moving to the first waypoint cords 
							double dist_prev_next = distAtoB(newUGVActionList.back().fX, newUGVActionList.back().fY, firstWaypoint.fX, firstWaypoint.fY);
							double t_duration = dist_prev_next/currentUGV.maxDriveAndChargeSpeed;
							double ugv_arrival_time = newUGVActionList.back().fCompletionTime + t_duration;
							// TODO need to make sure the drones are charged by the time we arrive at first way point again have to wait for them to charge if not 
							UGVAction moveToFirstWaypoint(E_UGVActionTypes::e_MoveToWaypoint, firstWaypoint.fX, firstWaypoint.fY, ugv_arrival_time ,firstWaypoint.mDetails);
							newUGVActionList.push_back(moveToFirstWaypoint);


							// * Create new actions for the looping actions + update new arrival times + add them to new list 
							int index = 0; 
							for (UGVAction loopAction : UGVLoopingActions) {
								double completionTime = newUGVActionList.back().fCompletionTime + UGVLoopingActionsTimes[index];
								newUGVActionList.emplace_back(loopAction.mActionType, loopAction.fX, loopAction.fY, completionTime, loopAction.mDetails);
								index++; 
							}
						}

						// * Rest variables related to the currnet (now that we are returning to a depot) BUT we can still leave for a new trip
						UGVLoopingActions.clear(); 
						UGVLoopingActionsTimes.clear(); 
					}
					break;
				default:
					if (DEBUG_SOL) {
						printf("This action is currently not being considered \n");
					}
					break;
			}

			// * If the UGV has already moved to the first locaction -> we want to keep track of these actions b/c they will represent the loop
			double actionTimeDifference;
			if (UGVActionsSoFar.size() == 0) {
				actionTimeDifference = 0; 
			} else {
				actionTimeDifference = action.fCompletionTime - UGVActionsSoFar.back().fCompletionTime;
			}
			if (hasMoved) {
				UGVLoopingActions.push_back(action); 
				UGVLoopingActionsTimes.push_back(actionTimeDifference); 
			}

			// * Regardless of the action type always add it to actions and action times thus far 
			UGVActionsSoFar.push_back(action);
			UGVActionsSoFarTimes.push_back(actionTimeDifference);
		}


		while (newUGVSolIndex < UGVActionsSoFar.size()) {
			UGVAction action = UGVActionsSoFar[newUGVSolIndex];
			double completionTime; 
			if (newUGVActionList.size() == 0) {
				completionTime = UGVActionsSoFarTimes[newUGVSolIndex];
			}
			else{
				completionTime = newUGVActionList.back().fCompletionTime + UGVActionsSoFarTimes[newUGVSolIndex];
			}
			newUGVActionList.emplace_back(action.mActionType, action.fX, action.fY, completionTime, action.mDetails);
			newUGVSolIndex++; 
		}

		


		for (const auto& dronePair : DroneWaypointActions) {
			int droneId = dronePair.first; // Access the drone ID
			const auto& waypointMap = dronePair.second; // Access the inner map

			printf("Drone ID: %d\n", droneId);

			for (const auto& waypointPair : waypointMap) {
				int waypointId = waypointPair.first; // Access the waypoint ID
				const auto& actions = waypointPair.second; // Access the vector of actions
				const auto& timeDifferences = DroneWaypointActionsTimeDifferences.at(droneId).at(waypointId); // Get corresponding time differences

				printf("  Waypoint ID: %d\n", waypointId);

				for (size_t i = 0; i < actions.size(); ++i) {
					const DroneAction& action = actions[i];
					double timeDifference = timeDifferences[i]; // Corresponding time difference

					printf("    [%d] %d(%d) : (%f, %f) - %f | Time Difference: %f\n",
						action.mActionID,
						static_cast<std::underlying_type<E_DroneActionTypes>::type>(action.mActionType),
						action.mDetails, action.fX, action.fY, action.fCompletionTime,
						timeDifference);
				}
			}
		}

		std::map<int, std::vector<DroneAction>> newDroneActionsLists;

		// Initialize empty vectors for all drones mapped to a particular UGV 
		for (const auto& dronePair : DroneWaypointActions) {
			int droneId = dronePair.first; // Access the drone ID
			newDroneActionsLists[droneId] = {}; // Initialize an empty vector for the drone ID
		}

		printf("Initialized keys for drone IDs:\n");
		for (const auto& pair : newDroneActionsLists) {
			printf("Drone ID: %d initialized with empty action list.\n", pair.first);
		}
		int droneId; 
		int waypointId;
		int index; 
		for (UGVAction newUGVAction : newUGVActionList) {
			switch(newUGVAction.mActionType) {
				case E_UGVActionTypes::e_MoveToWaypoint:
					waypointId = newUGVAction.mDetails;
					break; 
				case E_UGVActionTypes::e_AtDepot:
					// * If the UGV is at the Depot, we should add corresponing the Drone Action at UGV for all UGV - Drone pairs 
					for (auto& dronePair : newDroneActionsLists) {
						droneId = dronePair.first; 
						auto& actionList = dronePair.second; 
						// * Add the new action to the drone's action list
						actionList.emplace_back(E_DroneActionTypes::e_AtUGV, newUGVAction.fX, newUGVAction.fY, newUGVAction.fCompletionTime, newUGVAction.mDetails);
					}
					break;
				case E_UGVActionTypes::e_LaunchDrone:
					droneId = newUGVAction.mDetails;
					index = 0; 
					double completionTime;
					for (DroneAction droneAction : DroneWaypointActions[droneId][waypointId]) {
						if (droneAction.mActionType == E_DroneActionTypes::e_LaunchFromUGV) {
							completionTime = newUGVAction.fCompletionTime; 
						}
						else {
							double prevDroneActionTime = newDroneActionsLists[droneId].back().fCompletionTime;
							completionTime = prevDroneActionTime + DroneWaypointActionsTimeDifferences[droneId][waypointId][index]; 
						}
						newDroneActionsLists[droneId].emplace_back(droneAction.mActionType, droneAction.fX, droneAction.fY, completionTime, droneAction.mDetails); 
						index++; 
					}

					break; 
				case E_UGVActionTypes::e_KernelEnd:
					for (auto& dronePair : newDroneActionsLists) {
						droneId = dronePair.first; 
						auto& actionList = dronePair.second; 
						// * Add the new action to the drone's action list
						actionList.emplace_back(E_DroneActionTypes::e_KernelEnd, newUGVAction.fX, newUGVAction.fY, newUGVAction.fCompletionTime, newUGVAction.mDetails);
					}
					break; 
				default:
					if (DEBUG_SOL) {
						printf(" This action type is not currently being considered \n");
					}
					break; 
			}
		}

		if (SANITY_PRINT) {
			printf("-----------------------------\n");
			printf("New UGVActionList for UGV [%ld]\n", j);
			for (UGVAction action : newUGVActionList) {
				printf("  [%d] %d(%d) : (%f, %f) - %f\n", action.mActionID, static_cast<std::underlying_type<E_UGVActionTypes>::type>(action.mActionType), action.mDetails, action.fX, action.fY, action.fCompletionTime);
			}
			printf("-----------------------------\n");
			for (const auto& pair : newDroneActionsLists) {
				int droneId = pair.first;
				const auto& actions = pair.second;

				std::cout << "Drone ID: " << droneId << std::endl;

				for (DroneAction action : actions) {
					printf("    [%d] %d(%d) : (%f, %f) - %f\n",
						action.mActionID,
						static_cast<std::underlying_type<E_DroneActionTypes>::type>(action.mActionType),
						action.mDetails, action.fX, action.fY, action.fCompletionTime);
				}
			}
			printf("-----------------------------\n");
		}

		// * Now that we have our new UGV Action list and the corresponding Drone actions list we need to update the solution 
		sol_final->swapUGVActionList(currentUGVId, newUGVActionList);
		for (const auto& pair : newDroneActionsLists) {
				int droneId = pair.first;
				const auto& newDroneActionList = pair.second;
				sol_final->swapDroneActionLists(droneId, newDroneActionList);
			}

	}
}

// Adds (possibly multiple) actions to UGV action queue to move the vehicle to a new point
double Solver::moveUGVtoPoint(PatrollingInput* input, Solution* sol_final, double j_actual, double p_x, double p_y, int subtour, E_UGVActionTypes move_type, bool obstacle_avoidance) {
	// Determine how long it took to move to the next depot
	UGVAction ugv_last = sol_final->GetLastUGVAction(j_actual);
	double t_duration = 0;

	/// 9.5 (optional) add obstacle avoidance to UGV tour
	if(obstacle_avoidance) {
		// Is there an obstacle between the last stop and the next?
		UGVAction ugv_nxt(move_type, p_x, p_y, 0.0, subtour);
		bool detected_collision = false;
		for(const Obstacle& obstacle : input->GetObstacles()) {
			if(Obstacle::checkForObstacle(ugv_last.fX, ugv_last.fY, ugv_nxt.fX, ugv_nxt.fY, obstacle)) {
				detected_collision = true;
				break;
			}
		}

		// Did we detect a collision?
		if(detected_collision) {
			// Move out of the obstacle
			if(DEBUG_SOLVER)
				printf("Found collision in baseline\n");
			// * Find a path around the obstacle
		    OMPL_RRTSTAR pathSolver;
			std::vector<std::pair<double, double>> path;
			pathSolver.findPathBetweenActions(input, ugv_last, ugv_nxt, input->GetObstacles(), &path);

			if(DEBUG_SOLVER) {
				printf("Found path between:\n");
				ugv_last.print();
				ugv_nxt.print();
				printf(" Found path:\n  ");
				for(std::pair<double,double> n : path) {
					printf("->(%f,%f)", n.first, n.second);
				}
				puts("");
			}

			if(path.size() >= 3) {
				// * Path found successfully!
				double ugv_x = ugv_last.fX, ugv_y = ugv_last.fY;
				// * Add all middle items in the path to be MoveToPosition Actions
				for(int i = 1; i < boost::numeric_cast<int>(path.size()) - 1; i++) {
					// Determine which obstacle we are maneuvering around
					int obstacle_id = determineAssociatedObstacle({-1, path[i-1].first, path[i-1].second}, {-1, path[i].first, path[i].second}, {-1, path[i+1].first, path[i+1].second}, input->GetObstacles());
					// Is this point associated with an obstacle?
					if(obstacle_id > -1)
					{
						// Get next stop
						double next_x = path[i].first, next_y = path[i].second;
						// Dist/time to move to next stop
						double dist_prev_next = distAtoB(ugv_x, ugv_y, next_x, next_y);
						t_duration += dist_prev_next/input->getUGV(j_actual).maxDriveAndChargeSpeed;
						// Create a new action
						UGVAction tmp(E_UGVActionTypes::e_MoveToPosition, next_x, next_y, ugv_last.fCompletionTime+t_duration, obstacle_id);
						// Add to solution
						sol_final->PushUGVAction(j_actual, tmp);
						// Update position
						ugv_x = next_x, ugv_y = next_y;
					}
					else {
						fprintf(stderr, "[%s][Solver::moveUGVtoPoint] No associated obstacle for waypoint\n", WARNING);
					}
				}

				// Add in next depot
				double next_x = ugv_nxt.fX, next_y = ugv_nxt.fY;
				// Dist/time to move to next stop
				double dist_prev_next = distAtoB(ugv_x, ugv_y, next_x, next_y);
				t_duration += dist_prev_next/input->getUGV(j_actual).maxDriveAndChargeSpeed;
				// Correct arrival time
				ugv_nxt.fCompletionTime = ugv_last.fCompletionTime+t_duration;
				// Add to solution
				sol_final->PushUGVAction(j_actual, ugv_nxt);
			}
			else {
				fprintf(stderr,"[%s][Solver::RunBaseline] Path planning around obstacles failed\n", ERROR);
				throw std::runtime_error("RunBaseline Error\n");
			}
		}
		else {
			if(DEBUG_SOLVER)
				printf("Path from (%f,%f) to (%f,%f) is clear\n", ugv_last.fX, ugv_last.fY, ugv_nxt.fX, ugv_nxt.fY);
			// Add next action as we would normally
			double dist_prev_next = distAtoB(ugv_last.fX, ugv_last.fY, p_x, p_y);
			t_duration = dist_prev_next/input->getUGV(j_actual).maxDriveAndChargeSpeed;
			// Create action for moving here
			UGVAction moveToDepot(move_type, p_x, p_y, ugv_last.fCompletionTime+t_duration, subtour);
			sol_final->PushUGVAction(j_actual, moveToDepot);
		}
	}
	else {
		double dist_prev_next = distAtoB(ugv_last.fX, ugv_last.fY, p_x, p_y);
		t_duration = dist_prev_next/input->getUGV(j_actual).maxDriveAndChargeSpeed;
		// Create action for moving here
		UGVAction moveToDepot(move_type, p_x, p_y, ugv_last.fCompletionTime+t_duration, subtour);
		sol_final->PushUGVAction(j_actual, moveToDepot);
	}

	return t_duration;
}

/*
 * Solves TSP on on vertices held in lst and stores found ordering in result. The multiplier
 * variable can be set to force the solver to solver a fixed-HPP (forcing the first and last
 * vertices in lst to be connected in the TSP solution).
 */
void Solver::solverTSP_LKH(std::vector<GeneralVertex>& lst, std::vector<GeneralVertex>& result, double multiplier) {
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
	for(GeneralVertex v : lst) {
		for(GeneralVertex u : lst) {
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

	// TODO: don't read a file if the solver failed...
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
		fprintf(stderr, "[%s][Solver::solverTSP_LKH] totalPath order is not as expected\n", ERROR);
		for(int n : totalPath) {
			fprintf(stderr, " %d", n);
		}
		fprintf(stderr, "\n");
		throw std::runtime_error("LKH: bad path\n");
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
		for(GeneralVertex v : result) {
			printf(" %d", v.nID);
		}
		printf("\n\n");
	}
}


// * Loop through each UGV actions to check for obstacles, find a path around a obstacle if it exists
bool Solver::moveAroundObstacles(int ugv_num, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV) {
	bool updated_path = false;
	const std::vector<Obstacle> input_obstacles = input->GetObstacles();
    OMPL_RRTSTAR pathSolver;

	// * The first thing we need to do is push any overlapping actions outside of the obstacles
	updated_path |= pushActionsOutside(ugv_num, input, sol_current, drones_to_UGV);

	if(DEBUG_SOLVER)
		printf("** Check for point-to-point collisions\n");

	// Get current action list
	std::vector<UGVAction> ugv_action_list;
	sol_current->GetUGVActionList(ugv_num, ugv_action_list);
	// Create a new action list (might not be needed)
	std::vector<UGVAction> new_UGV_action_list;

	// Check each action
	for(size_t UGV_action_index = 0; UGV_action_index < ugv_action_list.size(); UGV_action_index++) {
		UGVAction current_action = ugv_action_list[UGV_action_index];

		// Make sure we jumped past the first action (it should always be a dummy action)
		if(UGV_action_index > 0) {
			// What type of action is this?
			switch(current_action.mActionType)
			{
				// We are intested in move actions...
				case E_UGVActionTypes::e_MoveToDepot:
				case E_UGVActionTypes::e_MoveToWaypoint:
				case E_UGVActionTypes::e_MoveToPosition:
				{
					// Verify that we don't cross through an obstacle over the move
					bool obstacle_found = false;
					UGVAction previous_action = new_UGV_action_list.back();

					for(const Obstacle& obstacle : input_obstacles) {
						if(Obstacle::checkForObstacle(previous_action.fX, previous_action.fY, current_action.fX, current_action.fY, obstacle)) {
							// We will hit something.. break out of the loop
							obstacle_found = true;
							break;
						}
					}

					if(obstacle_found) {
						if(DEBUG_SOLVER)
							printf("** Found collision!\n");

						// * Set the return variable to true if we have to move around even 1 obstacle
						updated_path = true;

						// * Find a path around the obstacle
						std::vector<std::pair<double, double>> path;
						pathSolver.findPathBetweenActions(input, previous_action, current_action, input_obstacles, &path);

						if(DEBUG_SOLVER) {
							printf("*** Path around obstacle:\n     ");
							for(std::pair<double,double> n : path) {
								printf("->(%f,%f)", n.first, n.second);
							}
							puts("");
						}

						// Verify this is a valid path
						if(path.size() >= 3) {
							// * Add all middle waypoints to the action list
							for(int i = 1; i < boost::numeric_cast<int>(path.size()) - 1; i++) {
								// Determine which obstacle we are maneuvering around
								int obstacle_id = determineAssociatedObstacle({-1, path[i-1].first, path[i-1].second}, {-1, path[i].first, path[i].second}, {-1, path[i+1].first, path[i+1].second}, input_obstacles);
								if(obstacle_id > -1) {
									// Create a new action here
									UGVAction tmp(E_UGVActionTypes::e_MoveToPosition, path[i].first, path[i].second, previous_action.fCompletionTime+(i*0.1), obstacle_id);
									new_UGV_action_list.push_back(tmp);
								}
								else {
									fprintf(stderr, "[%s][Solver::moveAroundObstacles] No associated obstacle for waypoint\n", WARNING);
								}
							}
						}
						else {
							throw std::runtime_error("Path around an obstacle should contain at least 3 points\n");
						}
					}
				}
					break;
				default:
					// * Do nothing
					break;
			}
		}

		// Add the current action to the new list
		new_UGV_action_list.push_back(current_action);

		// Have we been pushing things around..? Then consider removing extra actions.
		// TODO: We should be able to do the following without replanning, but we need to update action times
		if(updated_path) {
			// Is the next action an extra move action to avoid obstacles..?
			if(ugv_action_list[UGV_action_index+1].mActionType == E_UGVActionTypes::e_MoveToPosition) {
				if(DEBUG_SOLVER)
					printf("*** Next is e_MoveToPosition (%d), find next real action...\n", ugv_action_list[UGV_action_index+1].mActionID);
				// Can we skip future MoveToPosition actions?
				size_t jump_to_action_i = UGV_action_index+2;
				// Find next real action...
				while(jump_to_action_i < ugv_action_list.size() && ugv_action_list[jump_to_action_i].mActionType == E_UGVActionTypes::e_MoveToPosition) {
					if(DEBUG_SOLVER)
						printf(" %d is not real..\n", ugv_action_list[jump_to_action_i].mActionID);
					jump_to_action_i++;
				}
				if(DEBUG_SOLVER)
					printf(" * %d is a real action! (%d)\n", ugv_action_list[jump_to_action_i].mActionID, (int)ugv_action_list[jump_to_action_i].mActionType);

				// Work backwards to determine if we can skip the MoveToPosition actions
				while(jump_to_action_i > (UGV_action_index+1)) {
					// Can we jump directly to the jump_to_action?
					bool obstacle_found = false;
					UGVAction jump_to_action = ugv_action_list[jump_to_action_i];

					for(const Obstacle& obstacle : input_obstacles) {
						if(Obstacle::checkForObstacle(jump_to_action.fX, jump_to_action.fY, current_action.fX, current_action.fY, obstacle)) {
							// We will hit something.. break out of the loop
							obstacle_found = true;
							break;
						}
					}

					// Can we jump..?
					if(!obstacle_found) {
						if(DEBUG_SOLVER) {
							printf("*** Next is e_MoveToPosition (%d), this can be skipped\n", ugv_action_list[UGV_action_index+1].mActionID);
							printf(" ** Move forward to %d!\n", ugv_action_list[jump_to_action_i].mActionID);
						}

						// Yes! Move action index up so that we skip the extra moves
						UGV_action_index = jump_to_action_i - 1;
						updated_path = true;

						// Break out of the while-loop
						break;
					}
					else {
						// Nope.. Move back one action and try again
						jump_to_action_i--;
					}
				}
			}
		}
	}

	// Did we actually update the action list..?
	if(updated_path) {
		// Update our action list
		sol_current->swapUGVActionList(ugv_num, new_UGV_action_list);
	}

	return updated_path;
}



bool Solver::actionInObstacle(const UGVAction& action, const Obstacle& obstacle, double buffer) {
	double dist_a_o = distAtoB(action.fX, action.fY, obstacle.location.x, obstacle.location.y);
    return dist_a_o <= (obstacle.radius + buffer);
}

// Moves issueAction in the direction of the stepTowardsAction. Returns the ID of the last obstacle that we stepped out of.
int Solver::fixOverlappingActionOBS(UGVAction& issueAction, const DroneAction& stepTowardsAction, const std::vector<Obstacle>& input_obstacles) {
	// * We step from the issueAction action to the stepTowardsAction until we are not in any obstacles
	double dx = stepTowardsAction.fX - issueAction.fX;
	double dy = stepTowardsAction.fY - issueAction.fY;
	double distance = std::sqrt(dx * dx + dy * dy);
	if(distance == 0.0) {
		throw std::runtime_error("Cannot fix overlapping action — direction vector has zero length");
	}

//	double fixedX = issueAction.fX;
//	double fixedY = issueAction.fY;

	// * Create out unit vector
	double ux = dx / distance;
	double uy = dy / distance;

	// The last obstacle we crossed
	int last_obstacle = -1;
	for(const Obstacle& obstacle : input_obstacles) {
		// Still in an obstacle... move and try again
		if(actionInObstacle(issueAction, obstacle)) {
			last_obstacle = obstacle.get_id();
			break;
		}
	}

	// Is this actually an problem action..?
	if(last_obstacle == -1) {
		// No! Something is messed up... return -1
		fprintf(stderr, "[%s][Solver::fixOverlappingActionOBS] Asked to move action, but action is not in an obstacle\n", WARNING);
		return last_obstacle;
	}

	// Try to step out of obstacles
	bool isClear = true;
	for(int i = 0; i < OBS_MOVE_ITERATIONS; i++) {
		issueAction.fX += OBS_MOVE_STEP_SIZE * ux;
		issueAction.fY += OBS_MOVE_STEP_SIZE * uy;

		// Determine if we moved out of all obstacles
		isClear = true;
		for(const Obstacle& obstacle : input_obstacles) {
			// Still in an obstacle... move and try again
			if(actionInObstacle(issueAction, obstacle, OMPL_OBST_BUFFER_DIST + 2.0)) {
				isClear = false;
				last_obstacle = obstacle.get_id();
				break;
			}
		}

		// Did we clear all obstacles?
		if(isClear) {
			// Cleared obstacle!
			return last_obstacle;
		}
	}

	// If we made it this far... we never moved out of the obstacles...
	fprintf(stderr,"[%s][Solver::fixOverlappingActionOBS] Could not walk action out of obstacles\n", ERROR);
	throw std::runtime_error("Stuck in obstacle\n");


//	if(0) {
//		// * Stop when rounded coordinates match — "close enough" to consider on top of target
//		while (std::round(fixedX) != std::round(stepTowardsAction.fX) || std::round(fixedY) != std::round(stepTowardsAction.fY)) {
//			fixedX += OVERLAPPING_STEP_SIZE * ux;
//			fixedY += OVERLAPPING_STEP_SIZE * uy;
//
//			// * Test to see if our new action is inside any obstacles
//			UGVAction tempAction(issueAction.mActionType, fixedX, fixedY, issueAction.fCompletionTime, issueAction.mDetails);
//			bool isClear = true;
//			for (const Obstacle& obstacle : input_obstacles) {
//				if (isActionInsideObstacle(tempAction, obstacle)) {
//					isClear = false; // * We need to keep stepping
//					break;
//				}
//			}
//			if (isClear) return tempAction; // * We have stepped out of all obstacles
//		}
//
//		// * Test one last time
//		UGVAction tempAction(issueAction.mActionType, fixedX, fixedY, issueAction.fCompletionTime, issueAction.mDetails);
//		bool isClear = true;
//		for (const Obstacle& obstacle : input_obstacles) {
//			if (isActionInsideObstacle(tempAction, obstacle)) {
//				isClear = false; // * We need to keep stepping
//				break;
//			}
//		}
//
//		if (isClear) {
//			return tempAction;
//		} else {
//			std::cerr << "Could not step action out of obstacle -- solution is currently unsolveable" << std::endl;
//			std::cerr.flush();
//			throw std::runtime_error("Obstacle overlap");
//		}
//	}
}

// Attempts to update all sub-tours belong to drone drone_id. Returns true if one of the updates improved solution quality (that is, we changed the ordering of a sub-tour).
bool Solver::updateSubtours(int drone_id, Solution* sol_final) {
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

	if(DEBUG_SOLVER)
		printf(" Updating sub-tours for drone %d\n", drone_id);

	/// opt-flag := False
	bool opt_flag = false;

	// Start a new action list
	std::vector<DroneAction> new_action_list;
	std::vector<DroneAction> sub_tour_action_list;

	// Create a sub-tour vector
	std::vector<GeneralVertex> lst;
	double old_subtour_dist = 0.0;
	double prev_x=0.0, prev_y=0.0;
	DroneAction sub_tour_start(E_DroneActionTypes::e_LaunchFromUGV, prev_x, prev_y, 0.0);

	// Run through this drones action list...
	std::vector<DroneAction> old_action_list;
	sol_final->GetDroneActionList(drone_id, old_action_list);

	if(DEBUG_SOLVER)
		printf(" Running through actions\n");

	for(DroneAction a : old_action_list) {
		// Are we starting a new sub-tour?
		if(a.mActionType == E_DroneActionTypes::e_LaunchFromUGV) {

			// Clear old sub-tour
			lst.clear();
			sub_tour_action_list.clear();
			old_subtour_dist = 0.0;

			// Create a TSP vertex
			GeneralVertex depot;
			depot.nID = -1;
			depot.x = a.fX;
			depot.y = a.fY;
			lst.push_back(depot);

			// Record where we started the sub-tour
			sub_tour_start.mActionID = a.mActionID;
			sub_tour_start.mActionType = a.mActionType;
			sub_tour_start.fX = a.fX;
			sub_tour_start.fY = a.fY;
			sub_tour_start.fCompletionTime = a.fCompletionTime;
			sub_tour_start.mDetails = a.mDetails;
			prev_x = a.fX;
			prev_y = a.fY;
		}
		// Did we move to a node?
		else if(a.mActionType == E_DroneActionTypes::e_MoveToNode) {
			// Add node to current sub-tour
			GeneralVertex node;
			node.nID = a.mDetails;
			node.x = a.fX;
			node.y = a.fY;
			lst.push_back(node);
			sub_tour_action_list.push_back(a);

			// Update sub-tour distance
			old_subtour_dist += distAtoB(prev_x, prev_y, a.fX, a.fY);
			prev_x = a.fX;
			prev_y = a.fY;
		}
		// Is this the end of the tour?
		else if(a.mActionType == E_DroneActionTypes::e_MoveToUGV) {
			// Add terminal to current sub-tour
			GeneralVertex terminal;
			terminal.nID = -2;
			terminal.x = a.fX;
			terminal.y = a.fY;
			lst.push_back(terminal);

			// Update sub-tour distance
			old_subtour_dist += distAtoB(prev_x, prev_y, a.fX, a.fY);

			// Run TSP solver....
			std::vector<GeneralVertex> result;
			solverTSP_LKH(lst, result, 1000);

			// Determine the distance of this new tour
			double new_subtour_dist = 0;
			std::vector<GeneralVertex>::iterator nxt = result.begin()++;
			std::vector<GeneralVertex>::iterator prev = result.begin();
			while(nxt != result.end()) {
				// Get distance to next stop
				new_subtour_dist += distAtoB(prev->x, prev->y, nxt->x, nxt->y);
				// Update iterators
				prev = nxt;
				nxt++;
			}

			/// Start re-building tour
			// Add in original take-off
			new_action_list.push_back(sub_tour_start);

			// Did we find a shorter distance?
			if(new_subtour_dist < (old_subtour_dist-EPSILON)) {
				// Found better tour
				opt_flag = true;
				// Add in new sub-tour
				for(GeneralVertex v : result) {
					if(v.nID >= 0) {
						// Push action for this stop onto the drone
						DroneAction nodeStop(E_DroneActionTypes::e_MoveToNode, v.x, v.y, 0.0, v.nID);
						new_action_list.push_back(nodeStop);
					}
				}
			}
			else {
				// Just put the original tour back
				for(DroneAction node_act : sub_tour_action_list) {
					new_action_list.push_back(node_act);
				}
			}
			// Move to UGV
			new_action_list.push_back(a);
		}
		// Not on a sub-tour...
		else {
			// Just push this action back onto the list
			new_action_list.push_back(a);
		}
	}

	// Did we create a new tour?
	if(opt_flag) {
		// Clear old solution
		sol_final->ClearDroneSolution(drone_id);
		for(DroneAction a : new_action_list) {
			sol_final->PushDroneAction(drone_id, a);
		}
	}

	/// return opt-flag
	return opt_flag;
}

bool Solver::pushActionsOutside(int ugv_num, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV) {
	bool moved_something = false;

	// Get current action list
	std::vector<UGVAction> ugv_action_list;
	sol_current->GetUGVActionList(ugv_num, ugv_action_list);
	// Get another list (we will update this one)
	std::vector<UGVAction> temp_ugv_action_list;
	sol_current->GetUGVActionList(ugv_num, temp_ugv_action_list);
	// Get obstacles
	std::vector<Obstacle> input_obstacles = input->GetObstacles();

	// Get drones assigned to this UGV
	std::vector<int> drone_IDs = drones_to_UGV[ugv_num];
	// Mapping of drones to their action lists
	std::map<int, std::vector<DroneAction>> ugv_drone_action_lists;
	// Get the action lists
	for(size_t i = 0; i < drone_IDs.size(); ++i) {
		int droneId = drone_IDs[i];
		std::vector<DroneAction> temp_action_list;
		sol_current->GetDroneActionList(droneId, temp_action_list);
		ugv_drone_action_lists[droneId] = temp_action_list;
	}

	// For each UGV actions...
	for(size_t UGV_action_index = 0; UGV_action_index < ugv_action_list.size(); UGV_action_index++) {
		UGVAction curr_action = ugv_action_list[UGV_action_index];

		// Check that action isn't inside an obstacle...
		bool in_obstacle = false;
		for(const Obstacle& obstacle : input_obstacles) {
			// Is this action inside of an obstacle?
			if(actionInObstacle(curr_action, obstacle)) {
				if(DEBUG_SOLVER) {
					if(curr_action.mActionType == E_UGVActionTypes::e_LaunchDrone || curr_action.mActionType == E_UGVActionTypes::e_ReceiveDrone) {
						printf("We have found a obstacle overlaping with a action\n");
						printf("Action:\n");
						curr_action.print();
						printf("Obstacle:\n");
						obstacle.printInfo();
						printf("\n");
					}
				}
				in_obstacle = true;
				break;
			}
		}

		if(in_obstacle) {
			// What type of action is this?
			switch(curr_action.mActionType)
			{
				// * We do nothing, since we want to move the launch/land and will just recreate this action later
				case E_UGVActionTypes::e_MoveToWaypoint:
					break;
				// * For both of these we will need to move this action out of the obstacle and then create a new corresponding move to waypoint
				case E_UGVActionTypes::e_LaunchDrone:
					{
						// * We need to determine the drone action we want to move towards
						std::vector<DroneAction>& drone_actions = ugv_drone_action_lists.at(curr_action.mDetails);

						DroneAction* moveTowardsAction = nullptr;
						int swap_index = -1;
						for(size_t i = 0; i < drone_actions.size(); i++) {
							DroneAction& d_a = drone_actions[i];
							if(d_a.mActionType == E_DroneActionTypes::e_LaunchFromUGV &&
									isZero(d_a.fCompletionTime - curr_action.fCompletionTime)) {
								// * The action after the corresponding launch is the first move to waypoint
								// * This is the action we want to push towards
								moveTowardsAction = &drone_actions[i+1];

								// * Sanity Check
								if(moveTowardsAction->mActionType != E_DroneActionTypes::e_MoveToNode) {
									throw std::runtime_error("Action list is malformed, we are assuming to be moving toward a #2 action");
								}
								swap_index = i;

								break;
							}
						}

						if(moveTowardsAction == nullptr) {
							throw std::runtime_error("Matching DroneAction not found for UGV LaunchDrone action.");
						}

						if(DEBUG_SOLVER) {
							printf("Pushing the action toward this action:\n");
							moveTowardsAction->print();
							printf("\n");
						}

						int last_obstacle = fixOverlappingActionOBS(curr_action, *moveTowardsAction, input_obstacles);

						if(DEBUG_SOLVER) {
							printf("Here is our pushed action: \n");
							curr_action.print();
							printf("\n");
						}

						// * Now we create our new actions
						int uav_num = curr_action.mDetails;
						if(ugv_drone_action_lists.find(uav_num) != ugv_drone_action_lists.end()) { // * Double check to make sure we have a drone action list for the UAV #
							ugv_drone_action_lists[uav_num][swap_index].fX = curr_action.fX;
							ugv_drone_action_lists[uav_num][swap_index].fY = curr_action.fY;
							ugv_drone_action_lists[uav_num][swap_index].mObstacle = last_obstacle;
							if(DEBUG_SOLVER)
								printf("****** Launch pushed out of: %d\n", ugv_drone_action_lists[uav_num][swap_index].mObstacle);
						}
						else {
							// * Our lists doesn't exist in the mapping but it should so throw a error
							throw std::runtime_error("Drone list mapping error");
						}

						// Update this action's location
						temp_ugv_action_list[UGV_action_index].fX = curr_action.fX;
						temp_ugv_action_list[UGV_action_index].fY = curr_action.fY;
						// Update the previous action's location
						temp_ugv_action_list[UGV_action_index - 1].fX = curr_action.fX;
						temp_ugv_action_list[UGV_action_index - 1].fY = curr_action.fY;
						// Update the details of the previous action
						temp_ugv_action_list[UGV_action_index - 1].mDetails = last_obstacle;
					}
					moved_something = true;
					break;
				case E_UGVActionTypes::e_ReceiveDrone:
					{
						// * We need to determine the drone action we want to move towards
						std::vector<DroneAction>& drone_actions = ugv_drone_action_lists.at(curr_action.mDetails);

						DroneAction* moveTowardsAction = nullptr;
						int swap_index = -1;
						for(size_t i = 0; i < drone_actions.size(); i++) {
							DroneAction& d_a = drone_actions[i];
							if(d_a.mActionType == E_DroneActionTypes::e_LandOnUGV &&
									isZero(d_a.fCompletionTime - curr_action.fCompletionTime)) {
								// * The action 2 actions before the Land is the last waypoint
								// * This is the action we want to push towards
								moveTowardsAction = &drone_actions[i-2];

								// * Sanity Check
								if(moveTowardsAction->mActionType != E_DroneActionTypes::e_MoveToNode) {
									throw std::runtime_error("Action list is malformed, we are assuming to be moving toward a #2 action");
								}
								swap_index = i;

								break;
							}
						}

						if(moveTowardsAction == nullptr) {
							throw std::runtime_error("Matching DroneAction not found for UGV LaunchDrone action.");
						}

						if(DEBUG_SOLVER) {
							printf("Pushing the action toward this action:\n");
							moveTowardsAction->print();
							printf("\n");
						}

						int last_obstacle = fixOverlappingActionOBS(curr_action, *moveTowardsAction, input_obstacles);

						if(DEBUG_SOLVER) {
							printf("Here is our pushed action: \n");
							curr_action.print();
							printf("\n");
						}

						int uav_num = curr_action.mDetails;
						if(ugv_drone_action_lists.find(uav_num) != ugv_drone_action_lists.end()) { // * Double check to make sure we have a drone action list for the UAV #
							ugv_drone_action_lists[uav_num][swap_index - 1].fX = curr_action.fX;
							ugv_drone_action_lists[uav_num][swap_index - 1].fY = curr_action.fY;
							ugv_drone_action_lists[uav_num][swap_index].fX = curr_action.fX;
							ugv_drone_action_lists[uav_num][swap_index].fY = curr_action.fY;
							ugv_drone_action_lists[uav_num][swap_index].mObstacle = last_obstacle;
							if(DEBUG_SOLVER)
								printf("****** Land pushed out of: %d\n", ugv_drone_action_lists[uav_num][swap_index].mObstacle);
						}
						else {
							// * Our lists doesn't exist in the mapping but it should so throw a error
							throw std::runtime_error("Done List Mapping Error");
						}

						// Update the location of this action
						temp_ugv_action_list[UGV_action_index].fX = curr_action.fX;
						temp_ugv_action_list[UGV_action_index].fY = curr_action.fY;
						// Update the location of the previous action
						temp_ugv_action_list[UGV_action_index - 1].fX = curr_action.fX;
						temp_ugv_action_list[UGV_action_index - 1].fY = curr_action.fY;
						// Associate the previous move-to action with the obstacle
						temp_ugv_action_list[UGV_action_index - 1].mDetails = last_obstacle;
					}
					moved_something = true;
					break;
				default:
					fprintf(stderr, "[%s][Solver::pushActionsOutside] Unexpected action type (%d) in obstacle\n", WARNING, (int)curr_action.mActionType);
				break;
			}
		}
	}

	if(moved_something) {
		// * Swap our temp lists into the solution
		sol_current->swapUGVActionList(ugv_num, temp_ugv_action_list);
		for(const auto& pair : ugv_drone_action_lists) {
			int drone_ID = pair.first;
			const std::vector<DroneAction>& actionList = pair.second;

			if (drone_ID == -1) {
				continue;
			}
			sol_current->swapDroneActionLists(drone_ID, actionList);
		}

		if(DEBUG_SOLVER) {
			printf("\n");
			printf("Solution after the actions are pushed out of obstacles\n");
			sol_current->PrintSolution();
			printf("\n");
		}
	}

	return moved_something;
}

// Cycles through the obstacles (circles) and determines if this point is in at least on obstacle
bool Solver::pointInObstacle(double x, double y, const std::vector<Obstacle>& obstacles) {
	for(const Obstacle& obs : obstacles) {
		// Determine distance from a to the circle
		double dist = std::sqrt((x - obs.location.x)*(x - obs.location.x) + (y - obs.location.y)*(y - obs.location.y));
		if(dist <= obs.radius) {
			// Inside of this obstacle!
			return true;
		}
	}
	return false;
}

// Iteratively moves point (x,y) out of obstacles by pushing away from center of all obstacles that the point falls into
bool Solver::findClosestOutsidePointIterative(double* x, double* y, const std::vector<Obstacle>& obstacles) {
	for(int iter = 0; iter < OBS_MOVE_ITERATIONS; ++iter) {
		bool inside = false;
		double dir_x = 0.0, dir_y = 0.0;

		for(const Obstacle& obst : obstacles) {
			double dx = *x - obst.location.x;
			double dy = *y - obst.location.y;
			double dist = std::sqrt(dx * dx + dy * dy);

			if(dist < obst.radius) {
				inside = true;
				double delta = obst.radius - dist;

				if (dist > 1e-12) {
					dir_x += dx / dist * delta;
					dir_y += dy / dist * delta;
				} else {
					// Arbitrary direction if we're exactly at the center
					dir_x += delta;
				}
				if(DEBUG_SOLVER)
					printf("Inside of an obstacle!\n");
			}
		}

		if(!inside) {
			if(DEBUG_SOLVER)
				printf("Not inside any obstacle (%f,%f)\n", *x, *y);
			return true;
		}

		double mag = std::sqrt(dir_x * dir_x + dir_y * dir_y);
		if(mag < 1e-12) {
			if(DEBUG_SOLVER)
				printf(" degenerate case: push right\n");
			// degenerate case: push right
			dir_x = 1.0;
			dir_y = 0.0;
			mag = 1.0;
		}

		// Take a small step in the combined outward direction
		*x += dir_x / mag * OBS_MOVE_STEP_SIZE;
		*y += dir_y / mag * OBS_MOVE_STEP_SIZE;
		if(DEBUG_SOLVER)
			printf(" pushed point to: (%f, %f)\n", *x, *y);
	}

	return !pointInObstacle(*x, *y, obstacles);
}

/*
 * Determine which obstacle's boarder location B is closest to. If no obstacle is within some reasonable distance,
 * attempt to remove the point and verify that moving from A to C does not lead to a collision. Returns -1 if B
 * isn't reasonably close to anything and removing it does not lead to a collision.
 */
int Solver::determineAssociatedObstacle(GeneralVertex A, GeneralVertex B, GeneralVertex C, const std::vector<Obstacle>& obstacles) {
	/// Fist attempt to find closes obstacle
	// Assume no obstacle and 50 meters distance to the closest
	int closest_boarder = -1;
	double closest_dist = 50.0;
	// Cycle through all obstacles
	for(const Obstacle& obs : obstacles) {
		// Determine distance from p to center of the circle
		double dist = std::sqrt((B.x - obs.location.x)*(B.x - obs.location.x) + (B.y - obs.location.y)*(B.y - obs.location.y));
		// How close is p to the boarder?
		double boarder_dist = abs(dist - obs.radius);
		if(boarder_dist < closest_dist) {
			// Found a new closest obstacle
			closest_boarder = obs.get_id();
			closest_dist = boarder_dist;
		}
	}

	/// If the above didn't work...
	if(closest_boarder == -1) {
		// Try removing B
		for(const Obstacle& obstacle : obstacles) {
			// Would we hit this obstacle..?
			if(Obstacle::checkForObstacle(A.x, A.y, C.x, C.y, obstacle)) {
				// Yes, associate the waypoint with this obstacle
				closest_boarder = obstacle.get_id();
				// Stop checking obstacles
				break;
			}
		}
	}

	return closest_boarder;
}

void Solver::checkForRedundantMoves(PatrollingInput* input, int ugv_num, Solution* sol_current, const std::vector<Obstacle>& obstacles) {
	std::vector<UGVAction> list_to_check;
	sol_current->GetUGVActionList(ugv_num, list_to_check);

	std::vector<UGVAction> filtered_list;

	for (int i = 0; i < boost::numeric_cast<int>(list_to_check.size()); ++i) {
		const UGVAction& curr = list_to_check[i];

		// * Check if it's a MoveToPosition and if we can skip it
		if (curr.mActionType == E_UGVActionTypes::e_MoveToPosition &&
			i > 0 && i < static_cast<int>(list_to_check.size()) - 1)
		{
			const UGVAction& prev = list_to_check[i - 1];
			const UGVAction& next = list_to_check[i + 1];

			bool collision = false;
			for (const Obstacle& obs : obstacles) {
				if (Obstacle::checkForObstacle(prev.fX, prev.fY, next.fX, next.fY, obs)) {
					collision = true;
					break;
				}
			}

			if (!collision) {
				if (DEBUG_SOLVER) {
					printf("Redundant MoveToPosition removed:\n");
					curr.print();
				}
				continue; //* skip adding to filtered list
			}
		}

		filtered_list.push_back(curr); // * retain non-redundant actions
	}

	sol_current->swapUGVActionList(ugv_num, filtered_list);
}


void Solver::optimizeWithObstacles(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_current, std::vector<std::vector<int>>& drones_to_UGV) {
	//* Run the optimizer once to shake things up
	LaunchOptimizerOBS optimizerOBS;
	optimizerOBS.OptLaunching(ugv_num, drones_on_UGV, input, sol_current);

	if (DEBUG_SOLVER) {
		sol_current->PrintSolution();
	}

	 // * while we are finding collisions with obstacles
	while(moveAroundObstacles(ugv_num, input, sol_current, drones_to_UGV)) {
		if (DEBUG_SOLVER) {
			std::cout << "---------------------------" << std::endl;
			printf("Solution after attemping to move around obstacles\n");
			sol_current->PrintSolution();
			std::cout << "---------------------------" << std::endl;
		}

		optimizerOBS.OptLaunching(ugv_num, drones_on_UGV, input, sol_current);
		break;
	}
}
