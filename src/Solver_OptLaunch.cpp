#include "Solver_OptLaunch.h"

//using json = nlohmann::json;


OptLaunchSolver::OptLaunchSolver() {
	if(SANITY_PRINT)
		printf("Hello from Opt-Launch Solver!\n");
//	srand(time(NULL));
}

OptLaunchSolver::~OptLaunchSolver() { }


void OptLaunchSolver::Solve(PatrollingInput* input, Solution* sol_final) {
	if(SANITY_PRINT)
		printf("\nStarting Greedy solver\n\n");

//	try {
//		GRBEnv env = GRBEnv();
//		GRBModel model = GRBModel(env);
//
//		// Fixed variable
//		GRBVar base1_x = model.addVar(50.0, 50.1, 0.0, GRB_CONTINUOUS, "base1_x");
//		GRBVar base1_y = model.addVar(0.0, 0.1, 0.0, GRB_CONTINUOUS, "base1_y");
//		GRBVar base2_x = model.addVar(0.0, 0.1, 0.0, GRB_CONTINUOUS, "base2_x");
//		GRBVar base2_y = model.addVar(50.0, 50.1, 0.0, GRB_CONTINUOUS, "base2_y");
//		GRBVar waypoint1_x = model.addVar(150.0, 150.1, 0.0, GRB_CONTINUOUS, "waypoint1_x");
//		GRBVar waypoint1_y = model.addVar(50.0, 50.1, 0.0, GRB_CONTINUOUS, "waypoint1_y");
//		GRBVar waypointN_x = model.addVar(50.0, 50.1, 0.0, GRB_CONTINUOUS, "waypointN_x");
//		GRBVar waypointN_y = model.addVar(150.0, 150.1, 0.0, GRB_CONTINUOUS, "waypointN_y");
//
//		// Stop 1
//		GRBVar x1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x1");
//		GRBVar y1 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y1");
//		// Stop 2
//		GRBVar y2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y2");
//		GRBVar x2 = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x2");
//		// UGV legs
//		GRBVar z1 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z1");
//		GRBVar z2 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z2");
//		GRBVar z3 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z3");
//		// Drone lets
//		GRBVar d1 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d1");
//		GRBVar d2 = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d2");
//
//		// First UGV leg
//		model.addQConstr((x1-base1_x)*(x1-base1_x) + (y1-base1_y)*(y1-base1_y) <= z1*z1, "first-leg");
//		// Middle leg
//		model.addQConstr((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) <= z2*z2, "middle-leg");
//		// Last leg
//		model.addQConstr((x2-base2_x)*(x2-base2_x) + (y2-base2_y)*(y2-base2_y) <= z3*z3, "last-leg");
//		// Drone sub-tour start
//		model.addQConstr((x1-waypoint1_x)*(x1-waypoint1_x) + (y1-waypoint1_y)*(y1-waypoint1_y) <= d1*d1, "waypoint_1");
//		// Sub-tour end
//		model.addQConstr((waypointN_x-x2)*(waypointN_x-x2) + (waypointN_y-y2)*(waypointN_y-y2) <= d2*d2, "waypoint_N");
//		// Limited drone distance
//		model.addConstr(d1 + d2 <= 300, "d_max");
//
//		// Set objective
//		GRBLinExpr obj = z1+z2+z3;
//		model.setObjective(obj, GRB_MINIMIZE);
//
//		// Optimize model
//		model.optimize();
//
//		printf("%s = %f\n", x1.get(GRB_StringAttr_VarName).c_str(), x1.get(GRB_DoubleAttr_X));
//		printf("%s = %f\n", y1.get(GRB_StringAttr_VarName).c_str(), y1.get(GRB_DoubleAttr_X));
//		printf("%s = %f\n", x2.get(GRB_StringAttr_VarName).c_str(), x2.get(GRB_DoubleAttr_X));
//		printf("%s = %f\n", y2.get(GRB_StringAttr_VarName).c_str(), y2.get(GRB_DoubleAttr_X));
//		printf("Obj: %f\n", model.get(GRB_DoubleAttr_ObjVal));
//
//	} catch(GRBException e) {
//		printf("[ERROR] %d: %s\n", e.getErrorCode(), e.getMessage().c_str());
//	} catch(const std::exception& e) {
//		printf("Exception during optimization: %s\n", e.what());
//	}

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
	 * Current Agnostic Algorithm:
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
	if(DEBUG_OPTLAUNCH) {
		printf("UGVs-to-Drones:\n");
		for(int j_g = 0; j_g < input->GetMg(); j_g++) {
			printf(" UGV %d:\n  ", j_g);
			for(int n : drones_to_UGV.at(j_g)) {
				printf("%d ", n);
			}
			printf("\n");
		}
	}

	// Used for clustering
	std::vector<ClusteringPoint> vctrPoIPoint;
	// Each depot has a set of PoI, an ordering for a UGV to visit, and a set of sub-tours
	std::vector<VRPPoint> depots;
	std::vector<std::vector<int>> depot_order;
	// depot < subtour < PoI > >
	std::vector<std::vector<std::vector<int>>> depots_tours;

	if(DEBUG_OPTLAUNCH)
		printf("Input PoI Nodes:\n");
	for(long unsigned int i = 0; i < vctrPOINodes.size(); i++) {
		if(DEBUG_OPTLAUNCH)
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
		if(DEBUG_OPTLAUNCH) {
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
				if(DEBUG_OPTLAUNCH) {
					printf("Solving VRP on cluster %d, depot at (%f, %f)\n", k, depot.x, depot.y);
				}


				/// 6. Solve VRP on cluster with |D_j| vehicles
				std::vector<std::vector<int>> subtours;
				mVRPSolver.SolveVRP(nodes, boost::numeric_cast<int>(drones_to_UGV.at(j_g).size()), subtours);
				depots_tours.push_back(subtours);

				if(DEBUG_OPTLAUNCH)
					printf("VRP solver finished, checking tours\n");

				// Calculate the longest sub-tour distance
				for(const std::vector<int> subtour : subtours) {
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

					if(DEBUG_OPTLAUNCH)
						printf(" Distance of tour: %f\n", tour_length);


					/// 7. If there exists drone-tour A such that dist(A) > d^a_max, increase k and repeat steps 2-8
					if(tour_length > UAV_MAX_D) {
						if(DEBUG_OPTLAUNCH)
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
			if(DEBUG_OPTLAUNCH) {
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

			if(DEBUG_OPTLAUNCH)
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
					double totalFlyTime = 0.0;
					double prev_x = depots.at(depot_order.at(ugv_num).back()).x;
					double prev_y = depots.at(depot_order.at(ugv_num).back()).y;
					// For each PoI in drone tour
					for(int stop : depots_tours.at(depot_order.at(ugv_num).back()).at(drone_i)) {
						// Determine time required to move to node
						double dist_prev_next = distAtoB(prev_x, prev_y, vctrPOINodes.at(stop).location.x, vctrPOINodes.at(stop).location.y);
						totalFlyTime += dist_prev_next/UAV_V_MAX;
						// Update for next stop
						prev_x = vctrPOINodes.at(stop).location.x;
						prev_y = vctrPOINodes.at(stop).location.y;
					}
					// Move back to UGV
					double dist_to_ugv = distAtoB(prev_x, prev_y, depots.at(depot_order.at(ugv_num).back()).x, depots.at(depot_order.at(ugv_num).back()).y);
					totalFlyTime += dist_to_ugv/UAV_V_MAX;
					// Energy from flying
					double joules = totalFlyTime*DRONE_JOULES_PER_SECONDS;
					// Add in energy required to launch and to land
					joules += LAUNCH_ENERGY + LAND_ENERGY;
					// Determine how long the drone must charge
					double finalChargeTime = calcChargeTime(joules);

					// Determine time required to move to base
					double dist_to_base = distAtoB(depots.at(depot_order.at(ugv_num).back()).x, depots.at(depot_order.at(ugv_num).back()).y, depots.back().x, depots.back().y);
					double time_to_base = dist_to_base/UGV_V_CRG;
					// How long has the drone been charging by the end of the kernel?
					double kernel_end_time = time_to_base + UGV_BAT_SWAP_TIME;
					double charge_time_at_start = std::max(finalChargeTime - kernel_end_time, 0.0);
					chargeTimes.at(drone) = charge_time_at_start;

					if(DEBUG_OPTLAUNCH)
						printf("Drone %d, requires %f charging at start\n", drone, charge_time_at_start);
				}
			}

			// Used to track when the kernel ends
			double kernel_complete_time;

			// For each UGV
			for(int ugv_num = 0; ugv_num < input->GetMg() && valid_solution; ugv_num++) {
				if(DEBUG_OPTLAUNCH)
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
						double t_duration = dist_prev_next/UGV_V_CRG;
						ugv_arrival_time = ugv_last.fCompletionTime + t_duration;
						// Create action for moving here
						UGVAction moveToDepot(E_UGVActionTypes::e_MoveToWaypoint, depots.at(n).x, depots.at(n).y, ugv_arrival_time, n);
						sol_final->PushUGVAction(ugv_num, moveToDepot);
						// Calculate how much energy we used
						double moveEnergy = UGV_JOULES_PER_SECONDS_DRIVE*t_duration;
						ugvEnergySpent.at(ugv_num) += moveEnergy;

						if(DEBUG_OPTLAUNCH)
							printf("  Arrive at depot %d at %f, energy to move: %f\n", n, ugv_arrival_time, moveEnergy);
					}

					std::priority_queue<Arrival, std::vector<Arrival>, CompareArrival> arrival_queue;
					// For each drone that launches from the UGV
					for(long unsigned int drone_i = 0; drone_i < depots_tours.at(n).size(); drone_i++) {
						// Which drone gets launched next?
						Arrival next_drone = charging_queue.top();
						charging_queue.pop();
						int drone = next_drone.ID;

						double totalFlyTime = 0.0;
						// Launch the drone
						{
							// Determine what time to launch the drone
							UGVAction ugv_last = sol_final->GetLastUGVAction(ugv_num);
							DroneAction drone_last = sol_final->GetLastDroneAction(drone);
							double time_to_charge = chargeTimes.at(drone);
							double done_charging = drone_last.fCompletionTime+time_to_charge;
							double launch_start_time = std::max(ugv_last.fCompletionTime, done_charging);
							double completion_time = launch_start_time + UAV_LAUNCH_TIME;

							// Launch the drone (needs actions for both the drone and the UGV)
							DroneAction launchFromUGV(E_DroneActionTypes::e_LaunchFromUGV, ugv_last.fX, ugv_last.fY, completion_time, ugv_num);
							sol_final->PushDroneAction(drone, launchFromUGV);
							UGVAction launchDrone(E_UGVActionTypes::e_LaunchDrone, ugv_last.fX, ugv_last.fY, completion_time, drone);
							sol_final->PushUGVAction(ugv_num, launchDrone);

							if(DEBUG_OPTLAUNCH)
								printf("   Drone %d, done charge at %f, launch at %f\n", drone, done_charging, launch_start_time);
						}
						// For each PoI in drone tour
						for(int stop : depots_tours.at(n).at(drone_i)) {
							// Have the drone visit each node
							DroneAction drone_last = sol_final->GetLastDroneAction(drone);
							// Determine time required to move to node
							double dist_prev_next = distAtoB(drone_last.fX, drone_last.fY, vctrPOINodes.at(stop).location.x, vctrPOINodes.at(stop).location.y);
							double t_duration = dist_prev_next/UAV_V_MAX;
							double visit_time = drone_last.fCompletionTime + t_duration;
							totalFlyTime += t_duration;
							DroneAction moveToNode(E_DroneActionTypes::e_MoveToNode, vctrPOINodes.at(stop).location.x,
									vctrPOINodes.at(stop).location.y, visit_time, stop);
							sol_final->PushDroneAction(drone, moveToNode);

							if(DEBUG_OPTLAUNCH)
								printf("    Visit PoI %d at %f\n", stop, visit_time);
						}
						// Move back to UGV
						{
							DroneAction drone_last = sol_final->GetLastDroneAction(drone);
							UGVAction ugv_last = sol_final->GetLastUGVAction(ugv_num);
							// Determine time required to move to node
							double dist_prev_next = distAtoB(drone_last.fX, drone_last.fY, ugv_last.fX, ugv_last.fY);
							double t_duration = dist_prev_next/UAV_V_MAX;
							totalFlyTime += t_duration;
							double arrival_time = drone_last.fCompletionTime + t_duration;
							DroneAction moveToUGV(E_DroneActionTypes::e_MoveToUGV, ugv_last.fX, ugv_last.fY, arrival_time, ugv_num);
							// Push action
							sol_final->PushDroneAction(drone, moveToUGV);
							// Add arrival time to priority queue
							arrival_queue.emplace(arrival_time, drone);

							if(DEBUG_OPTLAUNCH)
								printf("    Arrive at UGV %d at %f\n", ugv_num, arrival_time);
						}

						// Determine how many joules of energy each drone has consumed
						{
							// Energy from launching and landing
							double joules = totalFlyTime*DRONE_JOULES_PER_SECONDS;
							// Add in energy required to launch and to land
							joules += LAUNCH_ENERGY + LAND_ENERGY;
							// Determine how long the drone must charge
							double chargeTime = calcChargeTime(joules);
							chargeTimes.at(drone) = chargeTime;
							// Record how much the UGV has given away
							ugvEnergySpent.at(ugv_num) += joules;

							if(DEBUG_OPTLAUNCH)
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
							double landOnArrival = next_arrival.time + UAV_LAND_TIME;
							double landWhenReady = ugv_last.fCompletionTime + UAV_LAND_TIME;
							double landTime = std::max(landOnArrival, landWhenReady);
							// Land the drone
							DroneAction landOnUGV(E_DroneActionTypes::e_LandOnUGV, ugv_last.fX, ugv_last.fY, landTime, ugv_num);
							sol_final->PushDroneAction(next_arrival.ID, landOnUGV);
							UGVAction landDrone(E_UGVActionTypes::e_ReceiveDrone, ugv_last.fX, ugv_last.fY, landTime, next_arrival.ID);
							sol_final->PushUGVAction(ugv_num, landDrone);
							// If needed, update when the UGV moves on
							ugv_exit_time = std::max(ugv_exit_time, landTime);

							if(DEBUG_OPTLAUNCH)
								printf("   Drone %d arrived at %f, lands at %f\n", next_arrival.ID, next_arrival.time, landTime);
						}
						// Pop arrival data off of priority queue
						arrival_queue.pop();
					}

					// Calculate how much energy we used while waiting
					double ugv_wait_time = ugv_exit_time - ugv_arrival_time;
					double ugv_wait_energy = UGV_JOULES_PER_SECONDS_WAIT*ugv_wait_time;
					ugvEnergySpent.at(ugv_num) += ugv_wait_energy;
					if(DEBUG_OPTLAUNCH)
						printf("   UGV energy waiting: %f\n", ugv_wait_energy);
				}

				// Return UGV to base
				{
					// Get last action
					UGVAction ugv_last = sol_final->GetLastUGVAction(ugv_num);
					// Determine time required to move to base
					double dist_prev_next = distAtoB(ugv_last.fX, ugv_last.fY, depots.back().x, depots.back().y);
					double t_duration = dist_prev_next/UGV_V_CRG;
					double arrive_time = ugv_last.fCompletionTime + t_duration;
					// Create action for moving here
					UGVAction moveToDepot(E_UGVActionTypes::e_MoveToDepot, depots.back().x, depots.back().y, arrive_time);
					sol_final->PushUGVAction(ugv_num, moveToDepot);

					// Calculate how much energy we used
					double energy_to_return = UGV_JOULES_PER_SECONDS_DRIVE*t_duration;
					ugvEnergySpent.at(ugv_num) += energy_to_return;

					// Swap UGV battery
					kernel_complete_time = arrive_time+UGV_BAT_SWAP_TIME;
					UGVAction ugvChargeAction(E_UGVActionTypes::e_AtDepot, depots.back().x, depots.back().y, kernel_complete_time);
					sol_final->PushUGVAction(ugv_num, ugvChargeAction);

					if(DEBUG_OPTLAUNCH) {
						printf("   Arrive at base at %f, energy to move: %f\n", arrive_time, energy_to_return);
						printf("  Total UGV energy used: %f, kernel end time: %f\n", ugvEnergySpent.at(ugv_num), kernel_complete_time);
					}


					// 9. If there exists UGV-tour A such that energy(A) > E^g_max, increase k and repeat steps 2-9
					if(ugvEnergySpent.at(ugv_num) > UGV_TOTAL_BAT) {
						if(DEBUG_OPTLAUNCH) {
							printf("\nUGV %d will run out of energy!\nIncrease k, start over...\n", ugv_num);
						}

						K++;
						valid_solution = false;
					}
					else {
						// Add in end-of-kernel action for the UGV and each of its drones
						UGVAction ugvEndAction(E_UGVActionTypes::e_KernelEnd, depots.back().x, depots.back().y, kernel_complete_time);
						sol_final->PushUGVAction(ugv_num, ugvChargeAction);
						for(int drone : drones_to_UGV.at(ugv_num)) {
							DroneAction endAction(E_DroneActionTypes::e_KernelEnd, depots.back().x, depots.back().y, kernel_complete_time);
							sol_final->PushDroneAction(drone, endAction);
						}
					}
				}
			}
		}
	}



	if(DEBUG_OPTLAUNCH) {
		printf("\nFinal Solution:\n");
		sol_final->PrintSolution();
		printf("\n");
	}


	// For each UGV...
	for(int ugv_num = 0; ugv_num < input->GetMg(); ugv_num++) {
		if(DEBUG_OPTLAUNCH)
			printf("Optimizing UGV %d route\n", ugv_num);

		try {
			GRBEnv env = GRBEnv();
			GRBModel model = GRBModel(env);
			std::vector<std::vector<GRBVar>> sub_tour_dist_vars;
			std::vector<std::vector<GRBVar>> sub_tour_pos_vars;

			// Queue for actions
			std::priority_queue<SOCAction, std::vector<SOCAction>, CompareSOCAction> action_queue;
			// Create array of drone sub-tours
			std::vector<SubTour> sub_tours;
			int subtour_counter = 0;
			// For each drone assigned to the UGV
			for(int drone_id : drones_to_UGV.at(ugv_num)) {
				if(DEBUG_OPTLAUNCH)
					printf(" Get actions for drone %d\n", drone_id);
				std::vector<DroneAction> action_list;
				sol_final->GetDroneActionList(drone_id, action_list);
				bool on_tour = false;
				double tour_dist = 0.0;
				double strt_x = 0.0, strt_y = 0.0;
				double prev_x = 0.0, prev_y = 0.0;
				// Cycle through the list of actions
				for(DroneAction act : action_list) {
					// Are we on tour..?
					if(on_tour) {
						// Are we still visiting nodes...?
						if(act.mActionType == E_DroneActionTypes::e_MoveToNode) {
							// Visited next node, record distance and update previous
							tour_dist += distAtoB(prev_x, prev_y, act.fX, act.fY);
							prev_x = act.fX;
							prev_y = act.fY;
						}
						else {
							// Not visiting a node... tour must have ended
							on_tour = false;

							// Record this sub-tour
							SubTour sub_tour(tour_dist, subtour_counter, strt_x, strt_y, prev_x, prev_y);
							sub_tours.push_back(sub_tour);

							// Create variables for the start/end distance of each sub-tour
							std::vector<GRBVar> start_end_dist_vars;
							GRBVar d_s = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d_s_"+itos(subtour_counter));
							GRBVar d_e = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d_e_"+itos(subtour_counter));
							start_end_dist_vars.push_back(d_s);
							start_end_dist_vars.push_back(d_e);
							sub_tour_dist_vars.push_back(start_end_dist_vars);

							// Create variables for the first/last waypoint of each sub-tour
							std::vector<GRBVar> start_end_pos_vars;
							GRBVar x_s = model.addVar(CONST_RELAXATION(strt_x), 0.0, GRB_CONTINUOUS, "x_s_"+itos(subtour_counter));
							GRBVar y_s = model.addVar(CONST_RELAXATION(strt_y), 0.0, GRB_CONTINUOUS, "y_s_"+itos(subtour_counter));
							GRBVar x_e = model.addVar(CONST_RELAXATION(prev_x), 0.0, GRB_CONTINUOUS, "x_e_"+itos(subtour_counter));
							GRBVar y_e = model.addVar(CONST_RELAXATION(prev_y), 0.0, GRB_CONTINUOUS, "x_e_"+itos(subtour_counter));
							start_end_pos_vars.push_back(x_s);
							start_end_pos_vars.push_back(y_s);
							start_end_pos_vars.push_back(x_e);
							start_end_pos_vars.push_back(y_e);
							sub_tour_pos_vars.push_back(start_end_pos_vars);

							tour_dist = 0.0;
						}
					}
					// Is this a tour stop? (i.e. we just finished the first leg)
					else if(act.mActionType == E_DroneActionTypes::e_MoveToNode) {
						on_tour = true;
						strt_x = prev_x = act.fX;
						strt_y = prev_y = act.fY;
					}

					// Is this launch command?
					if(act.mActionType == E_DroneActionTypes::e_LaunchFromUGV) {
						SOCAction action(act.fCompletionTime, drone_id, E_SOCActionType::e_LaunchDrone, subtour_counter);
						action_queue.push(action);
					}
					// Is this land command?
					if(act.mActionType == E_DroneActionTypes::e_LandOnUGV) {
						SOCAction action(act.fCompletionTime, drone_id, E_SOCActionType::e_ReceiveDrone, subtour_counter);
						action_queue.push(action);
						// We completed a subtour
						subtour_counter++;
					}
				}
			}

			// For my sanity
			if(DEBUG_OPTLAUNCH) {
				printf("\n** Pre-Solve **\n");
			}

			// Create x,y coordinate variables for each action in the list
			std::vector<std::vector<GRBVar>> action_coord_vars;
			std::vector<GRBVar> action_time_vars;
			int action_cout = 0;

			// Create initial fixed variable for the base
			{
				// Create an x,y coordinate
				std::vector<GRBVar> coord;
				double x_b, y_b;
				input->GetDepot(ugv_num, &x_b, &y_b);

				GRBVar x = model.addVar(CONST_RELAXATION(x_b), 0.0, GRB_CONTINUOUS, "x_"+itos(action_cout));
				GRBVar y = model.addVar(CONST_RELAXATION(y_b), 0.0, GRB_CONTINUOUS, "y_"+itos(action_cout));
				coord.push_back(x);
				coord.push_back(y);
				action_coord_vars.push_back(coord);

				// Create an initial action time variable for this action
				GRBVar t_i = model.addVar(CONST_RELAXATION(0.0), 0.0, GRB_CONTINUOUS, "t_"+itos(action_cout));
				action_time_vars.push_back(t_i);

				action_cout++;
			}

			if(DEBUG_OPTLAUNCH)
				printf("Action Queue:\n");

			// We need to track what the previous/first actions are for all drones to constrain charging time
			std::vector<int> prev_action_id;
			std::vector<int> first_action_id;
			for(int drone_j = 0; drone_j < input->GetMa(); drone_j++) {
				prev_action_id.push_back(-1);
				first_action_id.push_back(-1);
			}

			// For each launch/receive action...
			while(!action_queue.empty()) {
				// Get the next action
				SOCAction action = action_queue.top();
				action_queue.pop();
				// Debugging
				if(DEBUG_OPTLAUNCH)
					printf(" [%d] %d - %f\n", action.action_type, action.subtour_index, action.time);

				// Create an x,y coordinate
				std::vector<GRBVar> coord;
				GRBVar x_i = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x_"+itos(action_cout));
				GRBVar y_i = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_"+itos(action_cout));
				coord.push_back(x_i);
				coord.push_back(y_i);
				action_coord_vars.push_back(coord);

				// Create an end-time variable for this action
				GRBVar t_i = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "t_"+itos(action_cout));

				// Is this a launch/receive action?
				if(action.action_type == E_SOCActionType::e_LaunchDrone) {
					// Is this the second launch?
					if(prev_action_id.at(action.ID) >= 0) {
						// Require that the start time comes after charging is completed
						GRBVar t_j = action_time_vars.at(prev_action_id.at(action.ID));
						model.addConstr(t_i >= t_j + T_MAX, "t_"+itos(action_cout)+"_geq_t_"+itos(prev_action_id.at(action.ID)));
					}
					// Must be the first launch
					else {
						first_action_id.at(action.ID) = action_cout;
					}
					sub_tours.at(action.subtour_index).launch_ID = action_cout;
				}
				// Are we receiving the drone?
				else if(action.action_type == E_SOCActionType::e_ReceiveDrone) {
					// Record this action so we can refer to it later
					prev_action_id.at(action.ID) = action_cout;
					sub_tours.at(action.subtour_index).land_ID = action_cout;
				}

				action_time_vars.push_back(t_i);
				action_cout++;
			}

			// Create final fixed variable for the base
			{
				// Create an x,y coordinate
				std::vector<GRBVar> coord;
				double x_b, y_b;
				input->GetDepot(ugv_num, &x_b, &y_b);

				GRBVar x = model.addVar(CONST_RELAXATION(x_b), 0.0, GRB_CONTINUOUS, "x_"+itos(action_cout));
				GRBVar y = model.addVar(CONST_RELAXATION(y_b), 0.0, GRB_CONTINUOUS, "y_"+itos(action_cout));
				coord.push_back(x);
				coord.push_back(y);
				action_coord_vars.push_back(coord);
			}

			// Create an end-time variable for the final action (this is the objective)
			GRBVar t_base = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "t_"+itos(action_cout));
			action_time_vars.push_back(t_base);
			action_cout++;

			// Constrain the fist launch time of each drone
			for(int drone_id : drones_to_UGV.at(ugv_num)) {
				if(prev_action_id.at(drone_id) != first_action_id.at(drone_id)) {
					// Make sure the first action comes after the fully recharging from the last action
					GRBVar t_1 = action_time_vars.at(first_action_id.at(drone_id));
					GRBVar t_n = action_time_vars.at(prev_action_id.at(drone_id));
					model.addConstr(t_1 >= T_MAX - t_base + t_n, "t_f"+itos(drone_id)+"_geq_t_l"+itos(drone_id));
				}
			}

			// Create distance variables from one action location to the next
			std::vector<GRBVar> dist_vars;
			auto coord_i = action_coord_vars.begin();
			auto coord_j = coord_i+1;
			int dist_cout = 0;
			// Add a variable for each consecutive action pair
			while(coord_j != action_coord_vars.end()) {
				GRBVar d_i = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d_"+itos(dist_cout)+"_"+itos(dist_cout+1));
				dist_vars.push_back(d_i);

				// Add a constraint forcing the distance to be less than or equal to the distance from coord_i->coord_j
				GRBVar x_i = coord_i->at(0);
				GRBVar y_i = coord_i->at(1);
				GRBVar x_j = coord_j->at(0);
				GRBVar y_j = coord_j->at(1);
				model.addQConstr(d_i*d_i >= (x_i-x_j)*(x_i-x_j) + (y_i-y_j)*(y_i-y_j), "set_d_"+itos(dist_cout)+"_"+itos(dist_cout+1));

				// Create UGV travel time constraints
				// TODO: add in the time required for this action
				GRBVar t_i = action_time_vars.at(dist_cout);
				GRBVar t_j = action_time_vars.at(dist_cout+1);
				model.addConstr(t_j >= t_i + UGV_V_CRG*d_i, "t_"+itos(dist_cout+1)+"_geq_t_"+itos(dist_cout));

				coord_i = coord_j;
				coord_j++;
				dist_cout++;
			}

			// Create sub-tour start/end distance constraints
			if(DEBUG_OPTLAUNCH) {
				printf("|sub_tours| = %ld, |sub_tour_dist_vars| = %ld, |action_coord_vars| = %ld, |action_time_vars| = %ld\n", sub_tours.size(), sub_tour_dist_vars.size(), action_coord_vars.size(), action_time_vars.size());
				printf("Sub-Tours:\n");
			}
			for(int tour = 0; tour < boost::numeric_cast<int>(sub_tours.size()); tour++) {
				if(DEBUG_OPTLAUNCH)
					printf(" %d: (%d)-(%f,%f)- %f ->(%f,%f)-(%d)\n", sub_tours.at(tour).ID, sub_tours.at(tour).launch_ID, sub_tours.at(tour).start_x, sub_tours.at(tour).start_y, sub_tours.at(tour).end_x, sub_tours.at(tour).end_y, sub_tours.at(tour).tour_dist, sub_tours.at(tour).land_ID);

				// Add constraint on sub-tour end time
				GRBVar t_i = action_time_vars.at(sub_tours.at(tour).launch_ID);
				GRBVar t_j = action_time_vars.at(sub_tours.at(tour).land_ID);
				GRBVar d_s = sub_tour_dist_vars.at(tour).at(0);
				GRBVar d_e = sub_tour_dist_vars.at(tour).at(1);
				// TODO: add in launch/land times!
				model.addConstr(t_j == t_i + d_s/UAV_V_MAX + sub_tours.at(tour).tour_dist + d_e/UAV_V_MAX, "t_"+itos(sub_tours.at(tour).launch_ID)+"_geq_t_"+itos(sub_tours.at(tour).land_ID)+"_+_td");

				// Add constraint on sub-tour start/end leg distances
				auto coord_i = action_coord_vars.at(sub_tours.at(tour).launch_ID);
				GRBVar x_i = coord_i.at(0);
				GRBVar y_i = coord_i.at(1);
				GRBVar start_x = sub_tour_pos_vars.at(tour).at(0);
				GRBVar start_y = sub_tour_pos_vars.at(tour).at(1);
				GRBVar end_x = sub_tour_pos_vars.at(tour).at(2);
				GRBVar end_y = sub_tour_pos_vars.at(tour).at(3);
				model.addQConstr(d_s*d_s >= (x_i-start_x)*(x_i-start_x) + (y_i-start_y)*(y_i-start_y), "d_s"+itos(sub_tours.at(tour).ID)+"_geq_xi_x1");
				auto coord_j = action_coord_vars.at(sub_tours.at(tour).land_ID);
				GRBVar x_j = coord_j.at(0);
				GRBVar y_j = coord_j.at(1);
				model.addQConstr(d_e*d_e >= (x_j-end_x)*(x_j-end_x) + (y_j-end_y)*(y_j-end_y), "d_e"+itos(sub_tours.at(tour).ID)+"_geq_xm_xj");
			}

			// Set objective
			GRBLinExpr obj = t_base;
			model.setObjective(obj, GRB_MINIMIZE);

			if(DEBUG_OPTLAUNCH)
				printf("Run Gurobi\n");

			// Optimize model
			model.optimize();

//			printf("%s = %f\n", x.get(GRB_StringAttr_VarName).c_str(), x.get(GRB_DoubleAttr_X));
//			printf("%s = %f\n", y.get(GRB_StringAttr_VarName).c_str(), y.get(GRB_DoubleAttr_X));
//			printf("%s = %f\n", z.get(GRB_StringAttr_VarName).c_str(), z.get(GRB_DoubleAttr_X));
//			printf("Obj: %f\n", model.get(GRB_DoubleAttr_ObjVal));

		} catch(GRBException e) {
			printf("[ERROR] %d: %s\n", e.getErrorCode(), e.getMessage().c_str());
		} catch(const std::exception& e) {
			printf("Exception during optimization: %s\n", e.what());
		}
	}
}
