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

	// Run the baseline solver to get an initial solution
	RunBaseline(input, sol_final, drones_to_UGV);


	if(DEBUG_OPTLAUNCH) {
		printf("\nFinal Solution:\n");
		sol_final->PrintSolution();
		printf("\n");
		// Record this so we can watch how well the optimizer is improving things
		FILE * pOutputFile;
		pOutputFile = fopen("qp_improvement.dat", "a");
		fprintf(pOutputFile, "%d %f ", input->GetN(), sol_final->GetTotalTourTime(0));
		fclose(pOutputFile);
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

							if(DEBUG_OPTLAUNCH)
								printf("   Stop at node %d (%f,%f) total distance = %f\n", act.mDetails, prev_x, prev_y, tour_dist);
						}
						else {
							// Not visiting a node... tour must have ended
							on_tour = false;

							if(DEBUG_OPTLAUNCH)
								printf("  Tour %d complete, dist = %f, first node: (%f,%f), last node: (%f,%f)\n", subtour_counter, tour_dist, strt_x, strt_y, prev_x, prev_y);

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
						if(DEBUG_OPTLAUNCH)
							printf("   First node %d (%f,%f)\n", act.mDetails, strt_x, strt_y);
					}

					// Is this launch command?
					if(act.mActionType == E_DroneActionTypes::e_LaunchFromUGV) {
						SOCAction action(act.fCompletionTime, drone_id, E_SOCActionType::e_LaunchDrone, subtour_counter);
						action_queue.push(action);
						if(DEBUG_OPTLAUNCH)
							printf("  Starting tour %d\n", subtour_counter);
					}
					// Is this land command?
					if(act.mActionType == E_DroneActionTypes::e_LandOnUGV) {
						SOCAction action(act.fCompletionTime, drone_id, E_SOCActionType::e_ReceiveDrone, subtour_counter);
						action_queue.push(action);
						if(DEBUG_OPTLAUNCH)
							printf("  Finished tour %d\n", subtour_counter);
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
			std::vector<SOCAction> ordered_action_list;

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
				if(DEBUG_OPTLAUNCH)
					printf(" a_%d [%d]%d - %f\n", action_cout, E_SOCActionType::e_BaseStation, -1, 0.0);

				action_cout++;

				SOCAction initAction(0.0, -1, E_SOCActionType::e_BaseStation, -1);
				ordered_action_list.push_back(initAction);
			}

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
				ordered_action_list.push_back(action);

				// Debugging
				if(DEBUG_OPTLAUNCH)
					printf(" a_%d [%d]%d - %f drone: %d\n", action_cout, action.action_type, action.subtour_index, action.time, action.ID);

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
						if(DEBUG_OPTLAUNCH)
							printf("* t_%d >= t_%d + %f\n", action_cout, prev_action_id.at(action.ID), T_MAX);
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

				SOCAction lastAction(0.0, -1, E_SOCActionType::e_BaseStation, -1);
				ordered_action_list.push_back(lastAction);
			}

			// Create an end-time variable for the final action (this is the objective)
			GRBVar t_base = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "t_"+itos(action_cout));
			action_time_vars.push_back(t_base);
			if(DEBUG_OPTLAUNCH)
				printf(" a_%d [%d]%d - %f\n", action_cout, E_SOCActionType::e_BaseStation, -1, 0.0);

			// Constrain the fist launch time of each drone
			for(int drone_id : drones_to_UGV.at(ugv_num)) {
				if(prev_action_id.at(drone_id) != first_action_id.at(drone_id)) {
					// Make sure the first action comes after the fully recharging from the last action
					GRBVar t_1 = action_time_vars.at(first_action_id.at(drone_id));
					GRBVar t_n = action_time_vars.at(prev_action_id.at(drone_id));
					model.addConstr(t_1 >= T_MAX - (UGV_BAT_SWAP_TIME + t_base - t_n), "t_f"+itos(drone_id)+"_geq_t_l"+itos(drone_id));
					if(DEBUG_OPTLAUNCH)
						printf("* t_%d >= %f - t_%d + t_%d\n", first_action_id.at(drone_id), T_MAX, action_cout, prev_action_id.at(drone_id));
				}
			}

			// Create distance variables from one action location to the next
			std::vector<GRBVar> dist_vars;
			int dist_cout = 0;
			// Add a variable for each consecutive action pair
			for(int i = 0, j = 1; j < boost::numeric_cast<int>(action_coord_vars.size()); j++, i++) {
				auto coord_i = action_coord_vars.at(i);
				auto coord_j = action_coord_vars.at(j);
				GRBVar d_j = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "d_"+itos(dist_cout)+"_"+itos(dist_cout+1));

				// Add a constraint forcing the distance to be less than or equal to the distance from coord_i->coord_j
				GRBVar x_i = coord_i.at(0);
				GRBVar y_i = coord_i.at(1);
				GRBVar x_j = coord_j.at(0);
				GRBVar y_j = coord_j.at(1);
				model.addQConstr(d_j*d_j >= (x_i-x_j)*(x_i-x_j) + (y_i-y_j)*(y_i-y_j), "set_d_"+itos(dist_cout)+"_"+itos(dist_cout+1));
				if(DEBUG_OPTLAUNCH)
					printf("* d_%d*d_%d >= (x_%d-x_%d)*(x_%d-x_%d) + (y_%d-y_%d)*(y_%d-y_%d)\n", i, i, i, j, i, j, i, j, i, j);

				// Create UGV travel time constraints
				GRBVar t_i = action_time_vars.at(i);
				GRBVar t_j = action_time_vars.at(j);
				// How long does the action itself take?
				double action_time = 0.0;
				if(ordered_action_list.at(j).action_type == E_SOCActionType::e_LaunchDrone) {
					action_time = UAV_LAUNCH_TIME;
				}
				else if(ordered_action_list.at(j).action_type == E_SOCActionType::e_ReceiveDrone) {
					action_time = UAV_LAND_TIME;
				}

				model.addConstr(t_j >= t_i + d_j/UGV_V_CRG + action_time, "t_"+itos(j)+"_geq_t_"+itos(i));
				if(DEBUG_OPTLAUNCH)
					printf("* t_%d >= t_%d + d_%d/%f + %f\n", j, i, i, UGV_V_CRG, action_time);

				dist_cout++;
				dist_vars.push_back(d_j);
			}

			// Create sub-tour start/end distance constraints
			if(DEBUG_OPTLAUNCH) {
				printf("|sub_tours| = %ld, |sub_tour_dist_vars| = %ld, |action_coord_vars| = %ld, |action_time_vars| = %ld\n", sub_tours.size(), sub_tour_dist_vars.size(), action_coord_vars.size(), action_time_vars.size());
				printf("Sub-Tours:\n");
			}
			for(int tour = 0; tour < boost::numeric_cast<int>(sub_tours.size()); tour++) {
				int i = sub_tours.at(tour).launch_ID;
				int j = sub_tours.at(tour).land_ID;
				if(DEBUG_OPTLAUNCH)
					printf(" %d: [%d]:(%f,%f)- %f ->(%f,%f):[%d]\n", tour, i, sub_tours.at(tour).start_x, sub_tours.at(tour).start_y, sub_tours.at(tour).tour_dist, sub_tours.at(tour).end_x, sub_tours.at(tour).end_y, j);

				// Add constraint on sub-tour end time
				GRBVar t_i = action_time_vars.at(i);
				GRBVar t_j = action_time_vars.at(j);
				GRBVar d_s = sub_tour_dist_vars.at(tour).at(0);
				GRBVar d_e = sub_tour_dist_vars.at(tour).at(1);
				model.addConstr(t_j == t_i + d_s/UAV_V_MAX + sub_tours.at(tour).tour_dist/UAV_V_MAX + d_e/UAV_V_MAX + UAV_LAND_TIME, "t_"+itos(sub_tours.at(tour).launch_ID)+"_geq_t_"+itos(sub_tours.at(tour).land_ID)+"_+_td");
				if(DEBUG_OPTLAUNCH)
					printf("* t_%d == t_%d + d_s%d/%f + %f + d_e%d/%f + %f\n", j, i, tour, UAV_V_MAX, sub_tours.at(tour).tour_dist/UAV_V_MAX, tour, UAV_V_MAX, UAV_LAND_TIME);

				// Add constraint on sub-tour start/end leg distances
				auto coord_i = action_coord_vars.at(sub_tours.at(tour).launch_ID);
				GRBVar x_i = coord_i.at(0);
				GRBVar y_i = coord_i.at(1);
				GRBVar start_x = sub_tour_pos_vars.at(tour).at(0);
				GRBVar start_y = sub_tour_pos_vars.at(tour).at(1);
				model.addQConstr(d_s*d_s >= (x_i-start_x)*(x_i-start_x) + (y_i-start_y)*(y_i-start_y), "d_s"+itos(sub_tours.at(tour).ID)+"_geq_xi_x1");
				if(DEBUG_OPTLAUNCH)
					printf("* d_s%d*d_s%d >= (x_%d - start_x_%d)*(x_%d - start_x_%d) + (y_%d - start_y_%d)*(y_%d - start_y_%d)\n", tour, tour, i, tour, i, tour, i, tour, i, tour);
				auto coord_j = action_coord_vars.at(sub_tours.at(tour).land_ID);
				GRBVar x_j = coord_j.at(0);
				GRBVar y_j = coord_j.at(1);
				GRBVar end_x = sub_tour_pos_vars.at(tour).at(2);
				GRBVar end_y = sub_tour_pos_vars.at(tour).at(3);
				model.addQConstr(d_e*d_e >= (x_j-end_x)*(x_j-end_x) + (y_j-end_y)*(y_j-end_y), "d_e"+itos(sub_tours.at(tour).ID)+"_geq_xm_xj");
				if(DEBUG_OPTLAUNCH)
					printf("* d_e%d*d_e%d >= (x_%d - end_x%d)*(x_%d - end_x%d) + (y_%d - end_y%d)*(y_%d - end_y%d)\n",  tour, tour, j, tour, j, tour, j, tour, j, tour);
			}

			// Set objective
			GRBLinExpr obj = t_base;
			model.setObjective(obj, GRB_MINIMIZE);

			if(DEBUG_OPTLAUNCH)
				printf("Run Gurobi\n");

			// Optimize model
			model.optimize();


			if(DEBUG_OPTLAUNCH) {
				printf("minimized %s = %f\n", t_base.get(GRB_StringAttr_VarName).c_str(), t_base.get(GRB_DoubleAttr_X));

				// Record this so we can watch how well the optimizer is improving things
				FILE * pOutputFile;
				pOutputFile = fopen("qp_improvement.dat", "a");
				fprintf(pOutputFile, "%f\n", t_base.get(GRB_DoubleAttr_X));
				fclose(pOutputFile);

				printf("Actions:\n");
				for(int a_i = 0; a_i < boost::numeric_cast<int>(ordered_action_list.size()); a_i++) {
					printf(" %d: (%f,%f) @ %f", a_i, action_coord_vars.at(a_i).at(0).get(GRB_DoubleAttr_X), action_coord_vars.at(a_i).at(1).get(GRB_DoubleAttr_X), action_time_vars.at(a_i).get(GRB_DoubleAttr_X));
					if(ordered_action_list.at(a_i).action_type == E_SOCActionType::e_LaunchDrone) {
						printf(" launch %d\n", ordered_action_list.at(a_i).ID);
					}
					else if(ordered_action_list.at(a_i).action_type == E_SOCActionType::e_ReceiveDrone) {
						printf(" land %d\n", ordered_action_list.at(a_i).ID);
					}
					else {
						printf("\n");
					}
				}
			}


			// Clear the current solution
			sol_final->ClearUGVSolution(ugv_num);
			std::map<int, std::queue<DroneAction>> action_queues;
			std::map<int, std::vector<double>> drone_pos_x_y_t;
			for(int drone_j : drones_to_UGV.at(ugv_num)) {
				// Grab the current action list for this drone
				std::vector<DroneAction> action_list;
				sol_final->GetDroneActionList(drone_j, action_list);
				// Put the list into a queue
				std::queue<DroneAction, std::deque<DroneAction>> action_queue(std::deque<DroneAction>(action_list.begin(), action_list.end()));
				action_queues.insert(std::pair<int, std::queue<DroneAction>>(drone_j, action_queue));

				// Clear the current solution
				sol_final->ClearDroneSolution(drone_j);
			}

			double ugv_x_i;
			double ugv_y_i;
			double ugv_t_i;

			// Start with the initial action of each robot on this team
			{
				double x_b, y_b;
				input->GetDepot(ugv_num, &x_b, &y_b);
				double t_i = 0.0;

				// UGV is at the base depot
				UGVAction bsUGVAction(E_UGVActionTypes::e_AtDepot, x_b, y_b, t_i);
				sol_final->PushUGVAction(ugv_num, bsUGVAction);
				// Update the running position of the UGV
				ugv_x_i = x_b;
				ugv_y_i = y_b;
				ugv_t_i = t_i;

				// Add this to each drone
				for(int drone_j : drones_to_UGV.at(ugv_num)) {
					// Drone is at the UGV, at the base station
					DroneAction bsDroneAction(E_DroneActionTypes::e_AtUGV, x_b, y_b, t_i, ugv_num);
					sol_final->PushDroneAction(drone_j, bsDroneAction);
					// Record the drone's position
					std::vector<double> drone_x_y_t;
					drone_x_y_t.push_back(x_b);
					drone_x_y_t.push_back(y_b);
					drone_x_y_t.push_back(t_i);
					drone_pos_x_y_t.insert(std::pair<int, std::vector<double>>(drone_j, drone_x_y_t));
				}
			}

			// Extract solution (we want locations and times of each action)
			for(int a_i = 0; a_i < boost::numeric_cast<int>(ordered_action_list.size()); a_i++) {
				SOCAction action_i = ordered_action_list.at(a_i);

				// What type of action is this?
				if(action_i.action_type == E_SOCActionType::e_BaseStation) {
					// Do nothing... We handle this outside of this loop (before and after)
				}
				else if(action_i.action_type == E_SOCActionType::e_LaunchDrone) {
					// Where?
					double x_j = action_coord_vars.at(a_i).at(0).get(GRB_DoubleAttr_X);
					double y_j = action_coord_vars.at(a_i).at(1).get(GRB_DoubleAttr_X);
					// When?
					double dist_i_j = distAtoB(ugv_x_i, ugv_y_i, x_j, y_j);
					double t_i_j = dist_i_j/UGV_V_CRG;
					double t_j = t_i_j + ugv_t_i;

					// Move UGV to this location
					UGVAction moveUGV(E_UGVActionTypes::e_MoveToWaypoint, x_j, y_j, t_j);
					sol_final->PushUGVAction(ugv_num, moveUGV);

					// Update UGV position/time
					ugv_x_i = x_j;
					ugv_y_i = y_j;
					ugv_t_i = t_j;

					// Launch the drone
					t_j = ugv_t_i + UAV_LAUNCH_TIME;
					int drone_id = action_i.ID;
					UGVAction launchDrone(E_UGVActionTypes::e_LaunchDrone, ugv_x_i, ugv_y_i, t_j, drone_id);
					sol_final->PushUGVAction(ugv_num, launchDrone);
					DroneAction launchAction(E_DroneActionTypes::e_LaunchFromUGV, ugv_x_i, ugv_y_i, t_j, ugv_num);
					sol_final->PushDroneAction(drone_id, launchAction);

					// Update when the UGV finished doing this..
					ugv_t_i = t_j;

					// To track the drone
					double drone_x_i = ugv_x_i;
					double drone_y_i = ugv_y_i;
					double drone_t_i = ugv_t_i;

					// Add in all stops for this drone
					while(action_queues.at(drone_id).front().mActionType != E_DroneActionTypes::e_LandOnUGV) {
						DroneAction next_action = action_queues.at(drone_id).front();
						if(next_action.mActionType == E_DroneActionTypes::e_MoveToNode) {
							int node_id = next_action.mDetails;
							double x_j = vctrPOINodes.at(node_id).location.x, y_j = vctrPOINodes.at(node_id).location.y;
							double dist_i_j = distAtoB(drone_x_i, drone_y_i, x_j, y_j);
							double time_i_j = dist_i_j/UAV_V_MAX;
							double t_j = drone_t_i + time_i_j;

							// Add visit node action
							DroneAction launchAction(E_DroneActionTypes::e_MoveToNode, x_j, y_j, t_j, next_action.mDetails);
							sol_final->PushDroneAction(drone_id, launchAction);

							// Update drone's location
							drone_pos_x_y_t.at(drone_id).at(0) = x_j;
							drone_pos_x_y_t.at(drone_id).at(1) = y_j;
							drone_pos_x_y_t.at(drone_id).at(2) = t_j;

							// Update previous
							drone_x_i = x_j;
							drone_y_i = y_j;
							drone_t_i = t_j;
						}

						// Remove the last action
						action_queues.at(drone_id).pop();
					}
					// Pop off the land action (we create the new action later)
					action_queues.at(drone_id).pop();
				}
				else if(action_i.action_type == E_SOCActionType::e_ReceiveDrone) {
					// Move the UGV to this point, move the drone to this point, then land the drone
					// Where?
					double x_j = action_coord_vars.at(a_i).at(0).get(GRB_DoubleAttr_X);
					double y_j = action_coord_vars.at(a_i).at(1).get(GRB_DoubleAttr_X);
					// When?
					double dist_i_j = distAtoB(ugv_x_i, ugv_y_i, x_j, y_j);
					double t_i_j = dist_i_j/UGV_V_CRG;
					double t_j = t_i_j + ugv_t_i;

					// Move UGV to this location
					UGVAction moveUGV(E_UGVActionTypes::e_MoveToWaypoint, x_j, y_j, t_j);
					sol_final->PushUGVAction(ugv_num, moveUGV);

					// Update UGV position/time
					ugv_x_i = x_j;
					ugv_y_i = y_j;
					ugv_t_i = t_j;

					// Move the drone to this point
					int drone_id = action_i.ID;
					double dist_drone_j = distAtoB(drone_pos_x_y_t.at(drone_id).at(0), drone_pos_x_y_t.at(drone_id).at(1), x_j, y_j);
					double t_drone_j = drone_pos_x_y_t.at(drone_id).at(2) + dist_drone_j/UAV_V_MAX;
					double drone_arrives = std::max(t_drone_j, t_j);

					// Move drone to UGV
					DroneAction moveAction(E_DroneActionTypes::e_MoveToUGV, ugv_x_i, ugv_y_i, drone_arrives, ugv_num);
					sol_final->PushDroneAction(drone_id, moveAction);

					// Land the drone
					t_j = drone_arrives + UAV_LAND_TIME;
					UGVAction landDrone(E_UGVActionTypes::e_ReceiveDrone, ugv_x_i, ugv_y_i, t_j, drone_id);
					sol_final->PushUGVAction(ugv_num, landDrone);
					DroneAction landAction(E_DroneActionTypes::e_LandOnUGV, ugv_x_i, ugv_y_i, t_j, ugv_num);
					sol_final->PushDroneAction(drone_id, landAction);

					// Update when the UGV finished doing this..
					ugv_t_i = t_j;
				}
			}

			// Move back to the base
			{
				double x_b, y_b;
				input->GetDepot(ugv_num, &x_b, &y_b);
				double dist_i_b = distAtoB(ugv_x_i, ugv_y_i, x_b, y_b);
				double t_i_b = dist_i_b/UGV_V_CRG;
				double t_b = t_i_b + ugv_t_i;

				// UGV is at the base depot
				UGVAction bsUGVAction(E_UGVActionTypes::e_MoveToDepot, x_b, y_b, t_b);
				sol_final->PushUGVAction(ugv_num, bsUGVAction);

				// The kernel ends after the battery swap
				double kernel_complete_time = t_b + UGV_BAT_SWAP_TIME;
				// Add in end-of-kernel action for the UGV and each of its drones
				UGVAction ugvEndAction(E_UGVActionTypes::e_KernelEnd, x_b, y_b, kernel_complete_time);
				sol_final->PushUGVAction(ugv_num, ugvEndAction);
				for(int drone : drones_to_UGV.at(ugv_num)) {
					DroneAction endAction(E_DroneActionTypes::e_KernelEnd, x_b, y_b, kernel_complete_time);
					sol_final->PushDroneAction(drone, endAction);
				}
			}

		} catch(GRBException e) {
			printf("[ERROR] %d: %s\n", e.getErrorCode(), e.getMessage().c_str());
		} catch(const std::exception& e) {
			printf("Exception during optimization: %s\n", e.what());
		}
	}

	if(DEBUG_OPTLAUNCH) {
		sol_final->PrintSolution();
	}
}
