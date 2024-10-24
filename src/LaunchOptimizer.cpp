#include "LaunchOptimizer.h"
#include "UAV.h"
#include "UGV.h"


LaunchOptimizer::LaunchOptimizer() { }

LaunchOptimizer::~LaunchOptimizer() { }


void LaunchOptimizer::OptLaunching(int ugv_num, std::vector<int>& drones_on_UGV, PatrollingInput* input, Solution* sol_final) {
	if(DEBUG_LAUNCHOPT)
		printf("Optimizing UGV %d route\n", ugv_num);

	// Get the POI Nodes from the input
	std::vector<Node> vctrPOINodes = input->GetNodes();
	printf("vctrPOINodes size: %lu\n", vctrPOINodes.size());
	printf("ugv num: %d\n", ugv_num);
	// We need the lists of actions that each drone is performing
	std::vector<std::vector<DroneAction>> drone_action_lists;
	std::vector<int> drone_action_lists_i;
	// Just grab all of them for now...
	for(int j_a = 0; j_a < input->GetMa(); j_a++) {
		// Get action list
		std::vector<DroneAction> action_list_j;
		sol_final->GetDroneActionList(j_a, action_list_j);
		// Store said list
		drone_action_lists.push_back(action_list_j);
		// Track our progress through the list (start at 1 -- 0 is not really an action)
		drone_action_lists_i.push_back(1);
	}

	printf("Drone action lists size: %lu\n", drone_action_lists.size());

	// Create vectors of actions for each vehicle
	std::vector<UGVAction> ugv_final_actions;
	std::vector<std::vector<DroneAction>> drone_final_actions_j;
	for(int j_a = 0; j_a < input->GetMa(); j_a++) {
		std::vector<DroneAction> drone_actions;
		drone_final_actions_j.push_back(drone_actions);
	}

	// Create a map of queues of drone actions
	std::map<int, std::queue<DroneAction>> old_action_queues;
	for(int drone_j : drones_on_UGV) {
		// Grab the current action list for this drone
		std::vector<DroneAction> action_list;
		sol_final->GetDroneActionList(drone_j, action_list);
		// Put the list into a queue
		std::queue<DroneAction, std::deque<DroneAction>> action_queue(std::deque<DroneAction>(action_list.begin(), action_list.end()));
		old_action_queues.insert(std::pair<int, std::queue<DroneAction>>(drone_j, action_queue));
	}

	double ugv_tour_start = 0.0;

	bool process_next_team_tour = true;
	do {
		// Create a Gurobi environment
		try {
			GRBEnv env = GRBEnv();
			if(DEBUG_LAUNCHOPT) {
				printf("Starting up Gurobi\n");
			}
			else {
				env.set(GRB_INT_PAR_OUTPUTFLAG, "0");
			}
			GRBModel model = GRBModel(env);
			std::vector<std::vector<GRBVar>> sub_tour_dist_vars;
			std::vector<std::vector<GRBVar>> sub_tour_pos_vars;

			// Queue for actions
			std::priority_queue<SOCAction, std::vector<SOCAction>, CompareSOCAction> action_queue;
			// Create array of drone sub-tours
			std::vector<SubTour> sub_tours;
			int subtour_counter = 0;
			// For each drone assigned to the UGV
			for(int drone_id : drones_on_UGV) {
				printf("Drone %d\n", drone_id);
				if(DEBUG_LAUNCHOPT)
					printf(" Get actions for drone %d\n", drone_id);
				bool on_tour = false;
				double tour_dist = 0.0;
				double strt_x = 0.0, strt_y = 0.0;
				double prev_x = 0.0, prev_y = 0.0;


				// Cycle through the list of actions
				while(drone_action_lists_i.at(drone_id) < boost::numeric_cast<int>(drone_action_lists.at(drone_id).size())) {
					DroneAction next_action = drone_action_lists.at(drone_id).at(drone_action_lists_i.at(drone_id));

					// Move forward action index
					drone_action_lists_i.at(drone_id)++;

					// Are we on tour..?
					if(on_tour) {
						// Are we still visiting nodes...?
						if(next_action.mActionType == E_DroneActionTypes::e_MoveToNode) {
							// Visited next node, record distance and update previous
							printf("Visiting node %d (%f,%f)\n", next_action.mDetails, next_action.fX, next_action.fY);
							tour_dist += distAtoB(prev_x, prev_y, next_action.fX, next_action.fY);
							prev_x = next_action.fX;
							prev_y = next_action.fY;

							if(DEBUG_LAUNCHOPT)
								printf("   Stop at node %d (%f,%f) total distance = %f\n", next_action.mDetails, prev_x, prev_y, tour_dist);
						}
						else {
							// Not visiting a node... tour must have ended
							on_tour = false;

							if(DEBUG_LAUNCHOPT)
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
					else if(next_action.mActionType == E_DroneActionTypes::e_MoveToNode) {
						on_tour = true;
						strt_x = prev_x = next_action.fX;
						strt_y = prev_y = next_action.fY;
						if(DEBUG_LAUNCHOPT)
							printf("   First node %d (%f,%f)\n", next_action.mDetails, strt_x, strt_y);
					}
					// Is this a launch command?
					else if(next_action.mActionType == E_DroneActionTypes::e_LaunchFromUGV) {
						SOCAction action(next_action.fCompletionTime, drone_id, E_SOCActionType::e_LaunchDrone, subtour_counter);
						action_queue.push(action);
						if(DEBUG_LAUNCHOPT)
							printf("  Starting tour %d\n", subtour_counter);
					}
					// Is this a land command?
					else if(next_action.mActionType == E_DroneActionTypes::e_LandOnUGV) {
						SOCAction action(next_action.fCompletionTime, drone_id, E_SOCActionType::e_ReceiveDrone, subtour_counter);
						action_queue.push(action);
						if(DEBUG_LAUNCHOPT)
							printf("  Finished tour %d\n", subtour_counter);
						// We completed a subtour
						subtour_counter++;
					}
					// Is the end of the team's tour?
					else if(next_action.mActionType == E_DroneActionTypes::e_AtUGV) {
						// Break out of this loop...
						break;
					}
				}
			}

			// For my sanity
			if(DEBUG_LAUNCHOPT) {
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
				if(DEBUG_LAUNCHOPT)
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
				if(DEBUG_LAUNCHOPT)
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
						model.addConstr(t_i >= t_j + input->GetTMax(DRONE_I), "t_"+itos(action_cout)+"_geq_t_"+itos(prev_action_id.at(action.ID)));
						if(DEBUG_LAUNCHOPT)
							printf("* t_%d >= t_%d + %f\n", action_cout, prev_action_id.at(action.ID), input->GetTMax(DRONE_I));
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
			if(DEBUG_LAUNCHOPT) {
				printf(" a_%d [%d]%d - %f\n", action_cout, E_SOCActionType::e_BaseStation, -1, 0.0);
				printf("\nAll ordered actions:\n");
				for(SOCAction a : ordered_action_list) {
					printf(" %d type %d\n", a.ID, (int)a.action_type);
				}
			}

			/// Assume that stopping at the base station fully recharges the drones...
	#if 0
			// Constrain the fist launch time of each drone
			for(int drone_id : drones_on_UGV) {
				if(prev_action_id.at(drone_id) != first_action_id.at(drone_id)) {
					// Make sure the first action comes after the fully recharging from the last action
					GRBVar t_1 = action_time_vars.at(first_action_id.at(drone_id));
					GRBVar t_n = action_time_vars.at(prev_action_id.at(drone_id));
					UGV ugv = input->getUGV(ugv_num);
					model.addConstr(t_1 >= input->GetTMax(DRONE_I) - (ugv.batterySwapTime + t_base - t_n), "t_f"+itos(drone_id)+"_geq_t_l"+itos(drone_id));
					if(DEBUG_LAUNCHOPT)
						printf("* t_%d >= %f - t_%d + t_%d\n", first_action_id.at(drone_id), input->GetTMax(DRONE_I), action_cout, prev_action_id.at(drone_id));
				}
			}
	#endif

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
				if(DEBUG_LAUNCHOPT)
					printf("* d_%d*d_%d >= (x_%d-x_%d)*(x_%d-x_%d) + (y_%d-y_%d)*(y_%d-y_%d)\n", i, i, i, j, i, j, i, j, i, j);

				// Create UGV travel time constraints
				GRBVar t_i = action_time_vars.at(i);
				GRBVar t_j = action_time_vars.at(j);

				// How long does the action itself take?
				double action_time = 0.0;

				if(ordered_action_list.at(j).action_type == E_SOCActionType::e_LaunchDrone) {
					int droneID = ordered_action_list.at(j).ID;
					action_time = input->getUAV(droneID).timeNeededToLaunch;
				}
				else if(ordered_action_list.at(j).action_type == E_SOCActionType::e_ReceiveDrone) {
					int droneID = ordered_action_list.at(j).ID;
					action_time = input->getUAV(droneID).timeNeededToLand;
				}

				//ugv_num
				double ugv_v_charge = input->getUGV(ugv_num).ugv_v_crg;
				model.addConstr(t_j >= t_i + d_j/ugv_v_charge + action_time, "t_"+itos(j)+"_geq_t_"+itos(i));
				if(DEBUG_LAUNCHOPT)
					printf("* t_%d >= t_%d + d_%d/%f + %f\n", j, i, i, ugv_v_charge, action_time);


				dist_cout++;
				dist_vars.push_back(d_j);
			}

			// Create sub-tour start/end distance constraints
			if(DEBUG_LAUNCHOPT) {
				printf("|sub_tours| = %ld, |sub_tour_dist_vars| = %ld, |action_coord_vars| = %ld, |action_time_vars| = %ld\n", sub_tours.size(), sub_tour_dist_vars.size(), action_coord_vars.size(), action_time_vars.size());
				printf("Sub-Tours:\n");
			}
			for(int tour = 0; tour < boost::numeric_cast<int>(sub_tours.size()); tour++) {
				int i = sub_tours.at(tour).launch_ID;
				int j = sub_tours.at(tour).land_ID;
				if(DEBUG_LAUNCHOPT)
					printf(" %d: [%d]:(%f,%f)- %f ->(%f,%f):[%d]\n", tour, i, sub_tours.at(tour).start_x, sub_tours.at(tour).start_y, sub_tours.at(tour).tour_dist, sub_tours.at(tour).end_x, sub_tours.at(tour).end_y, j);

				// Add constraint on sub-tour end time
				GRBVar t_i = action_time_vars.at(i);
				GRBVar t_j = action_time_vars.at(j);
				GRBVar d_s = sub_tour_dist_vars.at(tour).at(0);
				GRBVar d_e = sub_tour_dist_vars.at(tour).at(1);

				int droneID = ordered_action_list.at(j).ID;
				UAV uav = input->getUAV(droneID);
				model.addConstr(t_j == t_i + d_s/input->GetDroneVMax(DRONE_I) + sub_tours.at(tour).tour_dist/input->GetDroneVMax(DRONE_I) + d_e/input->GetDroneVMax(DRONE_I) + uav.timeNeededToLand, "t_"+itos(sub_tours.at(tour).launch_ID)+"_geq_t_"+itos(sub_tours.at(tour).land_ID)+"_+_td");

				if(DEBUG_LAUNCHOPT)
					printf("* t_%d == t_%d + d_s%d/%f + %f + d_e%d/%f + %f\n", j, i, tour, input->GetDroneVMax(DRONE_I), sub_tours.at(tour).tour_dist/input->GetDroneVMax(DRONE_I), tour, input->GetDroneVMax(DRONE_I), uav.timeNeededToLand);


				// Add constraint on sub-tour start/end leg distances
				auto coord_i = action_coord_vars.at(sub_tours.at(tour).launch_ID);
				GRBVar x_i = coord_i.at(0);
				GRBVar y_i = coord_i.at(1);
				GRBVar start_x = sub_tour_pos_vars.at(tour).at(0);
				GRBVar start_y = sub_tour_pos_vars.at(tour).at(1);
				model.addQConstr(d_s*d_s >= (x_i-start_x)*(x_i-start_x) + (y_i-start_y)*(y_i-start_y), "d_s"+itos(sub_tours.at(tour).ID)+"_geq_xi_x1");
				if(DEBUG_LAUNCHOPT)
					printf("* d_s%d*d_s%d >= (x_%d - start_x_%d)*(x_%d - start_x_%d) + (y_%d - start_y_%d)*(y_%d - start_y_%d)\n", tour, tour, i, tour, i, tour, i, tour, i, tour);
				auto coord_j = action_coord_vars.at(sub_tours.at(tour).land_ID);
				GRBVar x_j = coord_j.at(0);
				GRBVar y_j = coord_j.at(1);
				GRBVar end_x = sub_tour_pos_vars.at(tour).at(2);
				GRBVar end_y = sub_tour_pos_vars.at(tour).at(3);
				model.addQConstr(d_e*d_e >= (x_j-end_x)*(x_j-end_x) + (y_j-end_y)*(y_j-end_y), "d_e"+itos(sub_tours.at(tour).ID)+"_geq_xm_xj");
				if(DEBUG_LAUNCHOPT)
					printf("* d_e%d*d_e%d >= (x_%d - end_x%d)*(x_%d - end_x%d) + (y_%d - end_y%d)*(y_%d - end_y%d)\n",  tour, tour, j, tour, j, tour, j, tour, j, tour);

				// Constrain total drone sub-tour distance
				model.addConstr(input->GetDroneMaxDist(DRONE_I) >= d_s + sub_tours.at(tour).tour_dist + d_e, "d_s"+itos(sub_tours.at(tour).launch_ID)+"_+_d_e"+itos(sub_tours.at(tour).land_ID)+"_leq_d_max");
				if(DEBUG_LAUNCHOPT)
					printf("* %f >= d_s%d + %f + d_e%d\n",  input->GetDroneMaxDist(DRONE_I), tour, sub_tours.at(tour).tour_dist, tour);
			}

			// Set objective
			GRBLinExpr obj = t_base;
			model.setObjective(obj, GRB_MINIMIZE);

			if(DEBUG_LAUNCHOPT)
				printf("Run Gurobi\n");

			// Optimize model
			model.optimize();

			if(DEBUG_LAUNCHOPT) {
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

			double ugv_x_i;
			double ugv_y_i;
			double ugv_t_i;
			std::map<int, std::vector<double>> drone_pos_x_y_t;

			// Start with the initial action of each robot on this team
			{
				double x_b, y_b;
				input->GetDepot(ugv_num, &x_b, &y_b);
				double t_i = ugv_tour_start;

				// UGV is at the base depot
				UGVAction bsUGVAction(E_UGVActionTypes::e_AtDepot, x_b, y_b, t_i);
				ugv_final_actions.push_back(bsUGVAction);
				// Update the running position of the UGV
				ugv_x_i = x_b;
				ugv_y_i = y_b;
				ugv_t_i = t_i;

				// Add this to each drone
				for(int drone_j : drones_on_UGV) {
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
					double t_i_j = dist_i_j/input->getUGV(ugv_num).ugv_v_crg;
					double t_j = t_i_j + ugv_t_i;

					// Move UGV to this location
					UGVAction moveUGV(E_UGVActionTypes::e_MoveToWaypoint, x_j, y_j, t_j);
					ugv_final_actions.push_back(moveUGV);

					// Update UGV position/time
					ugv_x_i = x_j;
					ugv_y_i = y_j;
					ugv_t_i = t_j;

					// Launch the drone

					int drone_id = action_i.ID;
					UAV uav = input->getUAV(drone_id);
					t_j = ugv_t_i + uav.timeNeededToLaunch;

					UGVAction launchDrone(E_UGVActionTypes::e_LaunchDrone, ugv_x_i, ugv_y_i, t_j, drone_id);
					ugv_final_actions.push_back(launchDrone);
					DroneAction launchAction(E_DroneActionTypes::e_LaunchFromUGV, ugv_x_i, ugv_y_i, t_j, ugv_num);
					drone_final_actions_j.at(drone_id).push_back(launchAction);

					// Update when the UGV finished doing this..
					ugv_t_i = t_j;

					// To track the drone
					double drone_x_i = ugv_x_i;
					double drone_y_i = ugv_y_i;
					double drone_t_i = ugv_t_i;

					// Add in all stops for this drone
					while(old_action_queues.at(drone_id).front().mActionType != E_DroneActionTypes::e_LandOnUGV) {
						DroneAction next_action = old_action_queues.at(drone_id).front();
						if(next_action.mActionType == E_DroneActionTypes::e_MoveToNode) {
							int node_id = next_action.mDetails;
							double x_j = vctrPOINodes.at(node_id).location.x, y_j = vctrPOINodes.at(node_id).location.y;
							double dist_i_j = distAtoB(drone_x_i, drone_y_i, x_j, y_j);
							double time_i_j = dist_i_j/input->GetDroneVMax(DRONE_I);
							double t_j = drone_t_i + time_i_j;

							// Add visit node action
							DroneAction launchAction(E_DroneActionTypes::e_MoveToNode, x_j, y_j, t_j, next_action.mDetails);
							drone_final_actions_j.at(drone_id).push_back(launchAction);

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
						old_action_queues.at(drone_id).pop();
					}
					// Pop off the land action (we create the new action later)
					old_action_queues.at(drone_id).pop();
				}
				else if(action_i.action_type == E_SOCActionType::e_ReceiveDrone) {
					// Move the UGV to this point, move the drone to this point, then land the drone
					// Where?
					double x_j = action_coord_vars.at(a_i).at(0).get(GRB_DoubleAttr_X);
					double y_j = action_coord_vars.at(a_i).at(1).get(GRB_DoubleAttr_X);
					// When?
					double dist_i_j = distAtoB(ugv_x_i, ugv_y_i, x_j, y_j);
					double t_i_j = dist_i_j/input->getUGV(ugv_num).ugv_v_crg;

					double t_j = t_i_j + ugv_t_i;

					// Move UGV to this location
					UGVAction moveUGV(E_UGVActionTypes::e_MoveToWaypoint, x_j, y_j, t_j);
					ugv_final_actions.push_back(moveUGV);

					// Update UGV position/time
					ugv_x_i = x_j;
					ugv_y_i = y_j;
					ugv_t_i = t_j;

					// Move the drone to this point
					int drone_id = action_i.ID;
					double dist_drone_j = distAtoB(drone_pos_x_y_t.at(drone_id).at(0), drone_pos_x_y_t.at(drone_id).at(1), x_j, y_j);
					double t_drone_j = drone_pos_x_y_t.at(drone_id).at(2) + dist_drone_j/input->GetDroneVMax(DRONE_I);
					double drone_arrives = std::max(t_drone_j, t_j);

					// Move drone to UGV
					DroneAction moveAction(E_DroneActionTypes::e_MoveToUGV, ugv_x_i, ugv_y_i, drone_arrives, ugv_num);
					drone_final_actions_j.at(drone_id).push_back(moveAction);

					// Land the drone
					UAV uav = input->getUAV(drone_id);
					t_j = drone_arrives + uav.timeNeededToLand;
					UGVAction landDrone(E_UGVActionTypes::e_ReceiveDrone, ugv_x_i, ugv_y_i, t_j, drone_id);
					ugv_final_actions.push_back(landDrone);
					DroneAction landAction(E_DroneActionTypes::e_LandOnUGV, ugv_x_i, ugv_y_i, t_j, ugv_num);
					drone_final_actions_j.at(drone_id).push_back(landAction);

					// Update when the UGV finished doing this..
					ugv_t_i = t_j;
				}
			}

			// Move back to the base
			{
				double x_b, y_b;
				input->GetDepot(ugv_num, &x_b, &y_b);
				double dist_i_b = distAtoB(ugv_x_i, ugv_y_i, x_b, y_b);
				double t_i_b = dist_i_b/input->getUGV(ugv_num).ugv_v_crg;

				double t_b = t_i_b + ugv_t_i;

				// UGV is at the base depot
				UGVAction bsUGVAction(E_UGVActionTypes::e_MoveToDepot, x_b, y_b, t_b);
				ugv_final_actions.push_back(bsUGVAction);

				// The kernel ends after the battery swap
				UGV ugv = input->getUGV(ugv_num);
				double kernel_complete_time = t_b + ugv.batterySwapTime;
				// Record that UGV is at the base station
				UGVAction ugvAtDepotAction(E_UGVActionTypes::e_AtDepot, x_b, y_b, kernel_complete_time);
				ugv_final_actions.push_back(ugvAtDepotAction);
				for(int drone : drones_on_UGV) {
					// Record that you are at the UGV
					DroneAction atUGVAction(E_DroneActionTypes::e_AtUGV, x_b, y_b, kernel_complete_time);
					drone_final_actions_j.at(drone).push_back(atUGVAction);
				}

				// Update when this things UGV tour ended
				ugv_tour_start = kernel_complete_time;
			}
		}
		catch(GRBException& e) {
			printf("[ERROR] %d: %s\n", e.getErrorCode(), e.getMessage().c_str());
		}
		catch(const std::exception& e) {
			printf("Exception during optimization: %s\n", e.what());
		}

		// Determine if there is another team-tour to process
		process_next_team_tour = false;
		for(int drone_j : drones_on_UGV) {
			if(drone_action_lists_i.at(drone_j) < boost::numeric_cast<int>(drone_action_lists.at(drone_j).size()) - 1) {
				process_next_team_tour = true;
			}
		}

		if(DEBUG_LAUNCHOPT) {
			if(process_next_team_tour) {
				printf("More team-tours to optimize!\n");
			}
			else {
				printf("Finished last team-tour\n");
			}
		}

	} while(process_next_team_tour);


	// Clear the current solution
	sol_final->ClearUGVSolution(ugv_num);
	for(int drone_j : drones_on_UGV) {
		sol_final->ClearDroneSolution(drone_j);
	}

	// Push back all UGV actions
	for(UGVAction a : ugv_final_actions) {
		sol_final->PushUGVAction(ugv_num, a);
	}
	// Add in end-of-kernel action for the UGV and each of its drones
	{
		UGVAction last_action = sol_final->GetLastUGVAction(ugv_num);
		UGVAction ugvEndAction(E_UGVActionTypes::e_KernelEnd, last_action.fX, last_action.fY, last_action.fCompletionTime);
		sol_final->PushUGVAction(ugv_num, ugvEndAction);
	}
	// Push back all drone actions
	for(int drone_id : drones_on_UGV) {
		double x_b, y_b;
		input->GetDepot(ugv_num, &x_b, &y_b);
		// The first action should be "at UGV"
		DroneAction bsDroneAction(E_DroneActionTypes::e_AtUGV, x_b, y_b, 0.0, ugv_num);
		sol_final->PushDroneAction(drone_id, bsDroneAction);
		for(DroneAction a : drone_final_actions_j.at(drone_id)) {
			sol_final->PushDroneAction(drone_id, a);
		}
		// Add in the end-of-kernel action
		auto last_action = sol_final->GetLastDroneAction(drone_id);
		DroneAction endAction(E_DroneActionTypes::e_KernelEnd, last_action.fX, last_action.fY, last_action.fCompletionTime);
		sol_final->PushDroneAction(drone_id, endAction);
	}

}
