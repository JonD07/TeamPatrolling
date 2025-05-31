#include "Solution.h"
#include "PatrollingInput.h"
#include "Utilities.h"
#include <cstdio>
#include <string>
#include <vector>

Solution::Solution(PatrollingInput* input) {
	// Initialize the size information
	m_input = input;

	// Add action vectors for each drone/UGV
	for(int j = 0; j < m_input->GetMa(); j++) {
		std::vector<DroneAction> action_list;
		m_Aa.push_back(action_list);
	}
	for(int j = 0; j < m_input->GetMg(); j++) {
		std::vector<UGVAction> action_list;
		m_Ag.push_back(action_list);
	}
}

Solution::Solution(const Solution &other) {
	m_input = other.m_input;

	// Add action vectors for each drone/UGV
	for(int j = 0; j < m_input->GetMa(); j++) {
		std::vector<DroneAction> action_list;
		m_Aa.push_back(action_list);
		// Move over all of the other solution's actions
		for(DroneAction action : other.m_Aa.at(j)) {
			m_Aa.at(j).push_back(action);
		}
	}
	for(int j = 0; j < m_input->GetMg(); j++) {
		std::vector<UGVAction> action_list;
		m_Ag.push_back(action_list);
		// Move over all of the other solution's actions
		for(UGVAction action : other.m_Ag.at(j)) {
			m_Ag.at(j).push_back(action);
		}
	}
}

Solution& Solution::operator=(const Solution &other) {
	m_input = other.m_input;

	m_Aa.clear();
	m_Ag.clear();

	// Add action vectors for each drone/UGV
	for(int j = 0; j < m_input->GetMa(); j++) {
		std::vector<DroneAction> action_list;
		m_Aa.push_back(action_list);
		// Move over all of the other solution's actions
		for(DroneAction action : other.m_Aa.at(j)) {
			m_Aa.at(j).push_back(action);
		}
	}
	for(int j = 0; j < m_input->GetMg(); j++) {
		std::vector<UGVAction> action_list;
		m_Ag.push_back(action_list);
		// Move over all of the other solution's actions
		for(UGVAction action : other.m_Ag.at(j)) {
			m_Ag.at(j).push_back(action);
		}
	}

	return *this;
}

Solution::~Solution() {
}

// Takes in an existing solution and generates a runtime version (clears any existing solution stored here)
void Solution::CreateRuntimeSolution(const Solution &other) {
//	m_input = other.m_input;
//	// Copy over the action lists
//	m_Aa.clear();
//	for(std::vector<DroneAction> act_list : other.m_Aa) {
//		// Create a new list locally
//		m_Aa.push_back(act_list);
//	}
//	m_Ag.clear();
//	for(std::vector<UGVAction> act_list : other.m_Ag) {
//		// Create a new list locally
//		m_Ag.push_back(act_list);
//	}
//
//	// For each UGV
//	for(int ugv_num = 0; ugv_num < m_input->GetMg(); ugv_num++) {
//		// Determine which starting point will reduce the total penalty (assume that it is the start of the tour)
//		double best_penalty = std::numeric_limits<double>::max();
//		int best_start_action = m_Ag.at(ugv_num).front().mActionID;
//
//		// Check each launch point
//		// TODO: ONLY WORKS WITH SINGLE DRONE!!
//		for(int act_i = 0; act_i < boost::numeric_cast<int>(m_Ag.at(ugv_num).size()); act_i++) {
//			if(m_Ag.at(ugv_num).at(act_i).mActionType == E_UGVActionTypes::e_LaunchDrone) {
//				// Create a new set of actions based on this being your starting point
//				std::vector<UGVAction> newUGVTour;
//				double prev_t = 0.0;
//				double prev_x;
//				double prev_y;
//				m_input->GetUGVInitLocal(ugv_num, &prev_x, &prev_y);
//
//				// Move from your location to this action
//				UGVAction inital_action(E_UGVActionTypes::e_MoveToWaypoint, m_Ag.at(ugv_num).at(act_i).fX, m_Ag.at(ugv_num).at(act_i).fY, prev_t);
//
//				// Add in the rest of the action list (including this first action)
//				for(int act_ii = act_i; act_ii < boost::numeric_cast<int>(m_Ag.at(ugv_num).size()); act_ii++) {
//
//				}
//			}
//			else {
//				// Don't consider this action as the starting point
//			}
//		}
//	}
}



// Calculates the average penalty accumulation rate
double Solution::CalculatePar() {
	double ret_val = 0.0;

	// Create a queue for times for each sensor
	std::vector< std::priority_queue<NodeService, std::vector<NodeService>, CompareNoderService> > Visits_i;
	for(int i = 0; i < m_input->GetN(); i++) {
		std::priority_queue<NodeService, std::vector<NodeService>, CompareNoderService> servicing_queue;
		Visits_i.push_back(servicing_queue);
	}

	// For each drone...
	for(std::vector<DroneAction> action_list : m_Aa) {
		double prev_kernel_end = 0.0;
		// Simulate the drone's kernel for ns trials
		for(int l = 0; l < N_S; l++) {
			for(DroneAction action : action_list) {
				// Did the drone visit a PoI?
				if(action.mActionType == E_DroneActionTypes::e_MoveToNode) {
					NodeService srv(action.fCompletionTime+prev_kernel_end, action.mActionID);
					// Push the completion time onto the respective PoI's queue
					Visits_i.at(action.mDetails).emplace(srv);
				}
				// Is this the end of the kernel?
				if(action.mActionType == E_DroneActionTypes::e_KernelEnd) {
					// Update when the previous kernel ends
					prev_kernel_end = action.fCompletionTime;
				}
			}
		}
	}

	// For each UGV...
	for(std::vector<UGVAction> action_list : m_Ag) {
		double prev_kernel_end = 0.0;
		// Simulate the UGV's kernel for ns trials
		for(int l = 0; l < N_S; l++) {
			for(UGVAction action : action_list) {
				// Did the drone visit a PoI?
				if(action.mActionType == E_UGVActionTypes::e_MoveToNode) {
					NodeService srv(action.fCompletionTime+prev_kernel_end, action.mActionID);
					// Push the completion time onto the respective PoI's queue
					Visits_i.at(action.mDetails).emplace(srv);
				}
				// Is this the end of the kernel?
				if(action.mActionType == E_UGVActionTypes::e_KernelEnd) {
					// Update when the previous kernel ends
					prev_kernel_end = action.fCompletionTime;
				}
			}
		}
	}

	// Sanity P:
	if(DEBUG_SOL) {
		printf("Cycling through visits to build S:\n");
	}

	// Create S, a set of sets where each S_i is a set of latencies from the kernel
	std::vector<std::vector<double>> S;
	for(auto queue : Visits_i) {
		// Latency list
		std::vector<double> S_i;
		// Get the time when the first visit occurs
		double prev = queue.top().time;
		prev = prev/3600.0; // Convert to hours
		// Record the ID of the first action... we will need this to know when the kernel has restarted
		int first_id = queue.top().ActionID;
		queue.pop();

		if(DEBUG_SOL)
			printf(" (%d:%f)", first_id, prev);

		// While the queue isn't empty and we haven't seen the first action again...
		bool run_again = true;
		while(!queue.empty() && run_again) {
			double next = queue.top().time;
			next = next/3600.0; // Convert to hours
			S_i.push_back(next - prev);

			if(DEBUG_SOL)
				printf(" (%d:%f)", queue.top().ActionID, next);

			// Did we hit the end of the kernel?
			if(queue.top().ActionID == first_id) {
				// We have found the repeat point..
				run_again = false;
			}
			else {
				// Get ready for the next iteration
				prev = next;
				queue.pop();
			}
		}

		if(DEBUG_SOL) {
			printf("\nS_i: ");
			for(double t : S_i) {
				printf("%f ", t);
			}
			printf("\n");
		}

		// Store S_i
		S.push_back(S_i);
	}

	if(DEBUG_SOL)
		printf("Total duration of each node kernel:\n");

	// Actually do the math for PAR
	for(std::vector<double> S_i : S) {
		// We first need the total time span of the node's kernel
		double total_time = 0.0;
		for(double t_i : S_i) {
			total_time += t_i;
		}

		if(DEBUG_SOL)
			printf("%f\n", total_time);

		// Add in this node's contribution to PAR
		for(double t_i : S_i) {
			ret_val += (t_i*t_i)/(2*total_time);
		}
	}

	if(DEBUG_SOL)
		printf("PAR: %f\n", ret_val);

	return ret_val;
}


// Prints this solution
void Solution::PrintSolution() {
	
	printf("Solution: N = %d, Ma = %d, Mg = %d\n", m_input->GetN(), m_input->GetMa(), m_input->GetMg());

	// Print the actions of each robot
	for(int j = 0; j < m_input->GetMa(); j++) {
		printf("Drone %d:\n", j);
		for(DroneAction action : m_Aa.at(j)) {
			printf("  [%d] %d(%d) : (%f, %f) - %f\n", action.mActionID, static_cast<std::underlying_type<E_DroneActionTypes>::type>(action.mActionType), action.mDetails, action.fX, action.fY, action.fCompletionTime);
		}
	}
	for(int j = 0; j < m_input->GetMg(); j++) {
		printf("UGV %d:\n", j);
		for(UGVAction action : m_Ag.at(j)) {
			printf("  [%d] %d(%d) : (%f, %f) - %f\n", action.mActionID, static_cast<std::underlying_type<E_UGVActionTypes>::type>(action.mActionType), action.mDetails, action.fX, action.fY, action.fCompletionTime);
		}
	}
}

// Prints the current solution into a yaml file (for testing with ARL)
void Solution::GenerateYAML(const std::string& filename) {
//	m_input
	YAML::Emitter out;
	out << YAML::Comment("API Version: 0.9.1");
	out << YAML::BeginMap;
	out << YAML::Key << "ID" << YAML::Value << "plan_every_UAV_action_02";
	out << YAML::Key << "state_ID" << YAML::Value << "state_every_UAV_action_02";
	out << YAML::Key << "description" << YAML::Value << "Team Patrolling";
	out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(0.0);

	double longest_end = 0.0;
	for(int a_j = 0; a_j < boost::numeric_cast<int>(m_Ag.size()); a_j++) {
		if(m_Ag.at(a_j).back().fCompletionTime > longest_end) {
			longest_end = m_Ag.at(a_j).back().fCompletionTime;
		}
	}
	out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(longest_end);

	out << YAML::Key << "individual_plans" << YAML::Value << YAML::BeginSeq;

	// UAV Plans
	for(int a_j = 0; a_j < boost::numeric_cast<int>(m_Aa.size()); a_j++) {
		out << YAML::BeginMap;
		out << YAML::Key << "agent_ID" << YAML::Value << m_input->GetDroneID(a_j);
		out << YAML::Key << "actions" << YAML::Value << YAML::BeginSeq;

		// Track the time of the last action
		double last_t = 0.0;
		double prev_x = 0.0, prev_y = 0.0;

		// Create a generic "start" actions
		{
			out << YAML::BeginMap;
			out << YAML::Key << "type" << YAML::Value << "start";
			out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(last_t);
			out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(last_t);
			out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "location" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "x" << YAML::Value << floatingPointToString(m_Aa.at(a_j).front().fX);
			out << YAML::Key << "y" << YAML::Value << floatingPointToString(m_Aa.at(a_j).front().fY);
			out << YAML::EndMap;
			out << YAML::EndMap;
			out << YAML::EndMap;

			prev_x = m_Aa.at(a_j).front().fX;
			prev_y = m_Aa.at(a_j).front().fY;
		}

		for(const auto& action : m_Aa.at(a_j)) {
			if(action.mActionType == E_DroneActionTypes::e_LaunchFromUGV) {
				// Record that the drone was purched on the UGV
				UAV uav = m_input->getUAV(a_j);
				out << YAML::BeginMap;
				out << YAML::Key << "type" << YAML::Value << "perch_on_UGV";
				out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(last_t);
				out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(action.fCompletionTime - uav.timeNeededToLaunch);
				out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "pad_ID" << YAML::Value << "pad_01";
				out << YAML::Key << "origin" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "x" << YAML::Value << floatingPointToString(prev_x);
				out << YAML::Key << "y" << YAML::Value << floatingPointToString(prev_y);
				out << YAML::EndMap;
				out << YAML::Key << "destination" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "x" << YAML::Value << floatingPointToString(action.fX);
				out << YAML::Key << "y" << YAML::Value << floatingPointToString(action.fY);
				out << YAML::EndMap;
				out << YAML::EndMap;
				out << YAML::EndMap;
				// Launch the drone
				out << YAML::BeginMap;
				out << YAML::Key << "type" << YAML::Value << "takeoff_from_UGV";;
				out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(action.fCompletionTime - uav.timeNeededToLaunch);
				out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(action.fCompletionTime);
				out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "pad_ID" << YAML::Value << "pad_01";
				out << YAML::Key << "start_progress" << YAML::Value << floatingPointToString(0.0);
				out << YAML::Key << "end_progress" << YAML::Value << floatingPointToString(1.0);
				out << YAML::Key << "location" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "x" << YAML::Value << floatingPointToString(action.fX);
				out << YAML::Key << "y" << YAML::Value << floatingPointToString(action.fY);
				out << YAML::EndMap;
				out << YAML::EndMap;
				out << YAML::EndMap;
			}
			else if(action.mActionType == E_DroneActionTypes::e_MoveToNode || action.mActionType == E_DroneActionTypes::e_MoveToUGV) {
				// First move to the node
				out << YAML::BeginMap;
				out << YAML::Key << "type" << YAML::Value << "move_to_location";
				out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(last_t);
				out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(action.fCompletionTime);
				out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "origin" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "x" << YAML::Value << floatingPointToString(prev_x);
				out << YAML::Key << "y" << YAML::Value << floatingPointToString(prev_y);
				out << YAML::EndMap;
				out << YAML::Key << "destination" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "x" << YAML::Value << floatingPointToString(action.fX);
				out << YAML::Key << "y" << YAML::Value << floatingPointToString(action.fY);
				out << YAML::EndMap;
				out << YAML::EndMap;
				out << YAML::EndMap;
				// Did we move to a node?
				if(action.mActionType == E_DroneActionTypes::e_MoveToNode) {
					// Service the node
					out << YAML::BeginMap;
					out << YAML::Key << "type" << YAML::Value << "service_node";;
					out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(action.fCompletionTime);
					out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(action.fCompletionTime);
					out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
					out << YAML::Key << "node_ID" << YAML::Value << m_input->GetNodeID(action.mDetails);
					out << YAML::Key << "location" << YAML::Value << YAML::BeginMap;
					out << YAML::Key << "x" << YAML::Value << floatingPointToString(action.fX);
					out << YAML::Key << "y" << YAML::Value << floatingPointToString(action.fY);
					out << YAML::EndMap;
					out << YAML::EndMap;
					out << YAML::EndMap;
				}
			}
			else if(action.mActionType == E_DroneActionTypes::e_LandOnUGV) {
				// Land on UGV
				out << YAML::BeginMap;
				out << YAML::Key << "type" << YAML::Value << "land_on_UGV";;
				out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(last_t);
				out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(action.fCompletionTime);
				out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "pad_ID" << YAML::Value << "pad_01";
				out << YAML::Key << "location" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "x" << YAML::Value << floatingPointToString(action.fX);
				out << YAML::Key << "y" << YAML::Value << floatingPointToString(action.fY);
				out << YAML::EndMap;
				out << YAML::Key << "start_progress" << YAML::Value << floatingPointToString(0.0);
				out << YAML::Key << "end_progress" << YAML::Value << floatingPointToString(1.0);
				out << YAML::EndMap;
				out << YAML::EndMap;
			}

			// Update previous location
			prev_x = action.fX;
			prev_y = action.fY;
			last_t = action.fCompletionTime;
		}

		// Create a generic "end" actions
		{
			out << YAML::BeginMap;
			out << YAML::Key << "type" << YAML::Value << "end";
			out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(m_Aa.at(a_j).back().fCompletionTime);
			out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(m_Aa.at(a_j).back().fCompletionTime);
			out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "location" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "x" << YAML::Value << floatingPointToString(m_Aa.at(a_j).back().fX);
			out << YAML::Key << "y" << YAML::Value << floatingPointToString(m_Aa.at(a_j).back().fY);
			out << YAML::EndMap;
			out << YAML::EndMap;
			out << YAML::EndMap;
		}

		out << YAML::EndSeq;
		out << YAML::EndMap;
	}

	// UGV Plans m_Ag
	for(int a_j = 0; a_j < boost::numeric_cast<int>(m_Ag.size()); a_j++) {
		out << YAML::BeginMap;
		out << YAML::Key << "agent_ID" << YAML::Value << m_input->GetUGVID(a_j);
		out << YAML::Key << "actions" << YAML::Value << YAML::BeginSeq;

		// Track the time of the last action
		double last_t = 0.0;
		double prev_x = 0.0, prev_y = 0.0;

		// Create a generic "start" actions
		{
			out << YAML::BeginMap;
			out << YAML::Key << "type" << YAML::Value << "start";
			out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(last_t);
			out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(last_t);
			out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "location" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "x" << YAML::Value << floatingPointToString(m_Ag.at(a_j).front().fX);
			out << YAML::Key << "y" << YAML::Value << floatingPointToString(m_Ag.at(a_j).front().fY);
			out << YAML::EndMap;
			out << YAML::EndMap;
			out << YAML::EndMap;

			prev_x = m_Ag.at(a_j).front().fX;
			prev_y = m_Ag.at(a_j).front().fY;
		}

		for(const auto& action : m_Ag.at(a_j)) {
			if(action.mActionType == E_UGVActionTypes::e_MoveToWaypoint || action.mActionType == E_UGVActionTypes::e_MoveToDepot || action.mActionType == E_UGVActionTypes::e_MoveToPosition) {
				if(CREATE_SPLINES) {
					// We need to break this up into smaller legs
					double crnt_x = prev_x;
					double crnt_y = prev_y;

					// While we are further than the max spline segment distance
					// while(distAtoB(crnt_x, crnt_y, action.fX, action.fY) > UGV_SPLINE_SEG_DIST) {
					double ugv_spline_seg_d = m_input->getUGV(a_j).SPlineSegDist;
					while(distAtoB(crnt_x, crnt_y, action.fX, action.fY) > ugv_spline_seg_d) {
						// Increment forward
						double delta_X = action.fX - prev_x;
						double delta_Y = action.fY - prev_y;
						double theta = 0.0;
						if(!isZero(delta_X)) {
							theta = atan(delta_Y/delta_X);
							if(delta_X < 0) {
								theta += PI;
							}
						}
						else {
							if(delta_Y > 0) {
								theta = PI/2.0;
							}
							else {
								theta = PI/-2.0;
							}
						}
						double delta_x = cos(theta)*ugv_spline_seg_d;
						double delta_y = sin(theta)*ugv_spline_seg_d;
						// How far are we going? (we expect this to be 10 m)
						double seg_dist = distAtoB(crnt_x, crnt_y, crnt_x + delta_x, crnt_y + delta_y);
					
						double seg_t = seg_dist/m_input->getUGV(a_j).ugv_v_crg;

						// Move the UGV over this distance
						out << YAML::BeginMap;
						out << YAML::Key << "type" << YAML::Value << "move_to_location";
						out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(last_t);
						out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(last_t + seg_t);
						out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
						out << YAML::Key << "origin" << YAML::Value << YAML::BeginMap;
						out << YAML::Key << "x" << YAML::Value << floatingPointToString(crnt_x);
						out << YAML::Key << "y" << YAML::Value << floatingPointToString(crnt_y);
						out << YAML::EndMap;
						out << YAML::Key << "destination" << YAML::Value << YAML::BeginMap;
						out << YAML::Key << "x" << YAML::Value << floatingPointToString(crnt_x + delta_x);
						out << YAML::Key << "y" << YAML::Value << floatingPointToString(crnt_y + delta_y);
						out << YAML::EndMap;
						out << YAML::EndMap;
						out << YAML::EndMap;

						// Update for next iteration
						crnt_x += delta_x;
						crnt_y += delta_y;
						last_t += seg_t;

					}

					// Add in the last leg (will be less than UGV_SPLINE_SEG_DIST in distance)
					out << YAML::BeginMap;
					out << YAML::Key << "type" << YAML::Value << "move_to_location";
					out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(last_t);
					out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(action.fCompletionTime);
					out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
					out << YAML::Key << "origin" << YAML::Value << YAML::BeginMap;
					out << YAML::Key << "x" << YAML::Value << floatingPointToString(crnt_x);
					out << YAML::Key << "y" << YAML::Value << floatingPointToString(crnt_y);
					out << YAML::EndMap;
					out << YAML::Key << "destination" << YAML::Value << YAML::BeginMap;
					out << YAML::Key << "x" << YAML::Value << floatingPointToString(action.fX);
					out << YAML::Key << "y" << YAML::Value << floatingPointToString(action.fY);
					out << YAML::EndMap;
					out << YAML::EndMap;
					out << YAML::EndMap;
				}
				else {
					// Just go directly to the next point
					out << YAML::BeginMap;
					out << YAML::Key << "type" << YAML::Value << "move_to_location";
					out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(last_t);
					out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(action.fCompletionTime);
					out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
					out << YAML::Key << "origin" << YAML::Value << YAML::BeginMap;
					out << YAML::Key << "x" << YAML::Value << floatingPointToString(prev_x);
					out << YAML::Key << "y" << YAML::Value << floatingPointToString(prev_y);
					out << YAML::EndMap;
					out << YAML::Key << "destination" << YAML::Value << YAML::BeginMap;
					out << YAML::Key << "x" << YAML::Value << floatingPointToString(action.fX);
					out << YAML::Key << "y" << YAML::Value << floatingPointToString(action.fY);
					out << YAML::EndMap;
					out << YAML::EndMap;
					out << YAML::EndMap;
				}

				// Did we go to the depot?
				if(action.mActionType == E_UGVActionTypes::e_MoveToDepot) {
					// Swap-out batteries
					out << YAML::BeginMap;
					out << YAML::Key << "type" << YAML::Value << "swap_battery";
					out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(action.fCompletionTime);
					UGV ugv = m_input->getUGV(a_j);	
					out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(action.fCompletionTime + ugv.batterySwapTime);
					out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
					out << YAML::Key << "start_progress" << YAML::Value << floatingPointToString(0.0);
					out << YAML::Key << "end_progress" << YAML::Value << floatingPointToString(1.0);
					out << YAML::Key << "location" << YAML::Value << YAML::BeginMap;
					out << YAML::Key << "x" << YAML::Value << floatingPointToString(action.fX);
					out << YAML::Key << "y" << YAML::Value << floatingPointToString(action.fY);
					out << YAML::EndMap;
					out << YAML::EndMap;
					out << YAML::EndMap;
				}
			}
			else if(action.mActionType == E_UGVActionTypes::e_LaunchDrone) {
				UAV uav = m_input->getUAV(a_j);
				out << YAML::BeginMap;
				out << YAML::Key << "type" << YAML::Value << "allow_takeoff_by_UAV";
				out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(action.fCompletionTime - uav.timeNeededToLaunch);
				out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(action.fCompletionTime);
				out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "UAV_ID" << YAML::Value << m_input->GetDroneID(action.mDetails);
				out << YAML::Key << "pad_ID" << YAML::Value << "pad_01";
				out << YAML::Key << "start_progress" << YAML::Value << floatingPointToString(0.0);
				out << YAML::Key << "end_progress" << YAML::Value << floatingPointToString(1.0);
				out << YAML::Key << "location" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "x" << YAML::Value << floatingPointToString(action.fX);
				out << YAML::Key << "y" << YAML::Value << floatingPointToString(action.fY);
				out << YAML::EndMap;
				out << YAML::EndMap;
				out << YAML::EndMap;
			}
			else if(action.mActionType == E_UGVActionTypes::e_ReceiveDrone) {
				UAV uav = m_input->getUAV(a_j);
				out << YAML::BeginMap;
				out << YAML::Key << "type" << YAML::Value << "allow_landing_by_UAV";
				out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(action.fCompletionTime - uav.timeNeededToLand);
				out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(action.fCompletionTime);
				out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "UAV_ID" << YAML::Value << m_input->GetDroneID(action.mDetails);
				out << YAML::Key << "pad_ID" << YAML::Value << "pad_01";
				out << YAML::Key << "start_progress" << YAML::Value << floatingPointToString(0.0);
				out << YAML::Key << "end_progress" << YAML::Value << floatingPointToString(1.0);
				out << YAML::Key << "location" << YAML::Value << YAML::BeginMap;
				out << YAML::Key << "x" << YAML::Value << floatingPointToString(action.fX);
				out << YAML::Key << "y" << YAML::Value << floatingPointToString(action.fY);
				out << YAML::EndMap;
				out << YAML::EndMap;
				out << YAML::EndMap;
			}

			// Update previous location
			prev_x = action.fX;
			prev_y = action.fY;
			last_t = action.fCompletionTime;
		}

		// Create a generic "end" actions
		{
			out << YAML::BeginMap;
			out << YAML::Key << "type" << YAML::Value << "end";
			out << YAML::Key << "start_time" << YAML::Value << floatingPointToString(m_Aa.at(a_j).back().fCompletionTime);
			out << YAML::Key << "end_time" << YAML::Value << floatingPointToString(m_Aa.at(a_j).back().fCompletionTime);
			out << YAML::Key << "task_parameters" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "location" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "x" << YAML::Value << floatingPointToString(m_Aa.at(a_j).back().fX);
			out << YAML::Key << "y" << YAML::Value << floatingPointToString(m_Aa.at(a_j).back().fY);
			out << YAML::EndMap;
			out << YAML::EndMap;
			out << YAML::EndMap;
		}

		out << YAML::EndSeq;
		out << YAML::EndMap;
	}

	out << YAML::EndSeq;
	out << YAML::EndMap;

	std::ofstream fout(filename);
	fout << out.c_str();
	fout.close();
}


// Calculates the Penalty Accumulation Rate of the current solution stored in this solution object.
double Solution::Benchmark() {
	return INF;
}


// Determines if this is a valid assignment solution (doesn't break constraints)
bool Solution::ValidSolution() {
	// bool valid = false;
	// return valid;
	/// Check each set of constraints
	// Loop through all actions, get each vehicle for that action, calculate energy at all times?
	for(int j = 0; j < m_input->GetMa(); j++) {
		printf("Drone %d:\n", j);
		UAV uav = m_input->getUAV(j);
		double currDroneEnergy = m_input->getUAV(j).battery_state.max_battery_energy;
		double lastActionTime = 0.0;
		for(DroneAction action : m_Aa.at(j)) {
			E_DroneActionTypes currAction = action.mActionType;
			if(currDroneEnergy <= 0.0) {
				printf("Drone %d ran out of energy at time %f\n", j, action.fCompletionTime);
				return false;
			}
			double energy;
			double time;
			switch (currAction) {
			
			case E_DroneActionTypes::e_AtUGV:
				//Nothing to be calculated here
				break;

			case E_DroneActionTypes::e_LaunchFromUGV:
				currDroneEnergy -= m_input->getUAV(j).energyToTakeOff;
				break;
			
			case E_DroneActionTypes::e_MoveToNode:
				time = action.fCompletionTime - lastActionTime;
				energy = uav.getJoulesPerSecondFlying(uav.maxSpeed);
				currDroneEnergy -= energy * time;
				break;

			case E_DroneActionTypes::e_MoveToUGV:
				time = action.fCompletionTime - lastActionTime;
				energy = uav.getJoulesPerSecondFlying(uav.maxSpeed);
				currDroneEnergy -= energy * time;
				break;
			
			case E_DroneActionTypes::e_LandOnUGV:
				currDroneEnergy -= m_input->getUAV(j).energyToLand;
				break;
			
			case E_DroneActionTypes::e_KernelEnd:
				//Nothing?
				break;
			
			default:
				break;
		}
			lastActionTime = action.fCompletionTime;
		}
	}

	for(int j = 0; j < m_input->GetMg(); j++) {
		printf("UGV %d:\n", j);
		double currUGVEnergy = m_input->getUGV(j).battery_state.max_battery_energy;
		double lastActionTime = 0.0;
		for(UGVAction action : m_Ag.at(j)) {
			E_UGVActionTypes currAction = action.mActionType;
			if(currUGVEnergy <= 0.0) {
				printf("UGV %d ran out of energy at time %f\n", j, action.fCompletionTime);
				return false;
			}
			UGV ugv = m_input->getUGV(j);
			double energy;
			double time;
			switch (currAction) {
			
			case E_UGVActionTypes::e_MoveToWaypoint:
				time = action.fCompletionTime - lastActionTime;
				energy = ugv.getJoulesPerSecondDriving(ugv.maxDriveAndChargeSpeed);
				currUGVEnergy -= energy * time;
				break;

			case E_UGVActionTypes::e_MoveToDepot:
				time = action.fCompletionTime - lastActionTime;
				energy = ugv.getJoulesPerSecondDriving(ugv.maxDriveAndChargeSpeed);
				currUGVEnergy -= energy * time;
				break;

			case E_UGVActionTypes::e_LaunchDrone:
				break;

			case E_UGVActionTypes::e_ReceiveDrone:
				break;

			case E_UGVActionTypes::e_MoveToNode:
				time  = action.fCompletionTime - lastActionTime;
				energy = ugv.getJoulesPerSecondDriving(ugv.maxDriveAndChargeSpeed);
				currUGVEnergy -= energy * time;				
				break;

			case E_UGVActionTypes::e_AtDepot:
				//reset energy to max?
				break;

			case E_UGVActionTypes::e_KernelEnd:
				//Nothing?
				break;

			default:
				break;
		}
			lastActionTime = action.fCompletionTime;
		}
	}
	return true;
}

// Pushes action onto drone j's action list
void Solution::PushDroneAction(int j, const DroneAction& action) {
	m_Aa.at(j).push_back(action);
}

// Pushes action onto UGV j's action list
void Solution::PushUGVAction(int j, const UGVAction& action) {
	m_Ag.at(j).push_back(action);
}

// Get the last action for drone j
const DroneAction& Solution::GetLastDroneAction(int j) {
	if(m_Aa.at(j).empty()) {
		// No previous actions, push in some dummy action
		DroneAction dummy(E_DroneActionTypes::e_AtUGV, 0, 0, 0);
		m_Aa.at(j).push_back(dummy);

		// Print some nasty message
		fprintf(stderr, "[%s]:Solution::GetLastDroneAction(): Drone action array %d is empty!\n", ERROR, j);
	}

	return m_Aa.at(j).back();
}

// Get the last action for UGV j
const UGVAction& Solution::GetLastUGVAction(int j) {
	if(m_Ag.at(j).empty()) {
		// No previous actions, push in some dummy action
		UGVAction dummy(E_UGVActionTypes::e_AtDepot, 0, 0, 0);
		m_Ag.at(j).push_back(dummy);

		// Print some nasty message
		fprintf(stderr, "[%s]:Solution::GetLastUGVAction(): UGV action array %d is empty!\n", ERROR, j);
	}

	return m_Ag.at(j).back();
}

// Get the current action list of drone j
void Solution::GetDroneActionList(int j, std::vector<DroneAction>& lst) {
	// Fill lst with all actions in drone j's list
	for(DroneAction a : m_Aa.at(j)) {
		lst.push_back(a);
	}
}

// Get the current action list of UGV j
void Solution::GetUGVActionList(int j, std::vector<UGVAction>& lst) {
	// Fill lst with all actions in UGV j's list
	for(UGVAction a : m_Ag.at(j)) {
		lst.push_back(a);
	}
}

// Returns the time of the last action of UGV j
double Solution::GetTotalTourTime(int j) {
	if(!m_Ag.at(j).empty()) {
		return m_Ag.at(j).back().fCompletionTime;
	}
	return 0.0;
}

// Completely clears the current solution (deletes all actions and completion times)
void Solution::ClearSolution() {
	// Clear all action vectors
	for(int j = 0; j < m_input->GetMa(); j++) {
		m_Aa.at(j).clear();
	}
	for(int j = 0; j < m_input->GetMg(); j++) {
		m_Ag.at(j).clear();
	}
}

// Deletes the current plan (actions and completion times) for drone j
void Solution::ClearDroneSolution(int j) {
	m_Aa.at(j).clear();
}

// Deletes the current plan (actions and completion times) for UGV j
void Solution::ClearUGVSolution(int j) {
	m_Ag.at(j).clear();
}

// Helper function to convert DroneActionType enum to string
std::string Solution::droneActionTypeToString(E_DroneActionTypes actionType) {
    switch (actionType) {
//        case E_DroneActionTypes::Start: return "start";
        case E_DroneActionTypes::e_AtUGV: return "perch_on_UGV";
        case E_DroneActionTypes::e_LaunchFromUGV: return "takeoff_from_UGV";
//        case E_DroneActionTypes::MoveToLocation: return "move_to_location";
//        case E_DroneActionTypes::ServiceNode: return "service_node";
        case E_DroneActionTypes::e_LandOnUGV: return "land_on_UGV";
        case E_DroneActionTypes::e_KernelEnd: return "end";
        default:return "unknown";
    }
}

// Helper function to convert UGVActionType enum to string
std::string Solution::ugvActionTypeToString(E_UGVActionTypes actionType) {
    switch (actionType) {
//        case E_UGVActionTypes::Start: return "start";
        case E_UGVActionTypes::e_MoveToWaypoint: return "move_to_location";
        case E_UGVActionTypes::e_LaunchDrone: return "allow_takeoff_by_UAV";
//        case E_UGVActionTypes::ServiceNode: return "service_node";
        case E_UGVActionTypes::e_ReceiveDrone: return "allow_landing_by_UAV";
        case E_UGVActionTypes::e_KernelEnd: return "end";
        default:return "unknown";
    }
}

// Reduce precision of floating point number
std::string Solution::floatingPointToString(double val) {
	std::stringstream stream;
	stream << std::fixed << std::setprecision(1) << val;
	return stream.str();
}


// Function to swap an Entire UGV Action List out
void Solution::swapUGVActionList(int UGVId, std::vector<UGVAction>& newActionList) {
	ClearUGVSolution(UGVId);

	for(size_t i = 0; i < newActionList.size(); i++) {
		m_Ag.at(UGVId).push_back(newActionList.at(i));
	}
}

// Function to swap an Entire Drone Action List out
void Solution::swapDroneActionLists(int DroneId, const std::vector<DroneAction>& newActionList) {
	ClearDroneSolution(DroneId);

	for(size_t i = 0; i < newActionList.size(); i++) {
		m_Aa.at(DroneId).push_back(newActionList.at(i));
	}
}

void Solution::swapUGVActions(int ugv_num, int index1, int index2) {
	std::vector<UGVAction>& actionList = m_Ag.at(ugv_num);

	// Bounds checking to prevent out-of-range errors
	if(index1 < 0 || index1 >= boost::numeric_cast<int>(actionList.size()) ||
			index2 < 0 || index2 >= boost::numeric_cast<int>(actionList.size())) {
		throw std::out_of_range("Index out of range in swapUGVActions");
	}

	UGVAction& ugv_action1 = actionList[index1];
	UGVAction& ugv_action2 = actionList[index2];

	// Swap the actions (manually, since std::swap isn't available)
	UGVAction temp = std::move(ugv_action1);
	ugv_action1 = std::move(ugv_action2);
	ugv_action2 = std::move(temp);

	// Swap the completion times explicitly
	std::swap(ugv_action1.fCompletionTime, ugv_action2.fCompletionTime);

	// Update the new completion times for the drones as well
	int action1_drone_id = ugv_action1.mDetails; 
	std::vector<DroneAction>& action1List = m_Aa[action1_drone_id];
	for(DroneAction& drone_action : action1List) {
		if(drone_action.fCompletionTime == ugv_action2.fCompletionTime ) {
			drone_action.fCompletionTime = ugv_action1.fCompletionTime;
			break;
		}
	}

	int action2_drone_id = ugv_action2.mDetails;
	std::vector<DroneAction>& action2List = m_Aa[action2_drone_id];
	for(DroneAction& drone_action : action2List) {
		if(drone_action.fCompletionTime == ugv_action1.fCompletionTime ) {
			drone_action.fCompletionTime = ugv_action2.fCompletionTime;
			break; 
		}
	}
}


bool Solution::collisionsPresent(const UGVAction actionA, const UGVAction actionB, const std::vector<Obstacle>& obstacles) {
	for (Obstacle o : obstacles) {
		if (Obstacle::checkForObstacle(actionA.fX, actionA.fY, actionB.fX, actionB.fY, o)) {
			printf("Obstacle found between two actions:\n");
			o.printInfo();
			actionA.print();
			actionB.print();
			return true;
		}
	}
	return false; 
}

double Solution::calcUGVMovingEnergy(UGVAction& UGV_last, UGVAction& UGV_current,UGV& UGV_current_object) {
	double dist_prev_next = distAtoB(UGV_last.fX, UGV_last.fY, UGV_current.fX, UGV_current.fY);
	double t_duration = dist_prev_next/UGV_current_object.ugv_v_crg; 
	double drivingEnergy = UGV_current_object.getJoulesPerSecondDriving(UGV_current_object.maxDriveAndChargeSpeed);
	double move_energy = drivingEnergy*t_duration; 
	if (DEBUG_SOL) {
		printf("The Energy calcuated from the move: [%f]\n", move_energy);
	}
	return move_energy; 
}

bool Solution::validateMovementAndTiming(const UGVAction& prev, const UGVAction& next, const UGV& ugv, double overhead_time = 0.0) {
    double dist = distAtoB(prev.fX, prev.fY, next.fX, next.fY);
    double expected_time = (dist / ugv.ugv_v_crg) + overhead_time;
    double actual_time = next.fCompletionTime - prev.fCompletionTime;

    // Return true if the actual time is enough to cover the distance including the overhead
	bool result = std::abs(actual_time - expected_time) <= 1.0;
	if (!result) {
		printf("Movement/Timing was invalid between these actions: \n");
		prev.print();
		next.print();
		printf("Actual time: %f vs Expected time: %f\n", actual_time, expected_time);
	}
	return result; 
}




bool Solution::syncUGVDroneAction(UGVAction& UGV_action, DroneAction& drone_action) {
	bool condition = floatEquality(UGV_action.fCompletionTime, drone_action.fCompletionTime) &&
		floatEquality(UGV_action.fX, drone_action.fX) && floatEquality(UGV_action.fY, drone_action.fY);

	if (!condition) {
		printf("UGV action is not synced with Drone action: \n");
		UGV_action.print();
		drone_action.print();
	}
	return condition; 
}

void Solution::chargeDrone(UAV& drone, double time_on_UGV) {
	double t_available = time_on_UGV - drone.charge_startup_t;

	if (t_available <= 0.0) {
		// Not enough time to begin charging
		return;
	}

	double energy_gained = 0.0;

	if (drone.subtype == "standard") {
		if (t_available <= (drone.t_max - drone.t_star)) {
			// Fast charge phase only
			double t = t_available;
			energy_gained = drone.fast_charge_a * t * t + drone.fast_charge_b * t;
		} else {
			// Fast charge + slow charge
			double fast_time = drone.t_max - drone.t_star;
			double fast_energy = drone.fast_charge_a * fast_time * fast_time + drone.fast_charge_b * fast_time;

			double t_slow = t_available - fast_time;
			double t_final = drone.t_star - t_slow;

			double slow_energy = drone.e_star -
			                     (drone.p_star / ALPHA) * (exp(ALPHA * (drone.t_star - t_final)) - 1);

			energy_gained = fast_energy + (drone.e_star - slow_energy);
		}
	}
	else if (drone.subtype == "a_field") {
		double t = t_available;
		energy_gained = drone.fast_charge_a * t * t + drone.fast_charge_b * t;
	}
	else {
		fprintf(stderr, "[%s] : calcChargeAmount() : Drone subtype '%s' not recognized!\n", ERROR, drone.subtype.c_str());
		exit(1);
	}

	// Safely add energy, but cap at max capacity
	drone.battery_state.current_battery_energy = std::min(
		drone.battery_state.current_battery_energy + energy_gained,
		drone.battery_state.max_battery_energy
	);
}




bool Solution::validateDroneTrip(UAV& droneA, const std::vector<DroneAction>& action_list, int& list_index) {
    double energyPerSecond = droneA.getJoulesPerSecondFlying(droneA.maxSpeed);
	const double DRONE_BATTERY_ZERO = -.005 * droneA.battery_state.max_battery_energy;

    if (list_index == 0 || list_index >= boost::numeric_cast<int>(action_list.size())) {
        printf("Invalid starting index for drone trip validation\n");
        return false;
    }

    // Step 1: Subtract energy to take off
    droneA.battery_state.current_battery_energy -= droneA.energyToTakeOff;
    if (droneA.battery_state.current_battery_energy < DRONE_BATTERY_ZERO) {
        printf("Drone %s ran out of battery at takeoff\n", droneA.ID.c_str());
        return false;
    }

    list_index++; // move past the launch

    while (true) {
        if (list_index >= boost::numeric_cast<int>(action_list.size())) {
            printf("Drone %s never landed on UGV\n", droneA.ID.c_str());
            return false;
        }

        const DroneAction& curr_action = action_list[list_index];

        if (curr_action.mActionType == E_DroneActionTypes::e_LandOnUGV) {
			// Step 3: Subtract energy to land
            droneA.battery_state.current_battery_energy -= droneA.energyToLand;
            if (droneA.battery_state.current_battery_energy < DRONE_BATTERY_ZERO) {
                printf("Drone %s ran out of battery during landing\n", droneA.ID.c_str());
				printf("Drone battery was this much over: %f: \n", droneA.battery_state.current_battery_energy);
				printf("This drone action caused it: \n");
				curr_action.print();
                return false;
            }
            break;
        }

        switch (curr_action.mActionType) {
            case E_DroneActionTypes::e_MoveToNode:
            case E_DroneActionTypes::e_MoveToUGV: {
                const DroneAction& prev_action = action_list[list_index - 1];
                double dist_prev_next = distAtoB(prev_action.fX, prev_action.fY, curr_action.fX, curr_action.fY);
                double t_duration = dist_prev_next / droneA.maxSpeed;
                double move_energy = t_duration * energyPerSecond;

                // Step 2: Subtract energy for this movement
                droneA.battery_state.current_battery_energy -= move_energy;
                if (droneA.battery_state.current_battery_energy < DRONE_BATTERY_ZERO) {
                    printf("Drone %s ran out of battery during movement\n", droneA.ID.c_str());
                    return false;
                }
                break;
            }
            default:
                printf("Invalid drone action after launch:\n");
                curr_action.print();
                return false;
        }

        list_index++;
    }

	// * If we go negative make sure to reset to prevent cascading issues
	if (droneA.battery_state.current_battery_energy < 0) {
		droneA.battery_state.current_battery_energy = 0; 
	}
    return true;
}

bool Solution::is_valid(PatrollingInput* input, int algorithm){
	// * Just trust me on this one (sorry if you are looking at this decision)
	struct DroneMeta {
		int action_list_index;
		UAV drone_obj;
		double time_landed;
	};

	// * Look through the actions of each UGV	
	for (int ugv_index = 0; ugv_index < input->GetMg(); ugv_index++){
		std::vector<UGVAction> UGVActions; 
		GetUGVActionList(ugv_index, UGVActions);
		double ugv_energy = input->GetUGVBatCap(ugv_index); 
		UGVAction curr_action = UGVActions[0];
		
		// * Variables to keep track of the corresponding Drones
		std::vector<std::vector<int>> drones_to_UGV;
		input->AssignDronesToUGV(drones_to_UGV);

		std::vector<int> assigned_drones = drones_to_UGV[ugv_index];

		std::map<int, DroneMeta> drone_map;  // droneID → DroneMeta


		int action_list_index = 1;
		for (int drone_index : assigned_drones) {
			UAV drone_obj = input->getUAV(drone_index);
			int drone_ID = drone_index;
			drone_obj.battery_state.current_battery_energy = drone_obj.battery_state.max_battery_energy;

			DroneMeta meta;
			meta.action_list_index = action_list_index;
			meta.drone_obj = drone_obj;
			meta.time_landed = 0.0; 

			drone_map[drone_ID] = meta;
		}

		// * Going to handle the first action hard code style
		// * Check to make sure there are no obstacles in this point 
		for (const auto &obstacle : input->GetObstacles()) {
			if (obstacle.containsPoint(curr_action.fX, curr_action.fY)) {
				printf("This action: \n");
				curr_action.print();
				printf("Was found inside this obstacle: \n");
				obstacle.printInfo();
				return false;
			}
		}

		// * First, check that its a at depot, and time is 0
		if (!(curr_action.mActionType == E_UGVActionTypes::e_AtDepot && floatEquality(curr_action.fCompletionTime, 0))) {
			printf("First action is malformed\n");
			curr_action.print();
			return false;
		}

		/* 
		* 3 things that we are focusing on checking for UGVs
		* 1. are action inside obstacles and do movement not cross through obstacles
		* 2. Does timing match up, does it start at 0, do times increase as we go, do times increase when the should increase and stay the same when they should
		* 3. Does the UGV correcrtly not use up all its energy
		*/
		for (size_t ugv_action_index = 1; ugv_action_index < UGVActions.size(); ugv_action_index++) {
			UGVAction prev_action = UGVActions[ugv_action_index -1];
			curr_action = UGVActions[ugv_action_index];


			// * Do obstacle checking for both containment and interseciton 
			// TODO this could be expensive / Need to find how to only check if it matters
			if (algorithm > 6) {
				if (collisionsPresent(prev_action, curr_action, input->GetObstacles())) {
					return false; 
				}
			}

			UGV ugv_obj = input->getUGV(ugv_index);
		


			// * Now Logic is specific to action type
			switch (curr_action.mActionType) {
				case E_UGVActionTypes::e_MoveToWaypoint:
				case E_UGVActionTypes::e_MoveToPosition:
				case E_UGVActionTypes::e_MoveToDepot:
					// * Calc energy of the move and remove from energy we have
					ugv_energy -= calcUGVMovingEnergy(prev_action, curr_action, ugv_obj);
					// * Validate distance, moving, and timing
					if (!validateMovementAndTiming(prev_action, curr_action, ugv_obj)) {
						return false; 
					}
					break;


					break;

				case E_UGVActionTypes::e_LaunchDrone:
					{
					// * Gather our drone info 
					int drone_ID = curr_action.mDetails;
					auto it = drone_map.find(drone_ID);
					if (it == drone_map.end()) {
						printf("Error: drone_ID %d not found in drone_map\n", drone_ID);
						printf("Known drone IDs in map: ");
						for (const auto& pair : drone_map) {
							printf("%d ", pair.first);
						}
						printf("\n");

						return false;
					}
					DroneMeta& meta = it->second; 
					UAV& drone_OBJ = meta.drone_obj;

					// * Validate the lack of movement of the UGV
					if (!validateMovementAndTiming(prev_action, curr_action, ugv_obj, drone_OBJ.timeNeededToLaunch)) {
						return false; 
					}

					
					int& drone_AL_index = meta.action_list_index;
					std::vector<DroneAction> AL_list; 
					GetDroneActionList(drone_ID, AL_list);
					DroneAction curr_drone_action = AL_list[drone_AL_index];
					// * First make sure the drone action is a launch
					if (curr_drone_action.mActionType != E_DroneActionTypes::e_LaunchFromUGV) {
						printf("This drone action should be a launch for Drone %d:\n", drone_ID);
						curr_drone_action.print();
						return false; 
					}

					// * Second we want to make sure the location and time are synced between the UAV/UGV launch
					if(!syncUGVDroneAction(curr_action, curr_drone_action)) {
						return false;
					}

					// * Charge the drone if its been sitting on the UGV (charging) for more than 0 seconds 
					double charge_time = curr_action.fCompletionTime - meta.time_landed;
					chargeDrone(drone_OBJ, charge_time);
					

					// * Validate the drone trip
					if (!validateDroneTrip(drone_OBJ, AL_list, drone_AL_index)) {
						return false; 
					}

					}
					break; 
				case E_UGVActionTypes::e_ReceiveDrone:
					{
					// * Gather our drone info 
					int drone_ID = curr_action.mDetails;
					auto it = drone_map.find(drone_ID);
					if (it == drone_map.end()) {
						printf("Error: drone_ID %d not found in drone_map\n", drone_ID);
						printf("Known drone IDs in map: ");
						for (const auto& pair : drone_map) {
							printf("%d ", pair.first);
						}
						printf("\n");

						return false;
					}
					DroneMeta& meta = it->second; 

					// * Validate no movement of the UGV
					bool noMovement = floatEquality(prev_action.fX, curr_action.fX) &&
                  			floatEquality(prev_action.fY, curr_action.fY);
					if (!noMovement) {
						printf("UGV moved during a recieve:\n");
						prev_action.print();
						curr_action.print();
						return false;
					}
					
					int& drone_AL_index = meta.action_list_index;
					std::vector<DroneAction> AL_list; 
					GetDroneActionList(drone_ID, AL_list);
					DroneAction curr_drone_action = AL_list[drone_AL_index];
					// * First make sure the drone action is a land
					if (curr_drone_action.mActionType != E_DroneActionTypes::e_LandOnUGV) {
						printf("This drone action should be a land:\n");
						curr_drone_action.print();
						return false; 
					}
					
					// * Make sure the drone and UGV landing are synced 
					if(!syncUGVDroneAction(curr_action, curr_drone_action)) {
						return false;
					}

					// * Lastly increment the index and set the land time
					meta.time_landed = curr_action.fCompletionTime;
					drone_AL_index++; 

					}
					break;
				case E_UGVActionTypes::e_AtDepot:
					// * Should be no movement and just the battery swap time 
					if (!validateMovementAndTiming(prev_action, curr_action, ugv_obj, ugv_obj.batterySwapTime)) {
							return false; 
					}

					// * Make sure its not out of energy
					if (floatEquality(ugv_energy, 0.0)) {
						printf("This action caused the UGV to run out energy: \n");
						curr_action.print();
						printf("The UGV energy level: \n");
						std::cout << ugv_energy << std::endl; 
						return false; 
					}

					// * Reset the energy
					ugv_energy = input->GetUGVBatCap(ugv_index);
					break;

				case E_UGVActionTypes::e_KernelEnd:
					// * Should be no movement and no time changing
					if (!validateMovementAndTiming(prev_action, curr_action, ugv_obj)) {
						return false; 
					}

					// * Check to make sure the drones and UGV end at the same time and at the same place
					for (int drone_index : assigned_drones) {
						DroneMeta meta = drone_map[drone_index];
						std::vector<DroneAction> AL_list; 
						GetDroneActionList(drone_index, AL_list);

						if (meta.action_list_index >= boost::numeric_cast<int>(AL_list.size())) {
							printf("Drone %d's action index is out of bounds\n", drone_index);
							return false;
						}

						const DroneAction& curr_drone_action = AL_list[meta.action_list_index];

						bool time_match = floatEquality(curr_action.fCompletionTime, curr_drone_action.fCompletionTime);
						bool x_match = floatEquality(curr_action.fX, curr_drone_action.fX);
						bool y_match = floatEquality(curr_action.fY, curr_drone_action.fY);

						if (!(time_match && x_match && y_match)) {
							printf("UGV and Drone %d are out of sync at end:\n", drone_index);
							printf("UGV Action:\n");
							curr_action.print();
							printf("Drone Action:\n");
							curr_drone_action.print();
							return false;
						}
					}


					break;

				default:
					break;
			}

			// * Final energy check 
			if (floatEquality(ugv_energy, 0.0)) {
				printf("This action caused the UGV to run out energy: \n");
				curr_action.print();
				printf("The UGV energy level: \n");
				std::cout << ugv_energy << std::endl; 
				return false; 
			}

		}
	}

	return true;
}


