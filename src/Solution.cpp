#include "Solution.h"

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
}

Solution::~Solution() {
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

	// Sanity print
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
		// Record the ID of the first action... we will need this to know when the kernel has restarted
		int first_id = queue.top().ActionID;
		queue.pop();

		if(DEBUG_SOL)
			printf(" (%d:%f)", first_id, prev);

		// While the queue isn't empty and we haven't seen the first action again...
		bool run_again = true;
		while(!queue.empty() && run_again) {
			double next = queue.top().time;
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

// Calculates the Penalty Accumulation Rate of the current solution stored in this solution object.
double Solution::Benchmark() {
	return INF;
}


// Determines if this is a valid assignment solution (doesn't break constraints)
bool Solution::ValidSolution() {
	/// Check each set of constraints
	bool valid = false;

	return valid;
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
		fprintf(stderr, "[ERROR]:Solution::GetLastDroneAction(): Drone action array %d is empty!\n", j);
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
		fprintf(stderr, "[ERROR]:Solution::GetLastUGVAction(): UGV action array %d is empty!\n", j);
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
