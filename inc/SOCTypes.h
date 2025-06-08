enum E_SOCActionType {
	e_LaunchDrone=0,
	e_ReceiveDrone,
	e_BaseStation,
	e_MoveToPosition,
};



// Data structure for a drone sub-tour
struct SubTour {
	// Fixed distance of inner tour
	double tour_dist;
	int ID;
	int launch_ID;
	int land_ID;
	double start_x;
	double start_y;
	double end_x;
	double end_y;

	// Constructor
	SubTour(double dist, int id, double sx, double sy, double ex, double ey) :
			tour_dist(dist), ID(id), start_x(sx), start_y(sy), end_x(ex), end_y(ey) {
		launch_ID = -1;
		land_ID = -1;
	}
	SubTour(const SubTour& other) {
		tour_dist = other.tour_dist;
		ID = other.ID;
		launch_ID = other.launch_ID;
		land_ID = other.land_ID;
		start_x = other.start_x;
		start_y = other.start_y;
		end_x = other.end_x;
		end_y =other.end_y;
	}


};

// Data structure for actions in the SOC program
struct SOCAction {
	double time; // Time that the action is completed
	int ID; // Which drone?
	E_SOCActionType action_type;
	int subtour_index; // Which sub-tour?
	int mDetails = -1; // Optional metadata, usually obstacle ID, defaults to -1 if unused
	double initial_x, initial_y;

	// Constructor
	SOCAction(double t, int id, E_SOCActionType type, int subtour, int m = -1)
		: time(t), ID(id), action_type(type), subtour_index(subtour), mDetails(m) {
		initial_x = 0.0;
		initial_y = 0.0;
	}

	// Copy constructor
	SOCAction(const SOCAction& other)
		: time(other.time), ID(other.ID), action_type(other.action_type),
		  subtour_index(other.subtour_index), mDetails(other.mDetails),
		  initial_x(other.initial_x), initial_y(other.initial_y) {}

	// Constructor from drone action
	SOCAction(const DroneAction& drone_action, int drone_ID, int sub_tour, int details) {
		time = drone_action.fCompletionTime;
		ID = drone_ID;
		subtour_index = sub_tour;
		mDetails = details;
		initial_x = drone_action.fX;
		initial_y = drone_action.fY;

		switch(drone_action.mActionType)
		{
		case E_DroneActionTypes::e_LaunchFromUGV:
			action_type = E_SOCActionType::e_LaunchDrone;
			break;
		case E_DroneActionTypes::e_LandOnUGV:
			action_type = E_SOCActionType::e_ReceiveDrone;
			break;
		default:
			action_type = E_SOCActionType::e_BaseStation;
			break;
		}
	}

	// Constructor from drone action
	SOCAction(const UGVAction& ugv_action, int ugv_ID, int details)
		: time(ugv_action.fCompletionTime), ID(ugv_ID), action_type(E_SOCActionType::e_MoveToPosition),
		  subtour_index(-1), mDetails(details), initial_x(ugv_action.fX), initial_y(ugv_action.fY) {
	}
};


// Comparison function to SOC Actions by time
struct CompareSOCAction {
	bool operator()(const SOCAction& a1, const SOCAction& a2) {
		return a1.time > a2.time;
	}
};
