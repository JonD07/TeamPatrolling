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
	int mDetails = -1; // Optional metadata used for move to position actions, default to -1 if unused

	// Constructor
	SOCAction(double t, int id, E_SOCActionType type, int subtour, int m = -1)
		: time(t), ID(id), action_type(type), subtour_index(subtour), mDetails(m) {}

	// Copy constructor
	SOCAction(const SOCAction& other)
		: time(other.time), ID(other.ID), action_type(other.action_type),
		  subtour_index(other.subtour_index), mDetails(other.mDetails) {}
};


// Comparison function to SOC Actions by time
struct CompareSOCAction {
	bool operator()(const SOCAction& a1, const SOCAction& a2) {
		return a1.time > a2.time;
	}
};
