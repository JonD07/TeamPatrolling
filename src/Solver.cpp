#include "Solver.h"

Solver::Solver() {
}

Solver::~Solver() {}


double Solver::calcChargeTime(double J) {
	double batt_charge = DRONE_TOTAL_BAT - J;
	double time = 0;

	// Will we be fast-charging?
	if(batt_charge < SLOW_CHARGE_POINT) {
		// How many joules can we fast charge?
		double fast_charge_joules = SLOW_CHARGE_POINT - batt_charge;
		// Find roots of polynomial
		Roots roots;
		roots.FindRoots(FAST_CHARGE_A, FAST_CHARGE_B, (-1*fast_charge_joules));
		if(!roots.imaginary) {
			double fast_charge_time = std::max(roots.root1, roots.root2);
			time = fast_charge_time + (T_MAX - T_STAR);
		}
		else {
			// Not expected to be here...
			fprintf(stderr, "[ERROR] : Solver::calcChargeTime() : Roots of charge time are imaginary!\n");
		}
	}
	else {
		// We are in the slow charging range. Find time to charge to where we are..
		double charge_to_current = T_STAR - (log(1+(ALPHA/P_STAR)*(E_STAR - J)))/ALPHA;
		time = T_MAX - charge_to_current;

	}

	return time;
}
